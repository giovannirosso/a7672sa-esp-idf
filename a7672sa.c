/**
 * @file       a7672sa.c
 * @brief      ESP-IDF Component for A7672SA LTE Modem
 * @author     Giovanni de Rosso Unruh
 * @date       01/2026
 * @version    2.0.0
 *
 * Pure ESP-IDF implementation (no Arduino dependencies)
 *
 * SPDX-FileCopyrightText: 2026 Mindtech Tecnologia
 * SPDX-FileCopyrightText: 2026 Giovanni de Rosso Unruh
 * SPDX-License-Identifier: MIT
 */

#include "a7672sa.h"
#include "esp_timer.h"

static const char *TAG = "A7672SA";

static a7672sa_handle_t s_instance = NULL;

/* ============================================================================
 * Internal Handle Structure
 * ============================================================================ */

struct a7672sa_handle_s
{
    // Configuration
    a7672sa_config_t config;

    // RTOS handles
    QueueHandle_t uart_queue;
    TaskHandle_t rx_task_handle;
    TaskHandle_t tx_task_handle;
    SemaphoreHandle_t rx_guard;
    SemaphoreHandle_t publish_guard;

    // State flags
    bool mqtt_connected;
    bool http_response_received;
    bool publishing;
    bool at_ok;
    bool at_error;
    bool at_ready;
    bool at_input;
    bool at_publish;
    bool at_subscribed;
    bool silent_mode;
    bool operators_list_updated;
    bool debug_enabled;

    // Response buffer. May contain binary MQTT payload bytes (including 0x00), so the
    // amount of valid data is tracked in at_response_len — strlen() must never be used
    // to measure it. The buffer is still kept NUL-terminated at at_response_len because
    // text-only consumers (AT response parsing, a7672sa_get_response) use str* functions.
    char *at_response;
    size_t at_response_len;
    uint32_t rx_buffer_size;

    // HTTP response data
    a7672sa_http_response_t http_response_data;

    // Registration status
    a7672sa_registration_status_t cs_reg_stat;
    a7672sa_registration_status_t ps_reg_stat;
    a7672sa_registration_status_t eps_reg_stat;

    // PDN status (CIDs 0-10)
    bool pdn_active[11];

    // Operators list
    a7672sa_network_operator_t available_operators[A7672SA_MAX_OPERATORS];
    size_t operators_count;

    // FS file handle (returned by AT+FSOPEN)
    int fs_handle;

    // Last command tracking for error reporting
    char last_command[64];
    char last_raw_response[256];

    // Callbacks
    a7672sa_mqtt_message_cb_t on_message_callback;
    a7672sa_mqtt_status_cb_t on_mqtt_status;
    a7672sa_ps_reg_event_cb_t on_ps_reg_event;
};

/* ============================================================================
 * Helper Functions - Forward Declarations
 * ============================================================================ */

static void rx_task(void *pvParameters);
static void tx_task(void *pvParameters);
static int send_cmd_to_modem(a7672sa_handle_t handle, const char *log_name, const char *data);
static int send_data_to_modem(a7672sa_handle_t handle, const char *log_name, const uint8_t *data,
                              size_t len);
static void response_parser(a7672sa_handle_t handle, const char *data);
static bool wait_for_condition(a7672sa_handle_t handle, uint32_t timeout_ms,
                               bool (*condition_check)(a7672sa_handle_t),
                               const char *operation_name);
static int64_t millis(void);

/* ============================================================================
 * Internal Helper: Get current time in milliseconds
 * ============================================================================ */

static int64_t millis(void) { return esp_timer_get_time() / 1000; }

/* ============================================================================
 * Initialization / Deinitialization
 * ============================================================================ */

esp_err_t a7672sa_init(const a7672sa_config_t *config, a7672sa_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Allocate handle
    a7672sa_handle_t handle = calloc(1, sizeof(struct a7672sa_handle_s));
    if (handle == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate handle");
        return ESP_ERR_NO_MEM;
    }

    // Copy configuration
    memcpy(&handle->config, config, sizeof(a7672sa_config_t));

    // Initialize state
    handle->at_ok = false;
    handle->at_ready = false;
    handle->at_input = false;
    handle->at_publish = false;
    handle->at_subscribed = false;
    handle->mqtt_connected = false;
    handle->http_response_received = false;
    handle->publishing = false;
    handle->operators_list_updated = false;
    handle->silent_mode = false;

    // Initialize command tracking
    handle->last_command[0] = '\0';
    handle->last_raw_response[0] = '\0';

    // Initialize debug mode
#ifdef CONFIG_A7672SA_DEBUG_LOG
    handle->debug_enabled = true;
#else
    handle->debug_enabled = false;
#endif

    handle->cs_reg_stat = A7672SA_REG_UNKNOWN;
    handle->ps_reg_stat = A7672SA_REG_UNKNOWN;
    handle->eps_reg_stat = A7672SA_REG_UNKNOWN;

    // Ensure minimum buffer size
    if (handle->config.rx_buffer_size < 128)
    {
        handle->config.rx_buffer_size = 128;
    }
    handle->rx_buffer_size = handle->config.rx_buffer_size;

    // Configure UART
    const uart_config_t uart_config = {
        .baud_rate = handle->config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err =
        uart_driver_install(handle->config.uart_num, handle->rx_buffer_size * 2, 0, 0, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
        free(handle);
        return err;
    }

    err = uart_param_config(handle->config.uart_num, &uart_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(err));
        uart_driver_delete(handle->config.uart_num);
        free(handle);
        return err;
    }

    err = uart_set_pin(handle->config.uart_num, handle->config.tx_pin, handle->config.rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
        uart_driver_delete(handle->config.uart_num);
        free(handle);
        return err;
    }

    *out_handle = handle;
    s_instance = handle;
    ESP_LOGI(TAG, "A7672SA initialized");
    return ESP_OK;
}

a7672sa_handle_t a7672sa_get_instance(void) { return s_instance; }

esp_err_t a7672sa_begin(a7672sa_handle_t handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Configure enable pin
    gpio_set_direction(handle->config.en_pin, GPIO_MODE_OUTPUT);
    ESP_LOGD(TAG, "Enable Pin: %d", handle->config.en_pin);

    // Power cycle modem
    gpio_set_level(handle->config.en_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(handle->config.en_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Allocate response buffer
    if (handle->at_response == NULL)
    {
        handle->at_response = malloc(handle->rx_buffer_size + 1);
        if (handle->at_response == NULL)
        {
            ESP_LOGE(TAG, "Failed to allocate response buffer");
            return ESP_ERR_NO_MEM;
        }
        memset(handle->at_response, 0, handle->rx_buffer_size + 1);
        handle->at_response_len = 0;
    }

    // Create semaphores
    handle->rx_guard = xSemaphoreCreateMutex();
    handle->publish_guard = xSemaphoreCreateRecursiveMutex();

    // Create command queue
    handle->uart_queue = xQueueCreate(A7672SA_UART_QUEUE_SIZE, sizeof(a7672sa_cmd_message_t));
    if (handle->uart_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create UART queue");
        return ESP_ERR_NO_MEM;
    }

    // Create tasks
    BaseType_t ret;
    ret = xTaskCreate(rx_task, "a7672sa_rx", 4608, handle, configMAX_PRIORITIES - 5,
                      &handle->rx_task_handle);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create RX task");
        return ESP_FAIL;
    }

    ret = xTaskCreate(tx_task, "a7672sa_tx", 4096, handle, configMAX_PRIORITIES - 6,
                      &handle->tx_task_handle);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create TX task");
        vTaskDelete(handle->rx_task_handle);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "A7672SA started");
    return ESP_OK;
}

esp_err_t a7672sa_stop(a7672sa_handle_t handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Disconnect MQTT if connected
    if (handle->mqtt_connected)
    {
        a7672sa_mqtt_disconnect(handle, 1000);
    }

    // Deinitialize UART and tasks
    a7672sa_deinit_uart(handle);

    // Cleanup RTOS objects
    if (handle->uart_queue)
    {
        vQueueDelete(handle->uart_queue);
        handle->uart_queue = NULL;
    }
    if (handle->rx_guard)
    {
        vSemaphoreDelete(handle->rx_guard);
        handle->rx_guard = NULL;
    }
    if (handle->publish_guard)
    {
        vSemaphoreDelete(handle->publish_guard);
        handle->publish_guard = NULL;
    }

    // Put modem in disabled state
    gpio_set_level(handle->config.en_pin, 1);

    ESP_LOGI(TAG, "A7672SA stopped");
    return ESP_OK;
}

esp_err_t a7672sa_deinit(a7672sa_handle_t handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    a7672sa_stop(handle);

    // Free response buffer
    if (handle->at_response != NULL)
    {
        free(handle->at_response);
        handle->at_response = NULL;
    }

    free(handle);
    ESP_LOGI(TAG, "A7672SA deinitialized");
    return ESP_OK;
}

esp_err_t a7672sa_restart(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    a7672sa_send_command(handle, "CFUN=0", "AT+CFUN=0" GSM_NL);
    vTaskDelay(pdMS_TO_TICKS(2000));
    a7672sa_send_command(handle, "RESTART", "AT+CRESET" GSM_NL);

    return a7672sa_wait_response(handle, timeout_ms) ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* ============================================================================
 * Callback Registration
 * ============================================================================ */

esp_err_t a7672sa_set_mqtt_message_callback(a7672sa_handle_t handle,
                                            a7672sa_mqtt_message_cb_t callback)
{
    if (handle == NULL)
        return ESP_ERR_INVALID_ARG;
    handle->on_message_callback = callback;
    return ESP_OK;
}

esp_err_t a7672sa_set_mqtt_status_callback(a7672sa_handle_t handle,
                                           a7672sa_mqtt_status_cb_t callback)
{
    if (handle == NULL)
        return ESP_ERR_INVALID_ARG;
    handle->on_mqtt_status = callback;
    return ESP_OK;
}

esp_err_t a7672sa_set_ps_reg_callback(a7672sa_handle_t handle, a7672sa_ps_reg_event_cb_t callback)
{
    if (handle == NULL)
        return ESP_ERR_INVALID_ARG;
    handle->on_ps_reg_event = callback;
    return ESP_OK;
}

/* ============================================================================
 * Status Functions
 * ============================================================================ */

bool a7672sa_is_ready(a7672sa_handle_t handle) { return handle ? handle->at_ready : false; }

bool a7672sa_ps_ready(a7672sa_handle_t handle)
{
    if (!handle)
        return false;
    return (handle->eps_reg_stat == A7672SA_REG_REGISTERED_HOME ||
            handle->eps_reg_stat == A7672SA_REG_REGISTERED_ROAMING) ||
           (handle->ps_reg_stat == A7672SA_REG_REGISTERED_HOME ||
            handle->ps_reg_stat == A7672SA_REG_REGISTERED_ROAMING);
}

bool a7672sa_cs_ready(a7672sa_handle_t handle)
{
    if (!handle)
        return false;
    return (handle->cs_reg_stat == A7672SA_REG_REGISTERED_HOME ||
            handle->cs_reg_stat == A7672SA_REG_REGISTERED_ROAMING);
}

a7672sa_registration_status_t a7672sa_cs_registration(a7672sa_handle_t handle)
{
    return handle ? handle->cs_reg_stat : A7672SA_REG_UNKNOWN;
}

a7672sa_registration_status_t a7672sa_ps_registration(a7672sa_handle_t handle)
{
    return handle ? handle->ps_reg_stat : A7672SA_REG_UNKNOWN;
}

a7672sa_registration_status_t a7672sa_eps_registration(a7672sa_handle_t handle)
{
    return handle ? handle->eps_reg_stat : A7672SA_REG_UNKNOWN;
}

bool a7672sa_is_pdn_active(a7672sa_handle_t handle, int cid)
{
    if (!handle || cid < 0 || cid > 10)
        return false;
    return handle->pdn_active[cid];
}

/* ============================================================================
 * RX/TX Tasks
 * ============================================================================ */

static void rx_task(void *pvParameters)
{
    a7672sa_handle_t handle = (a7672sa_handle_t)pvParameters;

    if (handle->at_response == NULL)
    {
        handle->at_response = malloc(handle->rx_buffer_size + 1);
        if (handle->at_response == NULL)
        {
            ESP_LOGE(TAG, "RX Task: Failed to allocate response buffer");
            vTaskDelete(NULL);
            return;
        }
        memset(handle->at_response, 0, handle->rx_buffer_size + 1);
        handle->at_response_len = 0;
    }

    uint8_t *rx_buffer = malloc(handle->rx_buffer_size);
    if (rx_buffer == NULL)
    {
        ESP_LOGE(TAG, "RX Task: Failed to allocate RX buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        int len = uart_read_bytes(handle->config.uart_num, rx_buffer, handle->rx_buffer_size - 1,
                                  pdMS_TO_TICKS(100));
        if (len > 0)
        {
            rx_buffer[len] = '\0';

            a7672sa_rx_lock(handle, portMAX_DELAY);

            // Append by explicit length — incoming bytes may be a binary MQTT payload
            // containing 0x00, which strlen/strncat would truncate silently.
            size_t current_len = handle->at_response_len;
            if ((size_t)len > handle->rx_buffer_size - current_len)
            {
                // Accumulator full and nothing consumed it — discard the stale contents
                // so new URCs and MQTT messages can still be received.
                ESP_LOGW(TAG, "RX accumulator full (%u bytes), discarding stale data",
                         (unsigned)current_len);
                current_len = 0;
            }
            memcpy(handle->at_response + current_len, rx_buffer, len);
            handle->at_response_len = current_len + len;
            handle->at_response[handle->at_response_len] = '\0';

            // Parse response
            response_parser(handle, (char *)rx_buffer);

            // Debug logging - show received data
            if (handle->debug_enabled)
            {
                // Clean up the output for display
                char clean_rx[512];
                size_t j = 0;
                for (int i = 0; i < len && j < sizeof(clean_rx) - 1; i++)
                {
                    if (rx_buffer[i] == '\r')
                    {
                        // Skip \r
                    }
                    else if (rx_buffer[i] == '\n')
                    {
                        if (j > 0 && clean_rx[j - 1] != ' ' && clean_rx[j - 1] != '|')
                        {
                            clean_rx[j++] = ' ';
                            clean_rx[j++] = '|';
                            clean_rx[j++] = ' ';
                        }
                    }
                    else if (rx_buffer[i] >= 32 || rx_buffer[i] == '\t')
                    {
                        clean_rx[j++] = rx_buffer[i];
                    }
                }
                // Remove trailing separators
                while (j > 0 && (clean_rx[j - 1] == ' ' || clean_rx[j - 1] == '|'))
                {
                    j--;
                }
                clean_rx[j] = '\0';
                if (j > 0)
                {
                    ESP_LOGD(TAG, "RX: %s", clean_rx);
                }
            }

            a7672sa_rx_unlock(handle);
        }

        taskYIELD();
    }

    free(rx_buffer);
    vTaskDelete(NULL);
}

static void tx_task(void *pvParameters)
{
    a7672sa_handle_t handle = (a7672sa_handle_t)pvParameters;

    // Wait for modem to be ready
    while (!handle->at_ready)
    {
        send_cmd_to_modem(handle, "AT_WAIT", "AT" GSM_NL);
        vTaskDelay(pdMS_TO_TICKS(500));

        if (handle->at_ok)
        {
            handle->at_ready = true;
            ESP_LOGI(TAG, "Modem ready");
        }
    }

    // Configure URCs if not in silent mode
    if (!handle->silent_mode)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        send_cmd_to_modem(handle, "ATE0", "ATE0" GSM_NL);
        vTaskDelay(pdMS_TO_TICKS(500));

        // log_name "CMQTTSTART" (not "AT+CMQTTSTART") to match the response_parser's
        // last_command check that tolerates any error on this specific command.
        send_cmd_to_modem(handle, "CMQTTSTART", "AT+CMQTTSTART" GSM_NL);
        vTaskDelay(pdMS_TO_TICKS(500));

        send_cmd_to_modem(handle, "AT+CGEREP", "AT+CGEREP=2,1" GSM_NL);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Process command queue
    a7672sa_cmd_message_t message;
    while (1)
    {
        if (xQueueReceive(handle->uart_queue, &message, pdMS_TO_TICKS(100)) == pdPASS)
        {
            if (message.data_len > 0)
            {
                send_data_to_modem(handle, message.log_name, message.data, message.data_len);
            }
            else
            {
                send_cmd_to_modem(handle, message.log_name, (char *)message.data);
            }
        }
        taskYIELD();
    }

    vTaskDelete(NULL);
}

/* ============================================================================
 * Command Sending
 * ============================================================================ */

static int send_cmd_to_modem(a7672sa_handle_t handle, const char *log_name, const char *data)
{
    if (!uart_is_driver_installed(handle->config.uart_num))
    {
        return 0;
    }

    // Track last command for error reporting
    strncpy(handle->last_command, log_name, sizeof(handle->last_command) - 1);
    handle->last_command[sizeof(handle->last_command) - 1] = '\0';

    int len = strlen(data);
    int tx_bytes = uart_write_bytes(handle->config.uart_num, data, len);

    // Debug logging - show sent command
    if (handle->debug_enabled)
    {
        // Remove trailing \r\n for cleaner output
        char clean_data[256];
        strncpy(clean_data, data, sizeof(clean_data) - 1);
        clean_data[sizeof(clean_data) - 1] = '\0';
        size_t clean_len = strlen(clean_data);
        while (clean_len > 0 &&
               (clean_data[clean_len - 1] == '\r' || clean_data[clean_len - 1] == '\n'))
        {
            clean_data[--clean_len] = '\0';
        }
        ESP_LOGI(TAG, "TX [%s]: %s", log_name, clean_data);
    }

    return tx_bytes;
}

static int send_data_to_modem(a7672sa_handle_t handle, const char *log_name, const uint8_t *data,
                              size_t len)
{
    if (!uart_is_driver_installed(handle->config.uart_num))
    {
        return 0;
    }

    // Track last command for error reporting (data commands too)
    strncpy(handle->last_command, log_name, sizeof(handle->last_command) - 1);
    handle->last_command[sizeof(handle->last_command) - 1] = '\0';

    int tx_bytes = uart_write_bytes(handle->config.uart_num, data, len);

    // Debug logging - show sent data
    if (handle->debug_enabled)
    {
        if (len <= 64)
        {
            // Show small data as string if printable
            char data_str[128];
            size_t i;
            bool printable = true;
            for (i = 0; i < len && printable; i++)
            {
                if (data[i] < 32 && data[i] != '\r' && data[i] != '\n' && data[i] != '\t')
                {
                    printable = false;
                }
            }
            if (printable)
            {
                memcpy(data_str, data, len);
                data_str[len] = '\0';
                // Remove trailing newlines for cleaner output
                while (len > 0 && (data_str[len - 1] == '\r' || data_str[len - 1] == '\n'))
                {
                    data_str[--len] = '\0';
                }
                ESP_LOGI(TAG, "TX [%s]: %s", log_name, data_str);
            }
            else
            {
                char hex_buf[128];
                size_t show = len < 32 ? len : 32;
                size_t pos = 0;
                for (size_t hi = 0; hi < show && pos + 3 < sizeof(hex_buf); hi++)
                    pos += snprintf(hex_buf + pos, sizeof(hex_buf) - pos, "%02X ", data[hi]);
                ESP_LOGI(TAG, "TX [%s]: %d bytes (binary): %s%s", log_name, (int)len, hex_buf,
                         len > 32 ? "..." : "");
            }
        }
        else
        {
            ESP_LOGI(TAG, "TX [%s]: %d bytes", log_name, (int)len);
        }
    }

    return tx_bytes;
}

esp_err_t a7672sa_send_command(a7672sa_handle_t handle, const char *log_name, const char *data)
{
    if (handle == NULL || handle->uart_queue == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    handle->at_ok = false;

    a7672sa_cmd_message_t message;
    memset(&message, 0, sizeof(message));
    strncpy(message.log_name, log_name, sizeof(message.log_name) - 1);
    strncpy((char *)message.data, data, sizeof(message.data) - 1);
    message.data_len = 0;  // String command

    if (xQueueSend(handle->uart_queue, &message, portMAX_DELAY) != pdPASS)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t a7672sa_send_data(a7672sa_handle_t handle, const char *log_name, const uint8_t *data,
                            size_t len)
{
    if (handle == NULL || handle->uart_queue == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    handle->at_ok = false;

    a7672sa_cmd_message_t message;
    memset(&message, 0, sizeof(message));
    strncpy(message.log_name, log_name, sizeof(message.log_name) - 1);

    size_t copy_len = len < sizeof(message.data) ? len : sizeof(message.data);
    memcpy(message.data, data, copy_len);
    message.data_len = copy_len;

    if (xQueueSend(handle->uart_queue, &message, portMAX_DELAY) != pdPASS)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ============================================================================
 * Response Parser
 * ============================================================================ */

static void on_ps_lost(a7672sa_handle_t handle)
{
    if (handle->mqtt_connected)
    {
        handle->mqtt_connected = false;
        if (handle->on_mqtt_status)
        {
            handle->on_mqtt_status(A7672SA_MQTT_DISCONNECTED);
        }
    }
}

static int parse_cid_from_cgev(const char *p)
{
    while (*p && !(*p >= '0' && *p <= '9'))
        p++;
    int cid = 0;
    if (*p)
        cid = atoi(p);
    return cid;
}

// memmem-style search over a buffer that may contain embedded NUL bytes (binary MQTT
// payloads) — strstr would stop at the first 0x00 and miss markers after it.
static const char *buf_find(const char *buf, size_t buf_len, const char *needle)
{
    size_t needle_len = strlen(needle);
    if (needle_len == 0 || buf_len < needle_len)
    {
        return NULL;
    }
    for (size_t i = 0; i + needle_len <= buf_len; i++)
    {
        if (buf[i] == needle[0] && memcmp(buf + i, needle, needle_len) == 0)
        {
            return buf + i;
        }
    }
    return NULL;
}

static void response_parser(a7672sa_handle_t handle, const char *data)
{
    // Skip empty lines
    if (strcmp(data, GSM_NL) == 0)
    {
        return;
    }

    // Input/Download prompt — checked first and unconditionally (not inside the CMQTTRXSTART
    // block below, which returns early). An incoming subscribed-message URC can land in the same
    // UART chunk as the '>' prompt for our own AT+CMQTTTOPIC/AT+CMQTTPAYLOAD command (the modem
    // multiplexes URCs and command responses on one UART). If the prompt check only ran after the
    // RXSTART/RXEND accumulation logic, that early return would silently drop the prompt and
    // a7672sa_wait_input() would time out ("No input prompt for topic").
    if (strstr(data, GSM_NL ">") || strstr(data, "\n>") ||
        (data[0] == '>' && (data[1] == '\0' || data[1] == ' ')))
    {
        handle->at_input = true;
    }
    if (strstr(data, GSM_NL "DOWNLOAD") || strstr(data, "\nDOWNLOAD"))
    {
        handle->at_input = true;
    }

    // MQTT message received — modem sends the message across multiple UART frames:
    //   +CMQTTRXSTART: <client>,<topic_len>,<payload_len>
    //   +CMQTTRXTOPIC: <client>,<topic_len>
    //   <topic>
    //   +CMQTTRXPAYLOAD: <client>,<payload_len>
    //   <payload>
    //   +CMQTTRXEND: <client>
    //
    // Parse from the accumulated at_response buffer (not the current chunk) so that
    // multi-read messages are handled correctly. The payload is raw binary (Protobuf)
    // and may contain any byte value including 0x00, so everything here must be
    // length-based (buf_find/memchr/memcpy) — str* functions would truncate at NULs.
    // Completeness is decided by byte counts from the header, not by searching for
    // CMQTTRXEND (whose marker could also appear inside a binary payload).
    if (buf_find(handle->at_response, handle->at_response_len, "CMQTTRXSTART:"))
    {
        // Drain every complete message in the buffer; leave a partial one for the
        // next UART chunk.
        while (1)
        {
            const char *acc = handle->at_response;
            const char *acc_end = acc + handle->at_response_len;

            const char *rx_start = buf_find(acc, handle->at_response_len, "CMQTTRXSTART:");
            if (rx_start == NULL)
            {
                break;
            }

            // Header lines are plain ASCII, and the buffer is NUL-terminated, so
            // sscanf is safe on them.
            int client_id = 0, topic_len = 0, payload_len = 0;
            if (sscanf(rx_start, "CMQTTRXSTART: %d,%d,%d", &client_id, &topic_len, &payload_len) <
                3)
            {
                return;  // header line not fully received yet
            }

            // Corrupted header (UART noise): lengths that can never fit would stall
            // the parser forever waiting for completion. Drop this message.
            if (topic_len <= 0 || payload_len < 0 ||
                (size_t)topic_len + (size_t)payload_len > handle->rx_buffer_size)
            {
                ESP_LOGE(TAG, "MQTT RX: bad header (topic_len=%d, payload_len=%d), dropping",
                         topic_len, payload_len);
                size_t drop = (rx_start - acc) + 13;  // past "CMQTTRXSTART:"
                size_t tail = handle->at_response_len - drop;
                memmove(handle->at_response, acc + drop, tail);
                handle->at_response_len = tail;
                handle->at_response[tail] = '\0';
                continue;
            }

            const char *topic_marker = buf_find(rx_start, acc_end - rx_start, "CMQTTRXTOPIC:");
            if (topic_marker == NULL)
            {
                return;
            }
            const char *topic_start = memchr(topic_marker, '\n', acc_end - topic_marker);
            if (topic_start == NULL)
            {
                return;
            }
            topic_start++;
            if (acc_end - topic_start < topic_len)
            {
                return;  // topic bytes not fully received yet
            }

            char topic[128] = {0};
            int topic_copy =
                topic_len < (int)sizeof(topic) - 1 ? topic_len : (int)sizeof(topic) - 1;
            memcpy(topic, topic_start, topic_copy);

            const char *cursor = topic_start + topic_len;

            // Payload section is omitted by the modem when payload_len == 0
            char payload[512];
            int payload_copy = 0;
            if (payload_len > 0)
            {
                const char *payload_marker = buf_find(cursor, acc_end - cursor, "CMQTTRXPAYLOAD:");
                if (payload_marker == NULL)
                {
                    return;
                }
                // Skip exactly the \n that ends the +CMQTTRXPAYLOAD: header line —
                // the very next byte is payload data (may itself be 0x0A).
                const char *payload_start = memchr(payload_marker, '\n', acc_end - payload_marker);
                if (payload_start == NULL)
                {
                    return;
                }
                payload_start++;
                if (acc_end - payload_start < payload_len)
                {
                    return;  // payload bytes not fully received yet
                }

                payload_copy =
                    payload_len < (int)sizeof(payload) ? payload_len : (int)sizeof(payload);
                memcpy(payload, payload_start, payload_copy);
                if (payload_copy < payload_len)
                {
                    ESP_LOGE(TAG, "MQTT payload truncated: %d bytes > %d buffer", payload_len,
                             (int)sizeof(payload));
                }

                ESP_LOGI(TAG, "MQTT payload: %d bytes (expected %d), hex[0..3]=%02X %02X %02X %02X",
                         payload_copy, payload_len, payload_copy > 0 ? (uint8_t)payload[0] : 0,
                         payload_copy > 1 ? (uint8_t)payload[1] : 0,
                         payload_copy > 2 ? (uint8_t)payload[2] : 0,
                         payload_copy > 3 ? (uint8_t)payload[3] : 0);

                cursor = payload_start + payload_len;
            }

            const char *end_marker = buf_find(cursor, acc_end - cursor, "CMQTTRXEND:");
            if (end_marker == NULL)
            {
                return;
            }
            const char *end_nl = memchr(end_marker, '\n', acc_end - end_marker);
            if (end_nl == NULL)
            {
                return;  // RXEND line not fully received yet
            }

            if (handle->on_message_callback && topic_copy > 0)
            {
                a7672sa_mqtt_message_t msg = {
                    .topic = topic, .payload = (uint8_t *)payload, .length = (size_t)payload_copy};
                ESP_LOGI(TAG, "MQTT Message: topic='%s', payload=%d bytes", topic, payload_copy);
                handle->on_message_callback(&msg);
            }

            // Consume the processed message (and anything before it) from the buffer
            size_t consumed = (end_nl + 1) - acc;
            size_t tail = handle->at_response_len - consumed;
            memmove(handle->at_response, acc + consumed, tail);
            handle->at_response_len = tail;
            handle->at_response[tail] = '\0';
        }
        return;
    }

    // Asynchronous MQTT state URCs can share a UART chunk with any other response — seen in
    // the field: "+CMQTTSUB: 0,24 / +CMQTTCONNLOST: 0,1 / +CMQTTTOPIC: 0,11 / ERROR" in a
    // single read. Inside the else-if chain below, an earlier match (ERROR, CMQTTSUB:) swallows
    // them, so a lost connection was never noticed and mqtt_connected stayed true forever
    // (every publish failing, no reconnect). Each one is checked on its own here instead.
    if (strstr(data, "CMQTTCONNLOST:"))
    {
        handle->mqtt_connected = false;
        if (handle->on_mqtt_status)
        {
            handle->on_mqtt_status(A7672SA_MQTT_DISCONNECTED);
        }
        ESP_LOGW(TAG, "MQTT Connection lost");
    }
    if (strstr(data, "CMQTTDISC:"))
    {
        handle->mqtt_connected = false;
        ESP_LOGI(TAG, "MQTT Disconnected");
        if (handle->on_mqtt_status)
        {
            handle->on_mqtt_status(A7672SA_MQTT_DISCONNECTED);
        }
        handle->at_ok = true;
    }
    {
        const char *p = strstr(data, "CMQTTSUB:");
        int client_id, result;
        if (p && sscanf(p, "CMQTTSUB: %d,%d", &client_id, &result) == 2 && result != 0)
        {
            ESP_LOGE(TAG, "MQTT Subscribe failed [%s]: error code %d", handle->last_command,
                     result);
            handle->at_error = true;
        }
    }

    // MQTT Connected
    if (strstr(data, "CMQTTCONNECT: 0,0" GSM_NL))
    {
        handle->mqtt_connected = true;
        if (handle->on_mqtt_status)
        {
            handle->on_mqtt_status(A7672SA_MQTT_CONNECTED);
        }
        ESP_LOGI(TAG, "MQTT Connected");
    }
    // MQTT Start error (already started) — kept for the URC-prefixed form; the general,
    // last_command-based check further below also catches the plain-ERROR form.
    else if (strstr(data, "CMQTTSTART: 19" GSM_NL))
    {
        ESP_LOGW(TAG, "MQTT already started");
        handle->at_ok = true;
    }
    // SIM removed
    else if (strstr(data, "CPIN: SIM REMOVED" GSM_NL))
    {
        ESP_LOGE(TAG, "SIM card removed!");
    }
    // Module ready
    else if (strstr(data, "CFUN: 1" GSM_NL))
    {
        handle->at_ready = true;
    }
    // MQTT Publish success
    else if (strstr(data, "CMQTTPUB: 0,0" GSM_NL))
    {
        handle->at_publish = true;
        ESP_LOGD(TAG, "MQTT Publish OK");
    }
    // MQTT Subscribe success — async URC, arrives after the AT+CMQTTSUB=0 "OK" ack
    else if (strstr(data, "CMQTTSUB: 0,0" GSM_NL))
    {
        handle->at_subscribed = true;
    }

    // CMQTTSTART failing is always non-fatal: worst case is the MQTT service was already
    // running — sometimes reported as the URC "+CMQTTSTART: 19" (handled above), but a
    // client already ACCQ'd/CONNECTED from a previous session can also make the modem
    // reply with a plain ERROR. This happens whenever the modem itself wasn't power-cycled
    // (e.g. esp_restart() only resets the ESP32, not the A7672SA) and still holds state
    // from before the reboot. Either way there's nothing to recover from — the pre-existing
    // MQTT service can just be reused as-is.
    if (strstr(data, GSM_ERROR) && strcmp(handle->last_command, "CMQTTSTART") == 0)
    {
        ESP_LOGW(TAG, "CMQTTSTART error ignored (MQTT service likely already running)");
        handle->at_error = false;
        handle->at_ok = true;
    }
    else if (strstr(data, GSM_ERROR))
    {
        handle->at_error = true;
        handle->at_ok = true;
        // Store raw response for debugging
        strncpy(handle->last_raw_response, data, sizeof(handle->last_raw_response) - 1);
        handle->last_raw_response[sizeof(handle->last_raw_response) - 1] = '\0';
        ESP_LOGE(TAG, "AT Error on command [%s]", handle->last_command);
#if defined(CONFIG_A7672SA_DEBUG_LOG)
        {
            size_t rlen = strlen(data);
            char hex_buf[256];
            size_t show = rlen < 64 ? rlen : 64;
            size_t pos = 0;
            for (size_t ri = 0; ri < show && pos + 3 < sizeof(hex_buf); ri++)
                pos += snprintf(hex_buf + pos, sizeof(hex_buf) - pos, "%02X ", (uint8_t)data[ri]);
            ESP_LOGI(TAG, "Modem response (%d bytes): %s%s", (int)rlen, hex_buf,
                     rlen > 64 ? "..." : "");
        }
#endif
    }
    // CME Error
    else if (strstr(data, "CME ERROR:"))
    {
        handle->at_error = true;
        handle->at_ok = true;
        // Extract error code/message
        const char *err_start = strstr(data, "CME ERROR:");
        char err_msg[64] = {0};
        if (err_start)
        {
            err_start += 10;  // Skip "CME ERROR:"
            while (*err_start == ' ')
                err_start++;
            int i = 0;
            while (*err_start && *err_start != '\r' && *err_start != '\n' && i < 63)
            {
                err_msg[i++] = *err_start++;
            }
        }
        strncpy(handle->last_raw_response, data, sizeof(handle->last_raw_response) - 1);
        handle->last_raw_response[sizeof(handle->last_raw_response) - 1] = '\0';
        ESP_LOGE(TAG, "CME Error on command [%s]: %s", handle->last_command, err_msg);
    }
    // CMS Error (SMS related)
    else if (strstr(data, "CMS ERROR:"))
    {
        handle->at_error = true;
        handle->at_ok = true;
        const char *err_start = strstr(data, "CMS ERROR:");
        char err_msg[64] = {0};
        if (err_start)
        {
            err_start += 10;
            while (*err_start == ' ')
                err_start++;
            int i = 0;
            while (*err_start && *err_start != '\r' && *err_start != '\n' && i < 63)
            {
                err_msg[i++] = *err_start++;
            }
        }
        ESP_LOGE(TAG, "CMS Error on command [%s]: %s", handle->last_command, err_msg);
    }
    // MQTT specific errors
    else if (strstr(data, "CMQTT"))
    {
        // Check for MQTT error responses like +CMQTTCONNECT: 0,X where X != 0
        const char *p = NULL;

        if ((p = strstr(data, "CMQTTCONNECT:")) != NULL)
        {
            int client_id, result;
            if (sscanf(p, "CMQTTCONNECT: %d,%d", &client_id, &result) == 2 && result != 0)
            {
                const char *mqtt_err = "Unknown";
                switch (result)
                {
                    case 1:
                        mqtt_err = "Connection Refused - unacceptable protocol";
                        break;
                    case 2:
                        mqtt_err = "Connection Refused - identifier rejected";
                        break;
                    case 3:
                        mqtt_err = "Connection Refused - server unavailable";
                        break;
                    case 4:
                        mqtt_err = "Connection Refused - bad username/password";
                        break;
                    case 5:
                        mqtt_err = "Connection Refused - not authorized";
                        break;
                    case 6:
                        mqtt_err = "Connection timeout";
                        break;
                }
                ESP_LOGE(TAG, "MQTT Connect failed [%s]: %s (code %d)", handle->last_command,
                         mqtt_err, result);
                handle->at_error = true;
                handle->at_ok = true;
            }
        }
        else if ((p = strstr(data, "CMQTTPUB:")) != NULL)
        {
            int client_id, result;
            if (sscanf(p, "CMQTTPUB: %d,%d", &client_id, &result) == 2 && result != 0)
            {
                ESP_LOGE(TAG, "MQTT Publish failed [%s]: error code %d", handle->last_command,
                         result);
                handle->at_error = true;
            }
        }
        // CMQTTSUB failures, CMQTTCONNLOST and CMQTTDISC are handled before this chain (see
        // the state-URC block above).
        else if ((p = strstr(data, "CMQTTSTART:")) != NULL)
        {
            int result;
            if (sscanf(p, "CMQTTSTART: %d", &result) == 1 && result != 0 && result != 19)
            {
                // 19 = already started, not an error
                ESP_LOGE(TAG, "MQTT Start failed [%s]: error code %d", handle->last_command,
                         result);
                handle->at_error = true;
                handle->at_ok = true;
            }
        }
        else if ((p = strstr(data, "CMQTTREL:")) != NULL)
        {
            int client_id, result;
            if (sscanf(p, "CMQTTREL: %d,%d", &client_id, &result) == 2 && result != 0 &&
                result != 20)
            {
                // 20 = client not acquired — expected on first boot, not an error
                ESP_LOGE(TAG, "MQTT Release failed [%s]: error code %d", handle->last_command,
                         result);
                handle->at_error = true;
                handle->at_ok = true;
            }
        }
    }
    // PB Done (phonebook ready = modem ready)
    else if (strstr(data, GSM_NL "PB DONE"))
    {
        handle->at_ready = true;
    }
    // HTTP Action response
    else if (strstr(data, "HTTPACTION:"))
    {
        const char *p = strstr(data, "HTTPACTION:");
        if (p)
        {
            int method, status, size;
            if (sscanf(p, "HTTPACTION: %d,%d,%d", &method, &status, &size) == 3)
            {
                handle->http_response_data.status_code = status;
                handle->http_response_data.content_size = size;
                handle->http_response_received = true;
            }
        }
    }
    // HTTP ReadFile response (+HTTPREADFILE: <errcode>)
    else if (strstr(data, "HTTPREADFILE:"))
    {
        const char *p = strstr(data, "HTTPREADFILE:");
        int errcode = -1;
        sscanf(p, "HTTPREADFILE: %d", &errcode);
        if (errcode == 0)
        {
            handle->at_ok = true;
        }
        else
        {
            handle->at_error = true;
            handle->at_ok = true;
            ESP_LOGE(TAG, "HTTPREADFILE error: %d", errcode);
        }
    }
    // HTTP Header
    else if (strstr(data, "HTTPHEAD:"))
    {
        const char *p = strstr(data, "HTTPHEAD:");
        if (p)
        {
            int size;
            if (sscanf(p, "HTTPHEAD: %d", &size) == 1)
            {
                handle->http_response_data.header_size = size;
            }

            // Parse ETag if present
            const char *etag = strstr(data, "ETag:");
            if (etag)
            {
                etag += 5;
                while (*etag == ' ' || *etag == '"')
                    etag++;
                int i = 0;
                while (*etag && *etag != '"' && *etag != '\r' && i < 32)
                {
                    handle->http_response_data.etag[i++] = *etag++;
                }
                handle->http_response_data.etag[i] = '\0';
            }
        }
    }
    // Ping response
    else if (strstr(data, "CPING:"))
    {
        const char *p = strstr(data, "CPING:");
        if (p && strstr(p, ",1,"))
        {
            handle->at_ok = true;
        }
    }
    // EPS PDN Activate
    else if (strstr(data, "CGEV: EPS PDN ACT"))
    {
        int cid = parse_cid_from_cgev(data);
        if (cid >= 0 && cid <= 10)
        {
            handle->pdn_active[cid] = true;
        }
    }
    // EPS PDN Deactivate
    else if (strstr(data, "CGEV: EPS PDN DEACT"))
    {
        int cid = parse_cid_from_cgev(data);
        if (cid >= 0 && cid <= 10)
        {
            handle->pdn_active[cid] = false;
        }
        on_ps_lost(handle);
    }
    // NW PDN Activate
    else if (strstr(data, "CGEV: NW PDN ACT"))
    {
        int cid = parse_cid_from_cgev(data);
        if (cid >= 0 && cid <= 10)
        {
            handle->pdn_active[cid] = true;
        }
    }
    // NW PDN Deactivate
    else if (strstr(data, "CGEV: NW PDN DEACT"))
    {
        int cid = parse_cid_from_cgev(data);
        if (cid >= 0 && cid <= 10)
        {
            handle->pdn_active[cid] = false;
        }
        on_ps_lost(handle);
    }
    // ME Deactivate
    else if (strstr(data, "CGEV: ME DEACT"))
    {
        int cid = parse_cid_from_cgev(data);
        if (cid >= 0 && cid <= 10)
        {
            handle->pdn_active[cid] = false;
        }
    }
    // ME Detach
    else if (strstr(data, "CGEV: ME DETACH"))
    {
        for (int i = 0; i <= 10; i++)
        {
            handle->pdn_active[i] = false;
        }
        on_ps_lost(handle);
    }
    // ME PDN Activate
    else if (strstr(data, "CGEV: ME PDN ACT"))
    {
        int cid = parse_cid_from_cgev(data);
        if (cid >= 0 && cid <= 10)
        {
            handle->pdn_active[cid] = true;
        }
    }
    // ME PDN Deactivate
    else if (strstr(data, "CGEV: ME PDN DEACT"))
    {
        int cid = parse_cid_from_cgev(data);
        if (cid >= 0 && cid <= 10)
        {
            handle->pdn_active[cid] = false;
        }
        on_ps_lost(handle);
    }
    // CS Registration (CREG)
    else if (strstr(data, "CREG: "))
    {
        const char *p = strstr(data, "CREG: ");
        if (p)
        {
            int stat;
            if (sscanf(p, "CREG: %*d,%d", &stat) == 1 || sscanf(p, "CREG: %d", &stat) == 1)
            {
                handle->cs_reg_stat = (a7672sa_registration_status_t)stat;
            }
        }
    }
    // EPS Registration (CEREG)
    else if (strstr(data, "CEREG: "))
    {
        const char *p = strstr(data, "CEREG: ");
        if (p)
        {
            int stat;
            bool before = a7672sa_ps_ready(handle);
            if (sscanf(p, "CEREG: %*d,%d", &stat) == 1 || sscanf(p, "CEREG: %d", &stat) == 1)
            {
                handle->eps_reg_stat = (a7672sa_registration_status_t)stat;
            }
            if (before && !a7672sa_ps_ready(handle))
            {
                on_ps_lost(handle);
            }
            if (handle->on_ps_reg_event)
            {
                handle->on_ps_reg_event(handle->eps_reg_stat);
            }
        }
    }
    // PS Registration (CGREG)
    else if (strstr(data, "CGREG: "))
    {
        const char *p = strstr(data, "CGREG: ");
        if (p)
        {
            int stat;
            bool before = a7672sa_ps_ready(handle);
            if (sscanf(p, "CGREG: %*d,%d", &stat) == 1 || sscanf(p, "CGREG: %d", &stat) == 1)
            {
                handle->ps_reg_stat = (a7672sa_registration_status_t)stat;
            }
            if (before && !a7672sa_ps_ready(handle))
            {
                on_ps_lost(handle);
            }
        }
    }
    // OK response
    else if (strstr(data, GSM_OK))
    {
        handle->at_ok = true;
    }
    else
    {
        ESP_LOGD(TAG, "Unhandled: %s", data);
    }
}

/* ============================================================================
 * Wait Functions
 * ============================================================================ */

static bool check_at_response(a7672sa_handle_t handle) { return handle->at_ok || handle->at_error; }

static bool check_at_input(a7672sa_handle_t handle) { return handle->at_input || handle->at_error; }

static bool check_at_publish(a7672sa_handle_t handle)
{
    return handle->at_publish || handle->at_error;
}

static bool check_at_subscribed(a7672sa_handle_t handle)
{
    return handle->at_subscribed || handle->at_error;
}

// Alguns firmwares do A7672SA (visto em bancada, mesmo com a inscrição já
// confirmada no broker) nunca emitem o "OK" síncrono do AT+CMQTTSUB=0,
// contrariando o manual AT — só chega a URC +CMQTTSUB: 0,<err>. Aceitar
// qualquer um dos dois sinais evita esperar por um ACK que pode nunca vir.
static bool check_at_ok_or_subscribed(a7672sa_handle_t handle)
{
    return handle->at_ok || handle->at_subscribed || handle->at_error;
}

static bool wait_for_condition(a7672sa_handle_t handle, uint32_t timeout_ms,
                               bool (*condition_check)(a7672sa_handle_t),
                               const char *operation_name)
{
    int64_t start = millis();
    ESP_LOGD(TAG, "Waiting for %s, timeout: %lu ms", operation_name, timeout_ms);

    while (!condition_check(handle) && (millis() - start) < timeout_ms)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    bool result = condition_check(handle);
    ESP_LOGD(TAG, "Wait for %s: %s", operation_name, result ? "OK" : "FAIL");
    return result;
}

bool a7672sa_wait_response(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;

    // Sample flags before clearing: a very fast response may arrive before this call
    bool early_ok = handle->at_ok;
    bool early_error = handle->at_error;

    handle->at_input = false;
    handle->at_publish = false;
    handle->at_ok = false;
    handle->at_error = false;
    handle->http_response_received = false;

    if (early_ok)
        return !early_error;  // success only if no error was also set

    if (early_error)
        return false;

    bool result = wait_for_condition(handle, timeout_ms, check_at_response, "AT RESPONSE");

    if (handle->at_error)
    {
        handle->at_ok = false;
        handle->at_error = false;
        return false;
    }

    handle->at_ok = false;
    return result;
}

bool a7672sa_wait_input(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;

    // Sample flags before clearing: a very fast response may arrive before this call
    bool early_input = handle->at_input;
    bool early_error = handle->at_error;

    handle->at_ok = false;
    handle->at_publish = false;
    handle->at_error = false;
    handle->http_response_received = false;

    if (early_input)
        return true;

    if (early_error)
        return false;

    bool result = wait_for_condition(handle, timeout_ms, check_at_input, "AT INPUT");

    if (handle->at_error)
    {
        handle->at_input = false;
        handle->at_error = false;
        return false;
    }

    handle->at_input = false;
    return result;
}

/* ============================================================================
 * Mutex Functions
 * ============================================================================ */

void a7672sa_rx_lock(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle && handle->rx_guard)
    {
        TickType_t ticks =
            (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
        xSemaphoreTake(handle->rx_guard, ticks);
    }
}

void a7672sa_rx_unlock(a7672sa_handle_t handle)
{
    if (handle && handle->rx_guard)
    {
        xSemaphoreGive(handle->rx_guard);
    }
}

bool a7672sa_publish_lock(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL || handle->publish_guard == NULL)
    {
        return false;
    }

    TickType_t ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    // Don't block in RX/TX task context
    TaskHandle_t cur = xTaskGetCurrentTaskHandle();
    if (cur == handle->rx_task_handle || cur == handle->tx_task_handle)
    {
        ticks = 0;
    }

    if (xSemaphoreTakeRecursive(handle->publish_guard, ticks) == pdPASS)
    {
        handle->publishing = true;
        return true;
    }

    return false;
}

void a7672sa_publish_unlock(a7672sa_handle_t handle)
{
    if (handle && handle->publish_guard)
    {
        xSemaphoreGiveRecursive(handle->publish_guard);
        handle->publishing = false;
    }
}

/* ============================================================================
 * UART Management
 * ============================================================================ */

esp_err_t a7672sa_reinit_uart(a7672sa_handle_t handle, uint32_t buffer_size, bool at_ready)
{
    if (handle == NULL)
        return ESP_ERR_INVALID_ARG;

    handle->at_ready = at_ready;
    handle->rx_buffer_size = buffer_size;

    const uart_config_t uart_config = {
        .baud_rate = handle->config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_is_driver_installed(handle->config.uart_num))
    {
        uart_wait_tx_done(handle->config.uart_num, pdMS_TO_TICKS(100));
        uart_driver_delete(handle->config.uart_num);
    }

    ESP_ERROR_CHECK(uart_param_config(handle->config.uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(handle->config.uart_num, handle->config.tx_pin,
                                 handle->config.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    esp_err_t err =
        uart_driver_install(handle->config.uart_num, handle->rx_buffer_size, 0, 0, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "reinit_uart: uart_driver_install falhou: %s", esp_err_to_name(err));
        return err;
    }

    // Resize response buffer
    if (handle->at_response != NULL)
    {
        free(handle->at_response);
    }
    handle->at_response = malloc(handle->rx_buffer_size + 1);
    if (handle->at_response)
    {
        memset(handle->at_response, 0, handle->rx_buffer_size + 1);
    }
    handle->at_response_len = 0;

    // Recreate tasks
    xTaskCreate(rx_task, "a7672sa_rx", 4608, handle, configMAX_PRIORITIES - 10,
                &handle->rx_task_handle);
    xTaskCreate(tx_task, "a7672sa_tx", 4096, handle, configMAX_PRIORITIES - 11,
                &handle->tx_task_handle);

    return ESP_OK;
}

esp_err_t a7672sa_deinit_uart(a7672sa_handle_t handle)
{
    if (handle == NULL)
        return ESP_ERR_INVALID_ARG;

    a7672sa_rx_lock(handle, portMAX_DELAY);

    if (handle->rx_task_handle)
    {
        vTaskDelete(handle->rx_task_handle);
        handle->rx_task_handle = NULL;
    }
    if (handle->tx_task_handle)
    {
        vTaskDelete(handle->tx_task_handle);
        handle->tx_task_handle = NULL;
    }

    if (uart_is_driver_installed(handle->config.uart_num))
    {
        uart_wait_tx_done(handle->config.uart_num, pdMS_TO_TICKS(1000));
        uart_flush(handle->config.uart_num);
        uart_driver_delete(handle->config.uart_num);
    }

    a7672sa_rx_unlock(handle);

    return ESP_OK;
}

esp_err_t a7672sa_resize_uart(a7672sa_handle_t handle, uint32_t buffer_size)
{
    if (handle == NULL)
        return ESP_ERR_INVALID_ARG;

    // Delete tasks WITHOUT acquiring rx_guard first.
    //
    // Acquiring rx_guard here would deadlock: rx_task (priority 20) and
    // ota_lte_task (priority 5) — even when rx_task holds the guard only
    // briefly, taskYIELD() does not let a lower-priority task preempt, so
    // ota_lte_task can never take the semaphore.
    //
    // It is safe to vTaskDelete without the guard because:
    //  - After fs_open returns, no further AT responses are expected.
    //  - a7672sa_fs_read uses uart_read_bytes directly and never touches
    //    handle->at_response, so a partially-written buffer doesn't matter.
    //  - The UART driver is kept running to avoid TX pin glitches that
    //    would leave the modem unresponsive.
    if (handle->rx_task_handle)
    {
        vTaskDelete(handle->rx_task_handle);
        handle->rx_task_handle = NULL;
    }

    if (handle->tx_task_handle)
    {
        vTaskDelete(handle->tx_task_handle);
        handle->tx_task_handle = NULL;
    }

    // rx_task may have been inside uart_read_bytes (holding the driver's
    // internal rx_mux) when deleted. uart_flush_input calls xSemaphoreTake
    // on that mutex and would block forever on a deleted task's lock.
    //
    // uart_driver_delete uses vSemaphoreDelete (NOT xSemaphoreTake), so it
    // destroys the stuck rx_mux unconditionally. After install, the new
    // driver has a fresh unlocked rx_mux and uart_flush_input is safe.
    if (uart_is_driver_installed(handle->config.uart_num))
    {
        uart_wait_tx_done(handle->config.uart_num, pdMS_TO_TICKS(100));
        uart_driver_delete(handle->config.uart_num);
    }

    // Allow the modem to settle after the driver teardown.
    vTaskDelay(pdMS_TO_TICKS(200));

    // Reconfigure UART hardware parameters and GPIO routing before reinstalling
    // the driver. uart_driver_delete does NOT preserve the GPIO matrix routing
    // for RX — without uart_set_pin the RX pin stops being routed to the UART
    // peripheral, so uart_read_bytes receives 0 bytes even though uart_write_bytes
    // works (TX GPIO keeps its route through the GPIO matrix).
    const uart_config_t uart_cfg = {
        .baud_rate = handle->config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = uart_param_config(handle->config.uart_num, &uart_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "resize_uart: uart_param_config falhou: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(handle->config.uart_num, handle->config.tx_pin, handle->config.rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "resize_uart: uart_set_pin falhou: %s", esp_err_to_name(ret));
        return ret;
    }

    uint32_t use_size = (buffer_size > 0) ? buffer_size : handle->rx_buffer_size;
    handle->rx_buffer_size = use_size;
    ret = uart_driver_install(handle->config.uart_num, use_size, 0, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "resize_uart: uart_driver_install falhou: %s", esp_err_to_name(ret));
        return ret;
    }
    uart_flush_input(handle->config.uart_num);

    ESP_LOGI(TAG, "resize_uart: OK (buf=%lu)", (unsigned long)use_size);
    return ESP_OK;
}

void a7672sa_set_silent_mode(a7672sa_handle_t handle, bool mode)
{
    if (handle)
    {
        handle->silent_mode = mode;
    }
}

void a7672sa_set_debug(a7672sa_handle_t handle, bool enable)
{
    if (handle)
    {
        handle->debug_enabled = enable;
        ESP_LOGI(TAG, "Debug mode %s", enable ? "ENABLED" : "DISABLED");
    }
}

bool a7672sa_is_debug_enabled(a7672sa_handle_t handle)
{
    return handle ? handle->debug_enabled : false;
}

const char *a7672sa_get_response(a7672sa_handle_t handle)
{
    return handle ? handle->at_response : NULL;
}

/* ============================================================================
 * AT Command Functions
 * ============================================================================ */

bool a7672sa_test_at(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    a7672sa_send_command(handle, "AT_TEST", "AT" GSM_NL);
    return a7672sa_wait_response(handle, timeout_ms);
}

bool a7672sa_sim_ready(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;

    a7672sa_send_command(handle, "SIM_READY", "AT+CPIN?" GSM_NL);
    if (!a7672sa_wait_response(handle, timeout_ms))
        return false;

    const char *response = a7672sa_get_response(handle);
    return response && strstr(response, "+CPIN: READY") != NULL;
}

int a7672sa_signal_quality(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL)
        return 0;
    if (handle->publishing)
        return 0;

    a7672sa_send_command(handle, "SIGNAL_QUALITY", "AT+CSQ" GSM_NL);
    if (a7672sa_wait_response(handle, timeout_ms))
    {
        const char *response = a7672sa_get_response(handle);
        if (response)
        {
            const char *p = strstr(response, "+CSQ:");
            if (p)
            {
                int rssi, ber;
                if (sscanf(p, "+CSQ: %d,%d", &rssi, &ber) >= 1)
                {
                    return rssi;
                }
            }
        }
    }
    return 0;
}

bool a7672sa_ping(a7672sa_handle_t handle, const char *host, uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;
    if (handle->publishing)
        return false;

    if (!a7672sa_ps_ready(handle))
    {
        ESP_LOGW(TAG, "PS not ready for ping");
        return false;
    }

    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CPING=\"%s\",1" GSM_NL, host);
    a7672sa_send_command(handle, "PING", cmd);
    return a7672sa_wait_response(handle, timeout_ms);
}

/* ============================================================================
 * Network Configuration
 * ============================================================================ */

bool a7672sa_set_network_mode(a7672sa_handle_t handle, a7672sa_network_mode_t mode,
                              uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;
    if (handle->publishing)
        return false;

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CNMP=%d" GSM_NL, mode);
    a7672sa_send_command(handle, "SET_NETWORK_MODE", cmd);
    return a7672sa_wait_response(handle, timeout_ms);
}

bool a7672sa_set_apn(a7672sa_handle_t handle, const char *apn, const char *user,
                     const char *password, uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;
    if (handle->publishing)
        return false;

    // Enable URCs
    a7672sa_send_command(handle, "SET_APN", "AT+CREG=1" GSM_NL);
    a7672sa_wait_response(handle, timeout_ms);
    a7672sa_send_command(handle, "SET_APN", "AT+CEREG=1" GSM_NL);
    a7672sa_wait_response(handle, timeout_ms);
    a7672sa_send_command(handle, "SET_APN", "AT+CGREG=1" GSM_NL);
    a7672sa_wait_response(handle, timeout_ms);

    // Set APN
    char cmd[100];
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IPV4V6\",\"%s\"" GSM_NL, apn);
    a7672sa_send_command(handle, "SET_APN", cmd);

    if (a7672sa_wait_response(handle, timeout_ms))
    {
        // Activate PDP context
        a7672sa_send_command(handle, "CGATT", "AT+CGATT=1" GSM_NL);
        if (a7672sa_wait_response(handle, timeout_ms))
        {
            a7672sa_send_command(handle, "CGACT", "AT+CGACT=1,1" GSM_NL);
            return a7672sa_wait_response(handle, timeout_ms);
        }
    }
    return false;
}

bool a7672sa_set_operator(a7672sa_handle_t handle, bool automatic,
                          const a7672sa_network_operator_t *op, uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;
    if (handle->publishing)
        return false;

    char cmd[64];
    if (automatic)
    {
        snprintf(cmd, sizeof(cmd), "AT+COPS=0" GSM_NL);
    }
    else if (op)
    {
        snprintf(cmd, sizeof(cmd), "AT+COPS=1,2,\"%s\"" GSM_NL, op->numeric_code);
    }
    else
    {
        return false;
    }

    a7672sa_send_command(handle, "SET_OPERATOR", cmd);
    return a7672sa_wait_response(handle, timeout_ms);
}

bool a7672sa_wait_network(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;
    if (handle->publishing)
        return false;

    // Enable registration URCs so the modem sends updates on status change
    a7672sa_send_command(handle, "CEREG=1", "AT+CEREG=1" GSM_NL);
    a7672sa_wait_response(handle, 1000);
    a7672sa_send_command(handle, "CGREG=1", "AT+CGREG=1" GSM_NL);
    a7672sa_wait_response(handle, 1000);

    // Initial poll
    a7672sa_send_command(handle, "CEREG?", "AT+CEREG?" GSM_NL);
    a7672sa_wait_response(handle, 1000);
    a7672sa_send_command(handle, "CGREG?", "AT+CGREG?" GSM_NL);
    a7672sa_wait_response(handle, 1000);

    int64_t start = millis();
    while (!a7672sa_ps_ready(handle) && (millis() - start) < timeout_ms)
    {
        // Poll every 5s as fallback in case URCs are missed
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (a7672sa_ps_ready(handle))
            break;
        a7672sa_send_command(handle, "CEREG?", "AT+CEREG?" GSM_NL);
        a7672sa_wait_response(handle, 1000);
        a7672sa_send_command(handle, "CGREG?", "AT+CGREG?" GSM_NL);
        a7672sa_wait_response(handle, 1000);
    }

    return a7672sa_ps_ready(handle);
}

/* ============================================================================
 * Time Functions
 * ============================================================================ */

bool a7672sa_set_ntp_server(a7672sa_handle_t handle, const char *ntp_server, int time_zone,
                            uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;
    if (handle->publishing)
        return false;

    char cmd[100];
    snprintf(cmd, sizeof(cmd), "AT+CNTP=\"%s\",%d" GSM_NL, ntp_server, time_zone);
    a7672sa_send_command(handle, "SET_NTP_SERVER", cmd);

    if (a7672sa_wait_response(handle, timeout_ms))
    {
        a7672sa_send_command(handle, "CNTP_SYNC", "AT+CNTP" GSM_NL);
        return a7672sa_wait_response(handle, timeout_ms);
    }
    return false;
}

time_t a7672sa_get_ntp_time(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL)
        return 0;
    if (handle->publishing)
        return 0;

    a7672sa_send_command(handle, "GET_NTP_TIME", "AT+CCLK?" GSM_NL);
    if (a7672sa_wait_response(handle, timeout_ms))
    {
        const char *response = a7672sa_get_response(handle);
        if (response)
        {
            const char *p = strstr(response, "+CCLK:");
            if (p)
            {
                p = strchr(p, '"');
                if (p)
                {
                    p++;
                    struct tm time_struct = {0};
                    int tz;
                    if (sscanf(p, "%d/%d/%d,%d:%d:%d%d", &time_struct.tm_year, &time_struct.tm_mon,
                               &time_struct.tm_mday, &time_struct.tm_hour, &time_struct.tm_min,
                               &time_struct.tm_sec, &tz) >= 6)
                    {
                        time_struct.tm_year += 100;  // Years since 1900
                        time_struct.tm_mon -= 1;     // 0-based months
                        return mktime(&time_struct);
                    }
                }
            }
        }
    }
    return 0;
}

/* ============================================================================
 * Device Information
 * ============================================================================ */

esp_err_t a7672sa_get_provider_name(a7672sa_handle_t handle, char *buffer, size_t buffer_size,
                                    uint32_t timeout_ms)
{
    if (handle == NULL || buffer == NULL)
        return ESP_ERR_INVALID_ARG;
    if (handle->publishing)
        return ESP_ERR_INVALID_STATE;

    // AT+CSPN? reads EF_SPN from the SIM. The modem may be busy flushing URCs
    // right after SIM-ready, so retry up to 3 times before giving up.
    for (int attempt = 1; attempt <= 3; attempt++)
    {
        a7672sa_send_command(handle, "GET_PROVIDER_NAME", "AT+CSPN?" GSM_NL);
        if (a7672sa_wait_response(handle, timeout_ms))
        {
            const char *response = a7672sa_get_response(handle);
            if (response)
            {
                const char *p = strstr(response, "+CSPN:");
                if (p)
                {
                    p = strchr(p, '"');
                    if (p)
                    {
                        p++;
                        const char *end = strchr(p, '"');
                        if (end && end > p)
                        {
                            size_t len = end - p;
                            if (len >= buffer_size)
                                len = buffer_size - 1;
                            strncpy(buffer, p, len);
                            buffer[len] = '\0';
                            return ESP_OK;
                        }
                    }
                }
            }
        }
    }

    strncpy(buffer, "UNKNOWN", buffer_size - 1);
    buffer[buffer_size - 1] = '\0';
    return ESP_FAIL;
}

/*
 * provider_from_mnc / a7672sa_get_imsi — detecção de operadora via IMSI
 * -----------------------------------------------------------------------
 * Uso conjunto pretendido (alternativa ao AT+CSPN? para chips que não têm
 * EF_SPN preenchido):
 *
 *   char imsi[20];
 *   if (a7672sa_get_imsi(handle, imsi, sizeof(imsi), 10000) == ESP_OK) {
 *       // IMSI brasileiro: MCC=724 (dígitos 0-2), MNC=2 dígitos (dígitos 3-4)
 *       char mnc[3] = {imsi[3], imsi[4], '\0'};
 *       const char *operadora = provider_from_mnc(mnc);
 *       // operadora == "TIM" | "VIVO" | "CLARO" | "OI" | "ALGAR" | "ARQIA" | NULL
 *   }
 *
 * Tabela MNC Brasil (MCC 724):
 *   02, 03, 04 → TIM
 *   05         → CLARO
 *   06, 10, 11 → VIVO
 *   16, 31     → OI
 *   33         → ALGAR
 *   39         → ARQIA
 *
 * Atenção: AT+CIMI pode demorar >5 s logo após SIM-ready porque o modem
 * ainda está processando URCs. Usar timeout_ms >= 10000.
 */
static const char *provider_from_mnc(const char *mnc2)
{
    if (strcmp(mnc2, "02") == 0 || strcmp(mnc2, "03") == 0 || strcmp(mnc2, "04") == 0)
        return "TIM";
    if (strcmp(mnc2, "05") == 0)
        return "CLARO";
    if (strcmp(mnc2, "06") == 0 || strcmp(mnc2, "10") == 0 || strcmp(mnc2, "11") == 0)
        return "VIVO";
    if (strcmp(mnc2, "16") == 0 || strcmp(mnc2, "31") == 0)
        return "OI";
    if (strcmp(mnc2, "33") == 0)
        return "ALGAR";
    if (strcmp(mnc2, "39") == 0)
        return "ARQIA";
    return NULL;
}

esp_err_t a7672sa_get_imsi(a7672sa_handle_t handle, char *buffer, size_t buffer_size,
                           uint32_t timeout_ms)
{
    if (handle == NULL || buffer == NULL)
        return ESP_ERR_INVALID_ARG;
    if (handle->publishing)
        return ESP_ERR_INVALID_STATE;

    a7672sa_send_command(handle, "GET_IMSI", "AT+CIMI" GSM_NL);
    if (!a7672sa_wait_response(handle, timeout_ms))
        return ESP_FAIL;

    const char *response = a7672sa_get_response(handle);
    if (!response)
        return ESP_FAIL;

    // O buffer pode conter URCs acumulados. Varre linha a linha procurando
    // uma linha puramente numérica de 14-15 dígitos (formato do IMSI).
    const char *line = response;
    while (*line)
    {
        while (*line == '\r' || *line == '\n' || *line == ' ')
            line++;
        if (!*line)
            break;

        const char *end = line;
        while (*end && *end != '\r' && *end != '\n')
            end++;

        size_t len = end - line;
        if (len >= 14 && len <= 16)
        {
            bool all_digits = true;
            for (size_t i = 0; i < len; i++)
            {
                if (line[i] < '0' || line[i] > '9')
                {
                    all_digits = false;
                    break;
                }
            }
            if (all_digits)
            {
                if (len >= buffer_size)
                    len = buffer_size - 1;
                strncpy(buffer, line, len);
                buffer[len] = '\0';
                return ESP_OK;
            }
        }
        line = end;
    }
    return ESP_FAIL;
}

esp_err_t a7672sa_get_imei(a7672sa_handle_t handle, char *buffer, size_t buffer_size,
                           uint32_t timeout_ms)
{
    if (handle == NULL || buffer == NULL)
        return ESP_ERR_INVALID_ARG;
    if (handle->publishing)
        return ESP_ERR_INVALID_STATE;

    a7672sa_send_command(handle, "GET_IMEI", "AT+CGSN" GSM_NL);
    if (a7672sa_wait_response(handle, timeout_ms))
    {
        const char *response = a7672sa_get_response(handle);
        if (response)
        {
            // Find numeric string (IMEI is 15 digits)
            const char *p = response;
            while (*p)
            {
                if (*p >= '0' && *p <= '9')
                {
                    int len = 0;
                    while (p[len] >= '0' && p[len] <= '9' && len < 16)
                        len++;
                    if (len == 15)
                    {
                        if ((size_t)len >= buffer_size)
                            len = buffer_size - 1;
                        strncpy(buffer, p, len);
                        buffer[len] = '\0';
                        return ESP_OK;
                    }
                }
                p++;
            }
        }
    }
    strncpy(buffer, "0", buffer_size - 1);
    return ESP_FAIL;
}

esp_err_t a7672sa_get_iccid(a7672sa_handle_t handle, char *buffer, size_t buffer_size,
                            uint32_t timeout_ms)
{
    if (handle == NULL || buffer == NULL)
        return ESP_ERR_INVALID_ARG;
    if (handle->publishing)
        return ESP_ERR_INVALID_STATE;

    a7672sa_send_command(handle, "GET_ICCID", "AT+CICCID" GSM_NL);
    if (a7672sa_wait_response(handle, timeout_ms))
    {
        const char *response = a7672sa_get_response(handle);
        if (response)
        {
            const char *p = strstr(response, "+ICCID:");
            if (p)
            {
                p += 7;
                while (*p == ' ')
                    p++;
                int len = 0;
                while (p[len] >= '0' && p[len] <= '9' && len < 22)
                    len++;
                if (len > 0)
                {
                    if ((size_t)len >= buffer_size)
                        len = buffer_size - 1;
                    strncpy(buffer, p, len);
                    buffer[len] = '\0';
                    return ESP_OK;
                }
            }
        }
    }
    strncpy(buffer, "0", buffer_size - 1);
    return ESP_FAIL;
}

esp_err_t a7672sa_get_local_ip(a7672sa_handle_t handle, a7672sa_ip_addr_t *ip, uint32_t timeout_ms)
{
    if (handle == NULL || ip == NULL)
        return ESP_ERR_INVALID_ARG;
    if (handle->publishing)
        return ESP_ERR_INVALID_STATE;

    memset(ip, 0, sizeof(a7672sa_ip_addr_t));

    a7672sa_send_command(handle, "GET_LOCAL_IP", "AT+CGPADDR" GSM_NL);
    if (a7672sa_wait_response(handle, timeout_ms))
    {
        const char *response = a7672sa_get_response(handle);
        if (response)
        {
            const char *p = strstr(response, "+CGPADDR:");
            if (p)
            {
                // Skip CID and find first IP
                p = strchr(p, ',');
                if (p)
                {
                    p++;
                    int o1, o2, o3, o4;
                    if (sscanf(p, "%d.%d.%d.%d", &o1, &o2, &o3, &o4) == 4)
                    {
                        ip->octets[0] = o1;
                        ip->octets[1] = o2;
                        ip->octets[2] = o3;
                        ip->octets[3] = o4;
                        return ESP_OK;
                    }
                }
            }
        }
    }
    return ESP_FAIL;
}

esp_err_t a7672sa_get_local_ipv6(a7672sa_handle_t handle, char *buffer, size_t buffer_size,
                                 uint32_t timeout_ms)
{
    if (handle == NULL || buffer == NULL)
        return ESP_ERR_INVALID_ARG;
    if (handle->publishing)
        return ESP_ERR_INVALID_STATE;

    a7672sa_send_command(handle, "GET_LOCAL_IPV6", "AT+CGPADDR" GSM_NL);
    if (a7672sa_wait_response(handle, timeout_ms))
    {
        const char *response = a7672sa_get_response(handle);
        if (response)
        {
            const char *p = strstr(response, "+CGPADDR:");
            if (p)
            {
                // Find second address (IPv6)
                p = strchr(p, ',');
                if (p)
                    p = strchr(p + 1, ',');
                if (p)
                {
                    p++;
                    const char *end = strstr(p, GSM_NL);
                    if (end)
                    {
                        size_t len = end - p;
                        if (len >= buffer_size)
                            len = buffer_size - 1;
                        strncpy(buffer, p, len);
                        buffer[len] = '\0';
                        return ESP_OK;
                    }
                }
            }
        }
    }
    buffer[0] = '\0';
    return ESP_FAIL;
}

/* ============================================================================
 * SSL/TLS Functions
 * ============================================================================ */

bool a7672sa_set_ca_cert(a7672sa_handle_t handle, const char *ca_cert, const char *ca_name,
                         size_t cert_size, uint32_t timeout_ms)
{
    if (handle == NULL || ca_cert == NULL || ca_name == NULL)
        return false;

    char cmd[100];
    snprintf(cmd, sizeof(cmd), "AT+CCERTDOWN=\"%s\",%d" GSM_NL, ca_name, (int)cert_size);

    handle->at_input = false;
    a7672sa_send_command(handle, "SET_CA_CERT", cmd);

    if (a7672sa_wait_input(handle, timeout_ms))
    {
        // Queue message is limited to A7672SA_CMD_DATA_SIZE — send in chunks
        const uint8_t *ptr = (const uint8_t *)ca_cert;
        size_t remaining = cert_size;
        while (remaining > 0)
        {
            size_t chunk = remaining < A7672SA_CMD_DATA_SIZE ? remaining : A7672SA_CMD_DATA_SIZE;
            a7672sa_send_data(handle, "CA_CERT_DATA", ptr, chunk);
            ptr += chunk;
            remaining -= chunk;
        }
        bool ok = a7672sa_wait_response(handle, timeout_ms);
        if (!ok)
        {
            const uint8_t ctrl_z = 0x1a;
            a7672sa_send_data(handle, "DATA_ABORT", &ctrl_z, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        return ok;
    }

    // Modem may have entered data-input mode (sent >) but we missed it.
    // Send Ctrl-Z to abort the upload and return the modem to command mode.
    const uint8_t ctrl_z = 0x1a;
    a7672sa_send_data(handle, "DATA_ABORT", &ctrl_z, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    return false;
}

/* ============================================================================
 * MQTT Functions
 * ============================================================================ */

bool a7672sa_mqtt_connect(a7672sa_handle_t handle, const char *host, uint16_t port,
                          const char *client_id, bool clean_session, const char *username,
                          const char *password, bool ssl, const char *ca_name, uint16_t keepalive,
                          uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;

    char cmd[256];

    // Ensure MQTT service is running — it may have been stopped by a previous disconnect.
    // Error 19 ("already started") is treated as OK by the response parser.
    a7672sa_send_command(handle, "CMQTTSTART", "AT+CMQTTSTART" GSM_NL);
    a7672sa_wait_response(handle, 5000);

    // Release any leftover client from a previous (possibly failed) session
    a7672sa_send_command(handle, "MQTT_RELEASE", "AT+CMQTTREL=0" GSM_NL);
    a7672sa_wait_response(handle, 2000);

    // Step 1: Acquire MQTT client
    if (ssl)
    {
        // SSL Configuration
        a7672sa_send_command(handle, "CSSLCFG_SNI", "AT+CSSLCFG=\"enableSNI\",0,1" GSM_NL);
        a7672sa_wait_response(handle, 1000);

        snprintf(cmd, sizeof(cmd), "AT+CSSLCFG=\"cacert\",0,\"%s\"" GSM_NL, ca_name);
        a7672sa_send_command(handle, "CSSLCFG_CA", cmd);
        a7672sa_wait_response(handle, 1000);

        a7672sa_send_command(handle, "CSSLCFG_SSL", "AT+CSSLCFG=\"sslversion\",0,3" GSM_NL);
        a7672sa_wait_response(handle, 1000);

        // authmode=0: no server cert verification (temporary — remove once cert upload works)
        a7672sa_send_command(handle, "CSSLCFG_AUTH", "AT+CSSLCFG=\"authmode\",0,0" GSM_NL);
        a7672sa_wait_response(handle, 1000);

        // Acquire client with SSL (servertype=1)
        snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\",1" GSM_NL,
                 client_id ? client_id : "esp32_client");
    }
    else
    {
        // Acquire client without SSL (servertype=0)
        snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\",0" GSM_NL,
                 client_id ? client_id : "esp32_client");
    }

    a7672sa_send_command(handle, "CMQTTACCQ", cmd);
    if (!a7672sa_wait_response(handle, 2000))
    {
        if (!ssl)
        {
            // Firmware compatibility fallback: some revisions do not accept <servertype> for
            // non-SSL ACCQ
            snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\"" GSM_NL,
                     client_id ? client_id : "esp32_client");
            a7672sa_send_command(handle, "CMQTTACCQ_FALLBACK", cmd);
            if (!a7672sa_wait_response(handle, 2000))
            {
                ESP_LOGE(TAG, "MQTT: Failed to acquire client");
                return false;
            }
        }
        else
        {
            ESP_LOGE(TAG, "MQTT: Failed to acquire client");
            return false;
        }
    }

    // Step 2: Disable UTF-8 validation on the payload. Default is checkUTF8=1, and the modem
    // rejects binary Protobuf payloads at AT+CMQTTPAYLOAD with err=2 ("bad UTF-8 string") since
    // they are not valid UTF-8. Must be sent after CMQTTACCQ and before CMQTTCONNECT — the
    // setting is cleared on CMQTTREL.
    a7672sa_send_command(handle, "CMQTTCFG_UTF8", "AT+CMQTTCFG=\"checkUTF8\",0,0" GSM_NL);
    if (!a7672sa_wait_response(handle, 2000))
    {
        ESP_LOGW(TAG, "MQTT: failed to disable checkUTF8 — binary payloads may be rejected");
    }

    // Step 3: Connect to broker
    // Format:
    // AT+CMQTTCONNECT=<client_index>,<server_addr>,<keepalive_time>,<clean_session>[,<username>,<password>]
    // Always tcp:// — SSL is handled at the CMQTTACCQ level (servertype=1), not via URL scheme.
    const char *scheme = "tcp";
    if (username && strlen(username) > 0)
    {
        snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=0,\"%s://%s:%d\",%d,%d,\"%s\",\"%s\"" GSM_NL,
                 scheme, host, port, keepalive, clean_session ? 1 : 0, username,
                 password ? password : "");
    }
    else
    {
        snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=0,\"%s://%s:%d\",%d,%d" GSM_NL, scheme, host,
                 port, keepalive, clean_session ? 1 : 0);
    }

    handle->mqtt_connected = false;
    handle->at_error = false;
    a7672sa_send_command(handle, "CMQTTCONNECT", cmd);

    // Wait for connection URC (+CMQTTCONNECT: 0,0) or error
    int64_t start = millis();
    while (!handle->mqtt_connected && !handle->at_error && (millis() - start) < timeout_ms)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Grace window: on a slow/congested link (NB-IoT) the CONNECT URC can land just past
    // the nominal timeout. Declaring failure and tearing down the session while the modem
    // is still finishing the handshake races the cleanup commands against that in-flight
    // URC — the DISC/REL/STOP below can end up failing too, leaving the modem genuinely
    // connected while this driver (and the caller) believe the attempt failed, with no way
    // to recover short of a physical power cycle. A short extra wait avoids that in the
    // common case.
    if (!handle->mqtt_connected && !handle->at_error)
    {
        int64_t grace_start = millis();
        while (!handle->mqtt_connected && (millis() - grace_start) < 2000)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    if (!handle->mqtt_connected)
    {
        ESP_LOGE(TAG, "MQTT: Connection failed — cleaning up MQTT state");
        // Immediately clean up so the modem is not stuck in a failed MQTT state
        // on the next retry (or after an ESP32 reset).
        a7672sa_send_command(handle, "MQTT_DISC_CLEAN", "AT+CMQTTDISC=0,10" GSM_NL);
        a7672sa_wait_response(handle, 3000);
        a7672sa_send_command(handle, "MQTT_REL_CLEAN", "AT+CMQTTREL=0" GSM_NL);
        a7672sa_wait_response(handle, 2000);
        a7672sa_send_command(handle, "MQTT_STOP_CLEAN", "AT+CMQTTSTOP" GSM_NL);
        a7672sa_wait_response(handle, 5000);

        // The CONNECT URC can still land after cleanup started (it was already in flight
        // on the modem side when we gave up), racing the DISC/REL/STOP above so they don't
        // actually land. Don't leave a connected-but-unknown session behind — tear it down
        // properly now that the modem has settled.
        if (handle->mqtt_connected)
        {
            ESP_LOGW(TAG, "MQTT: connected after cleanup had already started — disconnecting");
            a7672sa_mqtt_disconnect(handle, 3000);
        }
    }

    return handle->mqtt_connected;
}

bool a7672sa_mqtt_disconnect(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;

    // Step 1: Disconnect from broker
    bool was_connected = handle->mqtt_connected;
    a7672sa_send_command(handle, "MQTT_DISCONNECT", "AT+CMQTTDISC=0,120" GSM_NL);
    a7672sa_wait_response(handle, timeout_ms);

    // If the CMQTTDISC URC was parsed by rx_task it already called on_mqtt_status and
    // cleared mqtt_connected.  Only fire the callback here as a fallback (error / timeout).
    if (was_connected && handle->mqtt_connected)
    {
        handle->mqtt_connected = false;
        if (handle->on_mqtt_status)
        {
            handle->on_mqtt_status(A7672SA_MQTT_DISCONNECTED);
        }
    }

    // Step 2: Release the client (required before stopping)
    a7672sa_send_command(handle, "MQTT_RELEASE", "AT+CMQTTREL=0" GSM_NL);
    a7672sa_wait_response(handle, timeout_ms);

    // Step 3: Stop MQTT service
    a7672sa_send_command(handle, "MQTT_STOP", "AT+CMQTTSTOP" GSM_NL);
    return a7672sa_wait_response(handle, timeout_ms);
}

bool a7672sa_mqtt_release_client(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;
    a7672sa_send_command(handle, "MQTT_RELEASE", "AT+CMQTTREL=0" GSM_NL);
    return a7672sa_wait_response(handle, timeout_ms);
}

bool a7672sa_mqtt_publish(a7672sa_handle_t handle, const char *topic, const uint8_t *data,
                          size_t len, uint16_t qos, uint32_t timeout_ms)
{
    if (handle == NULL || topic == NULL)
        return false;
    if (len > 0 && data == NULL)
        return false;

    if (handle->publishing)
    {
        return false;
    }

    if (!a7672sa_publish_lock(handle, timeout_ms))
    {
        return false;
    }

    char cmd[256];
    size_t topic_len = strlen(topic);
    bool ok = false;

#if defined(CONFIG_A7672SA_DEBUG_LOG)
    ESP_LOGI(TAG, "MQTT Publish start: topic=%s len=%d mqtt_connected=%d", topic, (int)len,
             handle->mqtt_connected);
#endif

    handle->at_publish = false;
    handle->at_input = false;

    // Step 1: Set topic
    // AT+CMQTTTOPIC=<client_index>,<req_length>
    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d" GSM_NL, (int)topic_len);
    a7672sa_send_command(handle, "MQTT_PUB_TOPIC", cmd);

    if (!a7672sa_wait_input(handle, timeout_ms))
    {
        ESP_LOGE(TAG, "MQTT Publish: No input prompt for topic (%s)", topic);
        const uint8_t ctrl_z = 0x1a;
        a7672sa_send_data(handle, "MQTT_PUB_ABORT", &ctrl_z, 1);
        a7672sa_wait_response(handle, 1000);
        a7672sa_publish_unlock(handle);
        return false;
    }

    // Send topic data
    a7672sa_send_data(handle, "MQTT_TOPIC_DATA", (const uint8_t *)topic, topic_len);

    if (!a7672sa_wait_response(handle, timeout_ms))
    {
        ESP_LOGE(TAG, "MQTT Publish: Topic (%s) data failed", topic);
        const uint8_t ctrl_z = 0x1a;
        a7672sa_send_data(handle, "MQTT_PUB_ABORT", &ctrl_z, 1);
        a7672sa_wait_response(handle, 1000);
        a7672sa_publish_unlock(handle);
        return false;
    }

    // Step 2: Set payload (skipped for empty payloads — modem publishes with no payload)
    if (len > 0)
    {
        snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%d" GSM_NL, (int)len);
        a7672sa_send_command(handle, "MQTT_PUB_PAYLOAD", cmd);

        if (!a7672sa_wait_input(handle, timeout_ms))
        {
            ESP_LOGE(TAG, "MQTT Publish: No input prompt for payload (%s)", topic);
            const uint8_t ctrl_z = 0x1a;
            a7672sa_send_data(handle, "MQTT_PUB_ABORT", &ctrl_z, 1);
            a7672sa_wait_response(handle, 1000);
            a7672sa_publish_unlock(handle);
            return false;
        }

#if defined(CONFIG_A7672SA_DEBUG_LOG)
        ESP_LOGI(TAG, "MQTT Publish: Payload prompt OK, sending %d bytes (mqtt_connected=%d)",
                 (int)len, handle->mqtt_connected);
        {
            char hex_buf[192];
            size_t show = len < 48 ? len : 48;
            size_t pos = 0;
            for (size_t pi = 0; pi < show && pos + 3 < sizeof(hex_buf); pi++)
                pos += snprintf(hex_buf + pos, sizeof(hex_buf) - pos, "%02X ", data[pi]);
            ESP_LOGI(TAG, "MQTT Publish: Payload hex: %s%s", hex_buf, len > 48 ? "..." : "");
        }
#endif

        // Send payload in chunks — a7672sa_send_data copies at most A7672SA_CMD_DATA_SIZE
        // bytes per queue item, so payloads larger than 1024 bytes must be split.
        const uint8_t *ptr = data;
        size_t remaining = len;
        while (remaining > 0)
        {
            size_t chunk = remaining < A7672SA_CMD_DATA_SIZE ? remaining : A7672SA_CMD_DATA_SIZE;
            a7672sa_send_data(handle, "MQTT_PAYLOAD_DATA", ptr, chunk);
            ptr += chunk;
            remaining -= chunk;
        }

        if (!a7672sa_wait_response(handle, timeout_ms))
        {
            ESP_LOGE(TAG, "MQTT Publish: Payload data failed (%s) mqtt_connected=%d", topic,
                     handle->mqtt_connected);
            const uint8_t ctrl_z = 0x1a;
            a7672sa_send_data(handle, "MQTT_PUB_ABORT", &ctrl_z, 1);
            a7672sa_wait_response(handle, 1000);
            a7672sa_publish_unlock(handle);
            return false;
        }
    }

    // Step 3: Execute publish
    // AT+CMQTTPUB=<client_index>,<qos>,<pub_timeout>[,<retained>[,<dup>]]
    snprintf(cmd, sizeof(cmd), "AT+CMQTTPUB=0,%d,60" GSM_NL, qos);
    a7672sa_send_command(handle, "MQTT_PUB_EXEC", cmd);

    // Wait for +CMQTTPUB: 0,0
    ok = wait_for_condition(handle, timeout_ms, check_at_publish, "MQTT PUBLISH");

    a7672sa_publish_unlock(handle);
    return ok;
}

bool a7672sa_mqtt_subscribe(a7672sa_handle_t handle, const char *topic, uint16_t qos,
                            uint32_t timeout_ms)
{
    if (handle == NULL || topic == NULL)
        return false;
    if (handle->publishing)
        return false;

    char cmd[256];
    size_t topic_len = strlen(topic);

    // Step 1: Set topic length
    // AT+CMQTTSUBTOPIC=<client_index>,<req_length>,<qos>
    snprintf(cmd, sizeof(cmd), "AT+CMQTTSUBTOPIC=0,%d,%d" GSM_NL, (int)topic_len, qos);
    a7672sa_send_command(handle, "MQTT_SUBTOPIC", cmd);

    // Wait for ">" prompt
    if (!a7672sa_wait_input(handle, timeout_ms))
    {
        ESP_LOGE(TAG, "MQTT Subscribe: No input prompt for topic");
        return false;
    }

    // Step 2: Send topic data
    a7672sa_send_data(handle, "MQTT_TOPIC_DATA", (const uint8_t *)topic, topic_len);

    if (!a7672sa_wait_response(handle, timeout_ms))
    {
        ESP_LOGE(TAG, "MQTT Subscribe: Topic data failed");
        return false;
    }

    // Step 3: Execute subscribe
    // AT+CMQTTSUB=<client_index>
    handle->at_ok = false;
    handle->at_subscribed = false;
    handle->at_error = false;
    a7672sa_send_command(handle, "MQTT_SUBSCRIBE", "AT+CMQTTSUB=0" GSM_NL);

    // O manual AT diz que AT+CMQTTSUB=0 retorna "OK" imediatamente (comando aceito)
    // e o resultado real da inscrição chega depois via URC assíncrona
    // (+CMQTTSUB: 0,<result>). Na prática, em bancada, o "OK" às vezes nunca chega
    // mesmo com a inscrição já confirmada no broker — então aceita qualquer um dos
    // dois sinais, o que chegar primeiro, em vez de exigir o OK antes da URC.
    bool ok = wait_for_condition(handle, timeout_ms, check_at_ok_or_subscribed, "MQTT SUBSCRIBE");
    if (!ok)
        ESP_LOGE(TAG, "MQTT Subscribe: timed out waiting for CMQTTSUB ack/URC (%s)", topic);

    bool had_error = handle->at_error;
    handle->at_ok = false;
    handle->at_subscribed = false;
    handle->at_error = false;
    return ok && !had_error;
}

bool a7672sa_mqtt_subscribe_topics(a7672sa_handle_t handle, const char **topics, int n_topics,
                                   uint16_t qos, uint32_t timeout_ms)
{
    if (handle == NULL || topics == NULL)
        return false;
    if (handle->publishing)
        return false;

    char cmd[256];
    for (int i = 0; i < n_topics; i++)
    {
        if (topics[i])
        {
            snprintf(cmd, sizeof(cmd), "AT+CMQTTSUBTOPIC=0,%d,%d" GSM_NL, (int)strlen(topics[i]),
                     qos);
            a7672sa_send_command(handle, "MQTT_SUBTOPIC", cmd);
            if (a7672sa_wait_input(handle, timeout_ms))
            {
                a7672sa_send_command(handle, "TOPIC", topics[i]);
                a7672sa_wait_response(handle, timeout_ms);
            }
        }
    }

    handle->at_ok = false;
    handle->at_subscribed = false;
    handle->at_error = false;
    a7672sa_send_command(handle, "MQTT_SUBSCRIBE", "AT+CMQTTSUB=0" GSM_NL);

    // See a7672sa_mqtt_subscribe() for why OK and the URC are both accepted.
    bool ok = wait_for_condition(handle, timeout_ms, check_at_ok_or_subscribed, "MQTT SUBSCRIBE");
    if (!ok)
        ESP_LOGE(TAG, "MQTT Subscribe: timed out waiting for CMQTTSUB ack/URC");

    bool had_error = handle->at_error;
    handle->at_ok = false;
    handle->at_subscribed = false;
    handle->at_error = false;
    return ok && !had_error;
}

bool a7672sa_mqtt_is_connected(a7672sa_handle_t handle)
{
    return handle ? handle->mqtt_connected : false;
}

/* ============================================================================
 * HTTP Functions
 * ============================================================================ */

uint32_t a7672sa_http_request(a7672sa_handle_t handle, const a7672sa_http_request_config_t *config,
                              uint32_t timeout_ms)
{
    if (handle == NULL || config == NULL || config->url == NULL)
        return 0;

    a7672sa_send_command(handle, "HTTP_INIT", "AT+HTTPINIT" GSM_NL);
    if (!a7672sa_wait_response(handle, timeout_ms))
    {
        return 0;
    }

    char cmd[512];

    // Set URL
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"" GSM_NL, config->url);
    a7672sa_send_command(handle, "HTTP_URL", cmd);
    if (!a7672sa_wait_response(handle, timeout_ms))
    {
        a7672sa_http_term(handle, 1000);
        return 0;
    }

    // Set content type
    if (config->content_type && strlen(config->content_type) > 0)
    {
        snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"CONTENT\",\"%s\"" GSM_NL, config->content_type);
        a7672sa_send_command(handle, "HTTP_CONTENT", cmd);
        a7672sa_wait_response(handle, timeout_ms);
    }

    // Set accept type
    if (config->accept_type && strlen(config->accept_type) > 0)
    {
        snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"ACCEPT\",\"%s\"" GSM_NL, config->accept_type);
        a7672sa_send_command(handle, "HTTP_ACCEPT", cmd);
        a7672sa_wait_response(handle, timeout_ms);
    }

    // SSL configuration
    if (config->ssl)
    {
        a7672sa_send_command(handle, "HTTP_SSLVER", "AT+CSSLCFG=\"sslversion\",0,4" GSM_NL);
        a7672sa_wait_response(handle, timeout_ms);

        snprintf(cmd, sizeof(cmd), "AT+CSSLCFG=\"authmode\",0,%d" GSM_NL,
                 config->verify_server ? 1 : 0);
        a7672sa_send_command(handle, "HTTP_AUTHMODE", cmd);
        a7672sa_wait_response(handle, timeout_ms);

        // SNI: servidores atrás de proxy/ALB podem exigir para servir o cert correto
        a7672sa_send_command(handle, "HTTP_SNI", "AT+CSSLCFG=\"enableSNI\",0,1" GSM_NL);
        a7672sa_wait_response(handle, timeout_ms);

        if (config->verify_server)
        {
            snprintf(cmd, sizeof(cmd), "AT+CSSLCFG=\"cacert\",0,\"%s\"" GSM_NL, config->ca_name);
            a7672sa_send_command(handle, "HTTP_CACERT", cmd);
            a7672sa_wait_response(handle, timeout_ms);
        }

        a7672sa_send_command(handle, "HTTP_SSLCFG", "AT+HTTPPARA=\"SSLCFG\",0" GSM_NL);
        a7672sa_wait_response(handle, timeout_ms);
    }

    // Authorization / custom header
    if (config->user_data && config->user_data_size > 0)
    {
        snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"USERDATA\",\"%s\"" GSM_NL, config->user_data);
        a7672sa_send_command(handle, "HTTP_USERDATA", cmd);
        a7672sa_wait_response(handle, timeout_ms);
    }

    // Set timeouts
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"CONNECTTO\",%lu" GSM_NL, config->conn_timeout);
    a7672sa_send_command(handle, "HTTP_CONNTO", cmd);
    a7672sa_wait_response(handle, timeout_ms);

    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"RECVTO\",%lu" GSM_NL, config->recv_timeout);
    a7672sa_send_command(handle, "HTTP_RECVTO", cmd);
    a7672sa_wait_response(handle, timeout_ms);

    // POST data
    if (config->method == A7672SA_HTTP_POST && config->post_data_size > 0)
    {
        snprintf(cmd, sizeof(cmd), "AT+HTTPDATA=%d,%lu" GSM_NL, (int)config->post_data_size,
                 config->conn_timeout * 1000);
        a7672sa_send_command(handle, "HTTP_DATA", cmd);
        if (a7672sa_wait_input(handle, timeout_ms))
        {
            a7672sa_send_data(handle, "HTTP_POST_DATA", (const uint8_t *)config->post_data,
                              config->post_data_size);
            a7672sa_wait_response(handle, timeout_ms);
        }
    }

    // Perform action
    handle->http_response_received = false;
    handle->http_response_data.status_code = 0;
    handle->http_response_data.content_size = 0;

    snprintf(cmd, sizeof(cmd), "AT+HTTPACTION=%d" GSM_NL, config->method);
    a7672sa_send_command(handle, "HTTP_ACTION", cmd);

    // Wait for HTTP response
    int64_t start = millis();
    while (!handle->http_response_received && (millis() - start) < timeout_ms)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return handle->http_response_data.status_code;
}

uint32_t a7672sa_http_request_file(a7672sa_handle_t handle,
                                   const a7672sa_http_request_config_t *config,
                                   const char *filename, uint32_t timeout_ms)
{
    if (handle == NULL || config == NULL || filename == NULL)
        return 0;

    uint32_t status = a7672sa_http_request(handle, config, timeout_ms);
    if (status == 0)
        return 0;

    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+HTTPREADFILE=\"%s\"" GSM_NL, filename);
    a7672sa_send_command(handle, "HTTP_READFILE", cmd);
    a7672sa_wait_response(handle, timeout_ms);

    if (handle->at_error)
        return 0;
    return status;
}

size_t a7672sa_http_read_response(a7672sa_handle_t handle, uint8_t *buffer, size_t read_size,
                                  size_t offset, uint32_t timeout_ms)
{
    if (handle == NULL || buffer == NULL)
        return 0;

    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+HTTPREAD=%d,%d" GSM_NL, (int)offset, (int)read_size);
    a7672sa_send_command(handle, "HTTP_READ", cmd);

    if (a7672sa_wait_response(handle, timeout_ms))
    {
        const char *response = a7672sa_get_response(handle);
        if (response)
        {
            const char *p = strstr(response, "+HTTPREAD:");
            if (p)
            {
                int len;
                if (sscanf(p, "+HTTPREAD: %d", &len) == 1)
                {
                    p = strchr(p, '\n');
                    if (p)
                    {
                        p++;
                        size_t copy_len = len < (int)read_size ? len : read_size;
                        memcpy(buffer, p, copy_len);
                        return copy_len;
                    }
                }
            }
        }
    }
    return 0;
}

uint32_t a7672sa_http_response_size(a7672sa_handle_t handle)
{
    return handle ? handle->http_response_data.content_size : 0;
}

uint32_t a7672sa_http_response_header_size(a7672sa_handle_t handle)
{
    return handle ? handle->http_response_data.header_size : 0;
}

esp_err_t a7672sa_http_response_etag(a7672sa_handle_t handle, char *buffer, size_t buffer_size)
{
    if (handle == NULL || buffer == NULL)
        return ESP_ERR_INVALID_ARG;
    strncpy(buffer, handle->http_response_data.etag, buffer_size - 1);
    buffer[buffer_size - 1] = '\0';
    return ESP_OK;
}

bool a7672sa_http_term(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;
    a7672sa_send_command(handle, "HTTP_TERM", "AT+HTTPTERM" GSM_NL);
    return a7672sa_wait_response(handle, timeout_ms);
}

/* ============================================================================
 * File System Functions
 * ============================================================================ */

bool a7672sa_fs_open(a7672sa_handle_t handle, const char *filename, uint16_t mode,
                     uint32_t timeout_ms)
{
    if (handle == NULL || filename == NULL)
        return false;
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+FSOPEN=\"C:/%s\",%d" GSM_NL, filename, mode);
    a7672sa_send_command(handle, "FS_OPEN", cmd);
    ESP_LOGI(TAG, "FS_OPEN: waiting for modem response (timeout=%lu ms)...",
             (unsigned long)timeout_ms);
    bool ok = a7672sa_wait_response(handle, timeout_ms);
    const char *resp = a7672sa_get_response(handle);
    ESP_LOGI(TAG, "FS_OPEN: wait_response=%d  resp[0..39]='%.40s'", ok, resp ? resp : "(null)");
    if (!ok)
        return false;
    if (resp)
    {
        const char *p = strstr(resp, "+FSOPEN:");
        if (p)
        {
            sscanf(p, "+FSOPEN: %d", &handle->fs_handle);
            ESP_LOGI(TAG, "FS_OPEN: file handle = %d", handle->fs_handle);
            return true;
        }
    }
    ESP_LOGW(TAG, "FS_OPEN: +FSOPEN not found in response");
    return false;
}

bool a7672sa_fs_close(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL)
        return false;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+FSCLOSE=%d" GSM_NL, handle->fs_handle);
    a7672sa_send_command(handle, "FS_CLOSE", cmd);
    return a7672sa_wait_response(handle, timeout_ms);
}

uint32_t a7672sa_fs_size(a7672sa_handle_t handle, const char *filename, uint32_t timeout_ms)
{
    if (handle == NULL || filename == NULL)
        return 0;

    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+FSATTRI=\"C:/%s\"" GSM_NL, filename);
    a7672sa_send_command(handle, "FS_SIZE", cmd);

    if (a7672sa_wait_response(handle, timeout_ms))
    {
        const char *response = a7672sa_get_response(handle);
        if (response)
        {
            const char *p = strstr(response, "+FSATTRI:");
            if (p)
            {
                int size;
                if (sscanf(p, "+FSATTRI: %d", &size) == 1)
                {
                    return size;
                }
            }
        }
    }
    return 0;
}

bool a7672sa_fs_delete(a7672sa_handle_t handle, const char *filename, uint32_t timeout_ms)
{
    if (handle == NULL || filename == NULL)
        return false;
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+FSDEL=\"C:/%s\"" GSM_NL, filename);
    a7672sa_send_command(handle, "FS_DELETE", cmd);
    return a7672sa_wait_response(handle, timeout_ms);
}

size_t a7672sa_fs_read(a7672sa_handle_t handle, uint8_t *buffer, size_t read_size,
                       uint32_t timeout_ms)
{
    if (handle == NULL || buffer == NULL || read_size == 0)
        return 0;

    // Flush any stale bytes from a previous command/response before sending a new one.
    uart_flush_input(handle->config.uart_num);

    // A7672SA format: AT+FSREAD=<fh>,<readlen>  (2 parameters — no mode)
    // Response: +FSREAD: <actual_len>\r\n<binary data>\r\nOK\r\n
    // The 3-parameter variant (mode=1 CONNECT) returns ERROR on A7672SA.
    char cmd[64];
    int cmd_len =
        snprintf(cmd, sizeof(cmd), "AT+FSREAD=%d,%d" GSM_NL, handle->fs_handle, (int)read_size);
    uart_write_bytes(handle->config.uart_num, cmd, cmd_len);

    // Read response header byte-by-byte; accept both "+FSREAD: <len>" and "CONNECT <len>"
    char hdr[128];
    int hdr_pos = 0;
    int64_t start = esp_timer_get_time() / 1000;
    bool found = false;
    size_t data_len = 0;

    while ((esp_timer_get_time() / 1000 - start) < (int64_t)timeout_ms)
    {
        uint8_t b;
        if (uart_read_bytes(handle->config.uart_num, &b, 1, pdMS_TO_TICKS(10)) == 1)
        {
            if (hdr_pos < (int)(sizeof(hdr) - 1))
            {
                hdr[hdr_pos++] = (char)b;
                hdr[hdr_pos] = '\0';
            }

            // Bail out early if modem returned an error
            if (hdr_pos >= 7 && strstr(hdr, "ERROR"))
            {
                ESP_LOGE(TAG, "FS_READ: modem returned ERROR — hdr='%s'", hdr);
                return 0;
            }

            // Accept "+FSREAD: <len>\r\n" (A7672SA native format)
            const char *p = strstr(hdr, "+FSREAD:");
            if (p)
            {
                const char *nl = strstr(p, "\r\n");
                if (nl)
                {
                    data_len = (size_t)atoi(p + 8);
                    found = true;
                    break;
                }
            }

            // Also accept "CONNECT <len>\r\n" (alternative firmware format)
            p = strstr(hdr, "CONNECT ");
            if (p)
            {
                const char *nl = strstr(p, "\r\n");
                if (nl)
                {
                    data_len = (size_t)atoi(p + 8);
                    found = true;
                    break;
                }
            }
        }
    }

    if (!found || data_len == 0)
    {
        // Log first bytes received as hex to see what the modem actually sent
        char hex_dump[64] = {0};
        int dump_len = hdr_pos < 16 ? hdr_pos : 16;
        for (int i = 0; i < dump_len; i++)
            snprintf(hex_dump + i * 3, sizeof(hex_dump) - i * 3, "%02X ", (uint8_t)hdr[i]);
        ESP_LOGE(TAG, "FS_READ: sem header apos %lu ms — hdr_pos=%d hex='%s' str='%s'",
                 (unsigned long)(esp_timer_get_time() / 1000 - start), hdr_pos, hex_dump, hdr);
        return 0;
    }

    // Read exactly data_len bytes of binary firmware data
    size_t total = 0;
    while (total < data_len && (esp_timer_get_time() / 1000 - start) < (int64_t)timeout_ms)
    {
        int n = uart_read_bytes(handle->config.uart_num, buffer + total,
                                (uint32_t)(data_len - total), pdMS_TO_TICKS(500));
        if (n > 0)
            total += (size_t)n;
    }
    return total;
}

esp_err_t a7672sa_fs_list_files(a7672sa_handle_t handle, uint32_t timeout_ms)
{
    if (handle == NULL)
        return ESP_ERR_INVALID_ARG;
    a7672sa_send_command(handle, "FS_LIST", "AT+FSLS" GSM_NL);
    return a7672sa_wait_response(handle, timeout_ms) ? ESP_OK : ESP_FAIL;
}
