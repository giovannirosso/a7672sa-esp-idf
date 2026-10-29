/**
 * @file       uart_async_rxtxtasks_main.c
 * @brief      Example usage of A7672SA ESP-IDF Component
 * @author     Giovanni de Rosso Unruh
 * @date       01/2026
 *
 * This example demonstrates how to use the A7672SA component
 * for LTE modem communication with MQTT and HTTP support.
 *
 * SPDX-FileCopyrightText: 2026 Mindtech Tecnologia
 * SPDX-FileCopyrightText: 2026 Giovanni de Rosso Unruh
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "a7672sa.h"

static const char *TAG = "A7672SA_EXAMPLE";

/* Hardware pin definitions - adjust for your board */
#define LTE_EN_PIN GPIO_NUM_27 // Enable/Power pin
#define LTE_TX_PIN GPIO_NUM_17 // UART TX
#define LTE_RX_PIN GPIO_NUM_16 // UART RX

/* Modem handle */
static a7672sa_handle_t modem = NULL;

/* MQTT config and state */
#define MQTT_BROKER_HOST "test.mosquitto.org"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "esp32_a7672sa_client"
#define MQTT_TOPIC_TX "esp32/a7672sa/tx"
#define MQTT_TOPIC_RX "esp32/a7672sa/rx"
#define MQTT_KEEPALIVE_SEC 60
#define MQTT_RECONNECT_INTERVAL_MS 10000

static volatile bool mqtt_connected_event = false;
static volatile bool mqtt_reconnect_requested = true;
static volatile bool mqtt_pong_pending = false;
static volatile bool mqtt_force_disconnect = false;
static bool mqtt_subscribed = false;
static bool mqtt_hello_pending = false;
static int64_t mqtt_last_reconnect_attempt_ms = 0;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static bool mqtt_try_connect(void)
{
    ESP_LOGI(TAG, "Connecting to MQTT broker...");

    bool connected = a7672sa_mqtt_connect(modem,
                                          MQTT_BROKER_HOST,
                                          MQTT_BROKER_PORT,
                                          MQTT_CLIENT_ID,
                                          true,  // clean_session
                                          NULL,  // username (none)
                                          NULL,  // password (none)
                                          false, // ssl
                                          NULL,  // ca_name (none)
                                          MQTT_KEEPALIVE_SEC,
                                          30000);

    if (connected)
    {
        ESP_LOGI(TAG, "✅ MQTT connected");
        // Set flags immediately so the main loop reacts without waiting for
        // on_mqtt_status(CONNECTED), which fires from rx_task asynchronously.
        mqtt_connected_event = true;
        mqtt_reconnect_requested = false;
        mqtt_subscribed = false;
        mqtt_hello_pending = true;
    }
    else
    {
        ESP_LOGW(TAG, "⚠️ MQTT connect failed; retry in %d ms", MQTT_RECONNECT_INTERVAL_MS);
    }

    return connected;
}

static void mqtt_handle_connected_session(void)
{
    if (!mqtt_subscribed)
    {
        ESP_LOGI(TAG, "Subscribing to topic...");
        if (a7672sa_mqtt_subscribe(modem, MQTT_TOPIC_RX, 0, 10000))
        {
            mqtt_subscribed = true;
            ESP_LOGI(TAG, "✅ Subscribed to %s", MQTT_TOPIC_RX);
        }
        else
        {
            ESP_LOGW(TAG, "⚠️ Subscribe failed, will retry while connected");
        }
    }

    if (mqtt_hello_pending)
    {
        const char *msg = "Hello from ESP32 + A7672SA!";
        if (a7672sa_mqtt_publish(modem, MQTT_TOPIC_TX,
                                 (const uint8_t *)msg, strlen(msg), 0, 5000))
        {
            mqtt_hello_pending = false;
            ESP_LOGI(TAG, "✅ Hello message published");
        }
        else
        {
            ESP_LOGW(TAG, "⚠️ Hello publish failed, will retry while connected");
        }
    }

    if (mqtt_pong_pending)
    {
        const char *pong_msg = "pong";
        if (a7672sa_mqtt_publish(modem, MQTT_TOPIC_TX,
                                 (const uint8_t *)pong_msg, strlen(pong_msg), 0, 5000))
        {
            mqtt_pong_pending = false;
            ESP_LOGI(TAG, "📤 Responded with 'pong'");
        }
        else
        {
            ESP_LOGW(TAG, "⚠️ Pong publish failed, will retry");
        }
    }
}

/* ============================================================================
 * Callback Functions
 * ============================================================================ */

/**
 * @brief Callback for incoming MQTT messages
 */
static void on_mqtt_message(a7672sa_mqtt_message_t *message)
{
    ESP_LOGI(TAG, "📩 MQTT Message received!");
    ESP_LOGI(TAG, "   Topic: %s", message->topic);
    ESP_LOGI(TAG, "   Payload (%d bytes): %.*s",
             (int)message->length, (int)message->length, (char *)message->payload);

    // Defer all library calls to the main loop — this callback runs inside rx_task
    // and cannot block on AT commands without deadlocking the UART reader.
    if (strcmp((const char *)message->payload, "ping") == 0)
    {
        mqtt_pong_pending = true;
    }
    else if (strcmp((const char *)message->payload, "disconnect") == 0)
    {
        ESP_LOGW(TAG, "⚠️ Received 'disconnect' command");
        mqtt_force_disconnect = true;
    }
}

/**
 * @brief Callback for MQTT connection status changes
 */
static void on_mqtt_status(a7672sa_mqtt_status_t status)
{
    switch (status)
    {
    case A7672SA_MQTT_CONNECTED:
        ESP_LOGI(TAG, "✅ MQTT Connected!");
        mqtt_connected_event = true;
        mqtt_reconnect_requested = false;
        mqtt_subscribed = false;
        mqtt_hello_pending = true;
        break;
    case A7672SA_MQTT_DISCONNECTED:
        ESP_LOGW(TAG, "❌ MQTT Disconnected!");
        mqtt_connected_event = false;
        mqtt_reconnect_requested = true;
        break;
    case A7672SA_MQTT_TIMEOUT:
        ESP_LOGE(TAG, "⏰ MQTT Connection Timeout!");
        mqtt_connected_event = false;
        mqtt_reconnect_requested = true;
        break;
    case A7672SA_MQTT_CLIENT_USED:
        ESP_LOGW(TAG, "⚠️ MQTT Client already in use");
        mqtt_connected_event = false;
        mqtt_reconnect_requested = true;
        break;
    }
}

/**
 * @brief Callback for PS (Packet Switched) registration events
 */
static void on_ps_registration(a7672sa_registration_status_t status)
{
    const char *status_names[] = {
        "NOT_REGISTERED",
        "REGISTERED_HOME",
        "SEARCHING",
        "DENIED",
        "UNKNOWN",
        "REGISTERED_ROAMING",
        "SMS_ONLY_HOME",
        "SMS_ONLY_ROAMING"};

    if (status <= A7672SA_REG_REGISTERED_SMS_ONLY_ROAMING)
    {
        ESP_LOGI(TAG, "📶 Network Registration: %s", status_names[status]);
    }
}

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * @brief Print device information
 */
static void print_device_info(void)
{
    char buffer[64];

    ESP_LOGI(TAG, "=== Device Information ===");

    // IMEI
    if (a7672sa_get_imei(modem, buffer, sizeof(buffer), 2000) == ESP_OK)
        ESP_LOGI(TAG, "IMEI:     %s", buffer);

    // IMSI
    if (a7672sa_get_imsi(modem, buffer, sizeof(buffer), 2000) == ESP_OK)
        ESP_LOGI(TAG, "IMSI:     %s", buffer);

    // ICCID
    if (a7672sa_get_iccid(modem, buffer, sizeof(buffer), 2000) == ESP_OK)
        ESP_LOGI(TAG, "ICCID:    %s", buffer);

    // Provider
    if (a7672sa_get_provider_name(modem, buffer, sizeof(buffer), 2000) == ESP_OK)
        ESP_LOGI(TAG, "Provider: %s", buffer);

    // EPS (LTE) registration status
    const char *reg_names[] = {
        "NOT_REGISTERED", "REGISTERED_HOME", "SEARCHING", "DENIED",
        "UNKNOWN", "REGISTERED_ROAMING", "SMS_ONLY_HOME", "SMS_ONLY_ROAMING"};
    a7672sa_registration_status_t eps = a7672sa_eps_registration(modem);
    if (eps <= A7672SA_REG_REGISTERED_SMS_ONLY_ROAMING)
        ESP_LOGI(TAG, "EPS/LTE:  %s", reg_names[eps]);

    // Signal quality
    int rssi = a7672sa_signal_quality(modem, 2000);
    if (rssi > 0 && rssi < 99)
    {
        int dbm = -113 + (rssi * 2);
        ESP_LOGI(TAG, "Signal:   %d (%d dBm)", rssi, dbm);
    }

    // Local IPv4
    a7672sa_ip_addr_t ip;
    if (a7672sa_get_local_ip(modem, &ip, 2000) == ESP_OK)
        ESP_LOGI(TAG, "IPv4:     %d.%d.%d.%d",
                 ip.octets[0], ip.octets[1], ip.octets[2], ip.octets[3]);

    // Local IPv6
    if (a7672sa_get_local_ipv6(modem, buffer, sizeof(buffer), 2000) == ESP_OK)
        ESP_LOGI(TAG, "IPv6:     %s", buffer);

    ESP_LOGI(TAG, "==========================");
}

/* NTP configuration */
#define NTP_SERVER "pool.ntp.org"
#define NTP_TIMEZONE (-3) /* UTC-3 (Brasília) */
#define NTP_TIMEOUT_MS 10000

/**
 * @brief Attempt NTP synchronization and set system clock if successful
 */
static void sync_ntp_time(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== NTP Time Synchronization ===");
    ESP_LOGI(TAG, "   Server : %s | TZ: UTC%+d", NTP_SERVER, NTP_TIMEZONE);

    if (!a7672sa_set_ntp_server(modem, NTP_SERVER, NTP_TIMEZONE, NTP_TIMEOUT_MS))
    {
        ESP_LOGW(TAG, "⚠️  NTP sync request failed (AT+CNTP error)");
        return;
    }

    /* Allow the modem a brief window to complete the SNTP exchange */
    vTaskDelay(pdMS_TO_TICKS(2000));

    time_t epoch = a7672sa_get_ntp_time(modem, NTP_TIMEOUT_MS);
    if (epoch > 0)
    {
        /* Push the modem time into the ESP32 system clock */
        struct timeval tv = {.tv_sec = epoch, .tv_usec = 0};
        settimeofday(&tv, NULL);

        struct tm t;
        localtime_r(&epoch, &t);
        ESP_LOGI(TAG, "✅ NTP sync OK — %04d-%02d-%02d %02d:%02d:%02d (UTC%+d)",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min, t.tm_sec, NTP_TIMEZONE);
    }
    else
    {
        ESP_LOGW(TAG, "⚠️  Could not read time from modem after NTP sync");
    }
}

/* ============================================================================
 * Main Application
 * ============================================================================ */

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   A7672SA ESP-IDF Component Example   ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    ESP_LOGI(TAG, "");

    /* ========================================
     * Step 1: Configure modem
     * ======================================== */
    ESP_LOGI(TAG, "[1/7] Configuring modem...");

    a7672sa_config_t config = {
        .tx_pin = LTE_TX_PIN,
        .rx_pin = LTE_RX_PIN,
        .en_pin = LTE_EN_PIN,
        .uart_num = UART_NUM_1,
        .baud_rate = 115200,
        .rx_buffer_size = 2048,
    };

    /* ========================================
     * Step 2: Initialize modem
     * ======================================== */
    ESP_LOGI(TAG, "[2/7] Initializing modem...");

    esp_err_t ret = a7672sa_init(&config, &modem);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize modem: %s", esp_err_to_name(ret));
        return;
    }

    /* ========================================
     * Step 3: Register callbacks
     * ======================================== */
    ESP_LOGI(TAG, "[3/7] Registering callbacks...");

    a7672sa_set_mqtt_message_callback(modem, on_mqtt_message);
    a7672sa_set_mqtt_status_callback(modem, on_mqtt_status);
    a7672sa_set_ps_reg_callback(modem, on_ps_registration);

    /* ========================================
     * Enable debug mode to see AT commands
     * ======================================== */
    // Uncomment the line below to see all AT commands sent and responses received
    // a7672sa_set_debug(modem, true);

    /* ========================================
     * Step 4: Start modem (power on + tasks)
     * ======================================== */
    ESP_LOGI(TAG, "[4/7] Starting modem (this may take a few seconds)...");

    ret = a7672sa_begin(modem);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start modem: %s", esp_err_to_name(ret));
        a7672sa_deinit(modem);
        return;
    }

    /* ========================================
     * Step 5: Wait for modem ready
     * ======================================== */
    ESP_LOGI(TAG, "[5/7] Waiting for modem to be ready...");

    int timeout_count = 0;
    while (!a7672sa_is_ready(modem) && timeout_count < 60)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        timeout_count++;
        if (timeout_count % 10 == 0)
            ESP_LOGI(TAG, "   Still waiting... (%d s)", timeout_count / 2);
    }

    if (!a7672sa_is_ready(modem))
    {
        ESP_LOGE(TAG, "Modem not ready after 30 seconds!");
        a7672sa_deinit(modem);
        return;
    }

    ESP_LOGI(TAG, "✅ Modem is ready!");

    /* ========================================
     * Step 6: Test AT communication
     * ======================================== */
    ESP_LOGI(TAG, "[6/7] Testing AT communication...");

    if (a7672sa_test_at(modem, 2000))
        ESP_LOGI(TAG, "✅ AT communication OK");
    else
        ESP_LOGE(TAG, "❌ AT communication failed");

    /* ========================================
     * Check SIM card
     * ======================================== */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Checking SIM card...");

    while (!a7672sa_sim_ready(modem, 2000))
    {
        ESP_LOGI(TAG, "Waiting for SIM card...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "✅ SIM card ready");

    /* ========================================
     * Set network mode (LTE-only)
     * ======================================== */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Setting network mode...");

    if (a7672sa_set_network_mode(modem, A7672SA_NETWORK_LTE_ONLY, 10000))
        ESP_LOGI(TAG, "✅ Network mode: LTE-only");
    else
        ESP_LOGW(TAG, "⚠️ Could not set LTE-only mode, proceeding with auto");

    /* ========================================
     * Configure APN (adjust for your carrier)
     * ======================================== */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Configuring APN...");

    // Common APNs:
    // - TIM Brasil: "timbrasil.br"
    // - Claro Brasil: "claro.com.br"
    // - Vivo Brasil: "zap.vivo.com.br"
    // - Oi Brasil: "gprs.oi.com.br"

    if (a7672sa_set_apn(modem, "timbrasil.br", "", "", 10000))
        ESP_LOGI(TAG, "✅ APN configured");
    else
        ESP_LOGW(TAG, "⚠️ APN configuration may have failed");

    /* ========================================
     * Step 7: Wait for network registration
     * ======================================== */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "[7/7] Waiting for network registration (up to 60s)...");

    if (a7672sa_wait_network(modem, 60000))
    {
        ESP_LOGI(TAG, "✅ Network registered!");

        print_device_info();
        sync_ntp_time();

        /* ========================================
         * Test connectivity with ping
         * ======================================== */
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Testing connectivity (ping 8.8.8.8)...");

        if (a7672sa_ping(modem, "8.8.8.8", 10000))
            ESP_LOGI(TAG, "✅ Ping successful - Internet connectivity OK!");
        else
            ESP_LOGW(TAG, "⚠️ Ping failed - check network configuration");

/* ========================================
 * HTTP Example
 * ======================================== */
#if 1 // Set to 0 to disable HTTP test
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "=== HTTP Example ===");
        ESP_LOGI(TAG, "Performing HTTP GET request...");

        a7672sa_http_request_config_t http_config = A7672SA_HTTP_REQUEST_CONFIG_DEFAULT();
        http_config.url = "http://httpbin.org/get";
        http_config.method = A7672SA_HTTP_GET;

        uint32_t http_status = a7672sa_http_request(modem, &http_config, 30000);
        if (http_status == 200)
        {
            uint32_t content_size = a7672sa_http_response_size(modem);
            uint32_t header_size = a7672sa_http_response_header_size(modem);
            char etag[34] = {0};
            bool has_etag = (a7672sa_http_response_etag(modem, etag, sizeof(etag)) == ESP_OK && etag[0] != '\0');

            ESP_LOGI(TAG, "✅ HTTP 200 OK | content: %lu B | headers: %lu B%s%s",
                     (unsigned long)content_size, (unsigned long)header_size,
                     has_etag ? " | ETag: " : "", has_etag ? etag : "");

            // Read first 256 bytes of response body
            uint8_t body[257];
            size_t body_read = a7672sa_http_read_response(modem, body, 256, 0, 5000);
            if (body_read > 0)
            {
                body[body_read] = '\0';
                ESP_LOGI(TAG, "   Response preview:\n%s", (char *)body);
            }
        }
        else
        {
            ESP_LOGE(TAG, "❌ HTTP GET failed (status: %lu)", (unsigned long)http_status);
        }
        a7672sa_http_term(modem, 1000);
#endif

/* ========================================
 * MQTT Example - Connection Monitoring
 * ======================================== */
#if 1 // Set to 0 to disable MQTT monitoring
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "=== MQTT Monitor (test.mosquitto.org) ===");
        ESP_LOGI(TAG, "Keepalive: %d s | Reconnect interval: %d ms",
                 MQTT_KEEPALIVE_SEC, MQTT_RECONNECT_INTERVAL_MS);
        mqtt_reconnect_requested = true;
        mqtt_last_reconnect_attempt_ms = 0;
#endif
    }
    else
    {
        ESP_LOGE(TAG, "❌ Network registration timeout!");
        ESP_LOGE(TAG, "   Check: SIM card, antenna, signal strength");
    }

    /* ========================================
     * Main loop - periodic status check
     * ======================================== */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Entering main loop (signal + MQTT monitor)...");
    ESP_LOGI(TAG, "");

    int64_t last_signal_log_ms = 0;

    while (1)
    {
        bool ps_ready = a7672sa_ps_ready(modem);
        int64_t now = now_ms();

        // Deferred disconnect requested from on_mqtt_message callback.
        // Calls disconnect + release so the modem fully cleans up before the
        // next reconnect attempt; on_mqtt_status(DISCONNECTED) clears the flags.
        if (mqtt_force_disconnect)
        {
            mqtt_force_disconnect = false;
            ESP_LOGW(TAG, "⚠️ Disconnecting from MQTT broker (deferred)...");
            a7672sa_mqtt_disconnect(modem, 5000);
            a7672sa_mqtt_release_client(modem, 2000);
            // Reset reconnect timer so the broker has time to clean up the session
            // before we attempt to reconnect.
            mqtt_last_reconnect_attempt_ms = now_ms();
        }

        // Keep local MQTT state aligned with driver state
        if (mqtt_connected_event && !a7672sa_mqtt_is_connected(modem))
        {
            mqtt_connected_event = false;
            mqtt_reconnect_requested = true;
        }

#if 1 // MQTT monitor enabled
        if (ps_ready)
        {
            if (!mqtt_connected_event && mqtt_reconnect_requested &&
                (now - mqtt_last_reconnect_attempt_ms) >= MQTT_RECONNECT_INTERVAL_MS)
            {
                mqtt_last_reconnect_attempt_ms = now;
                mqtt_try_connect();
            }

            if (mqtt_connected_event)
                mqtt_handle_connected_session();
        }
        else
        {
            mqtt_connected_event = false;
            mqtt_reconnect_requested = true;
        }
#endif

        // Periodic signal quality check (every 30s)
        if ((now - last_signal_log_ms) >= 30000)
        {
            last_signal_log_ms = now;

            if (ps_ready)
            {
                int rssi = a7672sa_signal_quality(modem, 2000);
                if (rssi > 0 && rssi < 99)
                {
                    int dbm = -113 + (rssi * 2);
                    const char *quality;
                    if (dbm >= -70)
                        quality = "Excellent";
                    else if (dbm >= -85)
                        quality = "Good";
                    else if (dbm >= -100)
                        quality = "Fair";
                    else
                        quality = "Poor";

                    ESP_LOGI(TAG, "📶 Signal: %d dBm (%s)", dbm, quality);
                }
            }
            else
            {
                ESP_LOGW(TAG, "⚠️ Network not ready");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* ========================================
     * Cleanup (never reached in this example)
     * ======================================== */
    a7672sa_stop(modem);
    a7672sa_deinit(modem);
}
