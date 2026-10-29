/**
 * @file       a7672sa.h
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

#ifndef A7672SA_H_
#define A7672SA_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* ============================================================================
     * Defines
     * ============================================================================ */

#define A7672SA_UART_QUEUE_SIZE 10
#define A7672SA_DEFAULT_CID 1
#define A7672SA_MAX_OPERATORS 20
#define A7672SA_CMD_LOG_SIZE 48
#define A7672SA_CMD_DATA_SIZE 1024

#define GSM_NL "\r\n"
#define GSM_NM "+"
#define GSM_OK GSM_NL "OK" GSM_NL
#define GSM_ERROR GSM_NL "ERROR" GSM_NL

    /* ============================================================================
     * Type Definitions
     * ============================================================================ */

    /**
     * @brief MQTT message structure
     */
    typedef struct
    {
        char *topic;
        uint8_t *payload;
        size_t length;
    } a7672sa_mqtt_message_t;

    /**
     * @brief Command message for TX queue
     */
    typedef struct
    {
        char log_name[A7672SA_CMD_LOG_SIZE];
        uint8_t data[A7672SA_CMD_DATA_SIZE];
        size_t data_len;
    } a7672sa_cmd_message_t;

    /**
     * @brief HTTP response structure
     */
    typedef struct
    {
        uint32_t status_code;
        size_t header_size;
        size_t content_size;
        char etag[33];
    } a7672sa_http_response_t;

    /**
     * @brief Network operator information
     */
    typedef struct
    {
        int status;
        char long_name[32];
        char short_name[16];
        char numeric_code[8];
        int access_tech;
    } a7672sa_network_operator_t;

    /**
     * @brief Network registration status
     */
    typedef enum
    {
        A7672SA_REG_NOT_REGISTERED = 0,
        A7672SA_REG_REGISTERED_HOME = 1,
        A7672SA_REG_NOT_REGISTERED_SEARCHING = 2,
        A7672SA_REG_REGISTRATION_DENIED = 3,
        A7672SA_REG_UNKNOWN = 4,
        A7672SA_REG_REGISTERED_ROAMING = 5,
        A7672SA_REG_REGISTERED_SMS_ONLY_HOME = 6,
        A7672SA_REG_REGISTERED_SMS_ONLY_ROAMING = 7
    } a7672sa_registration_status_t;

    /**
     * @brief MQTT connection status
     */
    typedef enum
    {
        A7672SA_MQTT_CONNECTED = 0,
        A7672SA_MQTT_TIMEOUT = 1,
        A7672SA_MQTT_CLIENT_USED = 2,
        A7672SA_MQTT_DISCONNECTED = 3
    } a7672sa_mqtt_status_t;

    /**
     * @brief HTTP request methods
     */
    typedef enum
    {
        A7672SA_HTTP_GET = 0,
        A7672SA_HTTP_POST = 1,
        A7672SA_HTTP_HEAD = 2,
        A7672SA_HTTP_DELETE = 3,
        A7672SA_HTTP_PUT = 4
    } a7672sa_http_method_t;

    /**
     * @brief Network mode selection
     */
    typedef enum
    {
        A7672SA_NETWORK_AUTOMATIC = 2,
        A7672SA_NETWORK_GSM_ONLY = 13,
        A7672SA_NETWORK_WCDMA_ONLY = 14,
        A7672SA_NETWORK_LTE_ONLY = 38
    } a7672sa_network_mode_t;

    /**
     * @brief Callback types
     */
    typedef void (*a7672sa_mqtt_message_cb_t)(a7672sa_mqtt_message_t *message);
    typedef void (*a7672sa_mqtt_status_cb_t)(a7672sa_mqtt_status_t status);
    typedef void (*a7672sa_ps_reg_event_cb_t)(a7672sa_registration_status_t status);

    /**
     * @brief Modem configuration structure
     */
    typedef struct
    {
        gpio_num_t tx_pin;
        gpio_num_t rx_pin;
        gpio_num_t en_pin;
        uart_port_t uart_num;
        int32_t baud_rate;
        uint32_t rx_buffer_size;
    } a7672sa_config_t;

    /**
     * @brief IP address structure (for pure C compatibility)
     */
    typedef struct
    {
        uint8_t octets[4];
    } a7672sa_ip_addr_t;

    /**
     * @brief Modem handle (opaque type)
     */
    typedef struct a7672sa_handle_s *a7672sa_handle_t;

    /* ============================================================================
     * Default Configuration Macro
     * ============================================================================ */

#define A7672SA_CONFIG_DEFAULT() { \
    .tx_pin = GPIO_NUM_17,         \
    .rx_pin = GPIO_NUM_16,         \
    .en_pin = GPIO_NUM_4,          \
    .uart_num = UART_NUM_1,        \
    .baud_rate = 115200,           \
    .rx_buffer_size = 1024,        \
}

    /* ============================================================================
     * Initialization / Deinitialization
     * ============================================================================ */

    /**
     * @brief Create and initialize A7672SA modem handle
     *
     * @param config Pointer to configuration structure
     * @param out_handle Pointer to store the created handle
     * @return ESP_OK on success
     */
    esp_err_t a7672sa_init(const a7672sa_config_t *config, a7672sa_handle_t *out_handle);

    /**
     * @brief Get the singleton modem handle (set by the last successful a7672sa_init call)
     *
     * @return Handle or NULL if not yet initialized
     */
    a7672sa_handle_t a7672sa_get_instance(void);

    /**
     * @brief Start modem operation (enable power, start tasks)
     *
     * @param handle Modem handle
     * @return ESP_OK on success
     */
    esp_err_t a7672sa_begin(a7672sa_handle_t handle);

    /**
     * @brief Stop modem operation
     *
     * @param handle Modem handle
     * @return ESP_OK on success
     */
    esp_err_t a7672sa_stop(a7672sa_handle_t handle);

    /**
     * @brief Destroy modem handle and free resources
     *
     * @param handle Modem handle
     * @return ESP_OK on success
     */
    esp_err_t a7672sa_deinit(a7672sa_handle_t handle);

    /**
     * @brief Restart modem
     *
     * @param handle Modem handle
     * @param timeout_ms Timeout in milliseconds
     * @return ESP_OK on success
     */
    esp_err_t a7672sa_restart(a7672sa_handle_t handle, uint32_t timeout_ms);

    /* ============================================================================
     * Callback Registration
     * ============================================================================ */

    /**
     * @brief Register MQTT message callback
     */
    esp_err_t a7672sa_set_mqtt_message_callback(a7672sa_handle_t handle, a7672sa_mqtt_message_cb_t callback);

    /**
     * @brief Register MQTT status callback
     */
    esp_err_t a7672sa_set_mqtt_status_callback(a7672sa_handle_t handle, a7672sa_mqtt_status_cb_t callback);

    /**
     * @brief Register PS registration event callback
     */
    esp_err_t a7672sa_set_ps_reg_callback(a7672sa_handle_t handle, a7672sa_ps_reg_event_cb_t callback);

    /* ============================================================================
     * Status Functions
     * ============================================================================ */

    /**
     * @brief Check if modem is ready
     */
    bool a7672sa_is_ready(a7672sa_handle_t handle);

    /**
     * @brief Check if packet-switched network is ready
     */
    bool a7672sa_ps_ready(a7672sa_handle_t handle);

    /**
     * @brief Check if circuit-switched network is ready
     */
    bool a7672sa_cs_ready(a7672sa_handle_t handle);

    /**
     * @brief Get CS registration status
     */
    a7672sa_registration_status_t a7672sa_cs_registration(a7672sa_handle_t handle);

    /**
     * @brief Get PS registration status (2G/3G)
     */
    a7672sa_registration_status_t a7672sa_ps_registration(a7672sa_handle_t handle);

    /**
     * @brief Get EPS registration status (LTE)
     */
    a7672sa_registration_status_t a7672sa_eps_registration(a7672sa_handle_t handle);

    /**
     * @brief Check if PDN is active
     */
    bool a7672sa_is_pdn_active(a7672sa_handle_t handle, int cid);

    /* ============================================================================
     * AT Command Functions
     * ============================================================================ */

    /**
     * @brief Test AT communication
     */
    bool a7672sa_test_at(a7672sa_handle_t handle, uint32_t timeout_ms);

    /**
     * @brief Check if SIM card is ready
     */
    bool a7672sa_sim_ready(a7672sa_handle_t handle, uint32_t timeout_ms);

    /**
     * @brief Get signal quality (RSSI)
     *
     * @return Signal quality value (0-31, 99 = unknown)
     */
    int a7672sa_signal_quality(a7672sa_handle_t handle, uint32_t timeout_ms);

    /**
     * @brief Ping a host
     */
    bool a7672sa_ping(a7672sa_handle_t handle, const char *host, uint32_t timeout_ms);

    /* ============================================================================
     * Network Configuration
     * ============================================================================ */

    /**
     * @brief Set network mode
     */
    bool a7672sa_set_network_mode(a7672sa_handle_t handle, a7672sa_network_mode_t mode, uint32_t timeout_ms);

    /**
     * @brief Set APN configuration
     */
    bool a7672sa_set_apn(a7672sa_handle_t handle, const char *apn, const char *user, const char *password, uint32_t timeout_ms);

    /**
     * @brief Set network operator
     */
    bool a7672sa_set_operator(a7672sa_handle_t handle, bool automatic, const a7672sa_network_operator_t *op, uint32_t timeout_ms);

    /**
     * @brief Get available operators list
     *
     * @param handle Modem handle
     * @param operators Array to store operators
     * @param max_operators Maximum number of operators to return
     * @param out_count Pointer to store actual count
     * @param timeout_ms Timeout in milliseconds
     * @return ESP_OK on success
     */
    esp_err_t a7672sa_get_operator_list(a7672sa_handle_t handle, a7672sa_network_operator_t *operators,
                                        size_t max_operators, size_t *out_count, uint32_t timeout_ms);

    /**
     * @brief Wait for network registration
     */
    bool a7672sa_wait_network(a7672sa_handle_t handle, uint32_t timeout_ms);

    /* ============================================================================
     * Time Functions
     * ============================================================================ */

    /**
     * @brief Set NTP server
     */
    bool a7672sa_set_ntp_server(a7672sa_handle_t handle, const char *ntp_server, int time_zone, uint32_t timeout_ms);

    /**
     * @brief Get time from NTP
     */
    time_t a7672sa_get_ntp_time(a7672sa_handle_t handle, uint32_t timeout_ms);

    /* ============================================================================
     * Device Information
     * ============================================================================ */

    /**
     * @brief Get provider name
     *
     * @param handle Modem handle
     * @param buffer Buffer to store provider name
     * @param buffer_size Size of buffer
     * @param timeout_ms Timeout in milliseconds
     * @return ESP_OK on success
     */
    esp_err_t a7672sa_get_provider_name(a7672sa_handle_t handle, char *buffer, size_t buffer_size, uint32_t timeout_ms);

    /**
     * @brief Get IMSI (International Mobile Subscriber Identity) via AT+CIMI.
     *        Returns a 14-15 digit string identifying the subscriber.
     */
    esp_err_t a7672sa_get_imsi(a7672sa_handle_t handle, char *buffer, size_t buffer_size, uint32_t timeout_ms);

    /**
     * @brief Get IMEI
     */
    esp_err_t a7672sa_get_imei(a7672sa_handle_t handle, char *buffer, size_t buffer_size, uint32_t timeout_ms);

    /**
     * @brief Get ICCID
     */
    esp_err_t a7672sa_get_iccid(a7672sa_handle_t handle, char *buffer, size_t buffer_size, uint32_t timeout_ms);

    /**
     * @brief Get local IP address
     */
    esp_err_t a7672sa_get_local_ip(a7672sa_handle_t handle, a7672sa_ip_addr_t *ip, uint32_t timeout_ms);

    /**
     * @brief Get local IPv6 address
     */
    esp_err_t a7672sa_get_local_ipv6(a7672sa_handle_t handle, char *buffer, size_t buffer_size, uint32_t timeout_ms);

    /* ============================================================================
     * SSL/TLS Functions
     * ============================================================================ */

    /**
     * @brief Set CA certificate
     */
    bool a7672sa_set_ca_cert(a7672sa_handle_t handle, const char *ca_cert, const char *ca_name,
                             size_t cert_size, uint32_t timeout_ms);

    /* ============================================================================
     * MQTT Functions
     * ============================================================================ */

    /**
     * @brief Connect to MQTT broker
     */
    bool a7672sa_mqtt_connect(a7672sa_handle_t handle, const char *host, uint16_t port,
                              const char *client_id, bool clean_session,
                              const char *username, const char *password,
                              bool ssl, const char *ca_name, uint16_t keepalive,
                              uint32_t timeout_ms);

    /**
     * @brief Disconnect from MQTT broker
     */
    bool a7672sa_mqtt_disconnect(a7672sa_handle_t handle, uint32_t timeout_ms);

    /**
     * @brief Release MQTT client
     */
    bool a7672sa_mqtt_release_client(a7672sa_handle_t handle, uint32_t timeout_ms);

    /**
     * @brief Publish MQTT message
     */
    bool a7672sa_mqtt_publish(a7672sa_handle_t handle, const char *topic,
                              const uint8_t *data, size_t len,
                              uint16_t qos, uint32_t timeout_ms);

    /**
     * @brief Subscribe to MQTT topic
     */
    bool a7672sa_mqtt_subscribe(a7672sa_handle_t handle, const char *topic,
                                uint16_t qos, uint32_t timeout_ms);

    /**
     * @brief Subscribe to multiple MQTT topics
     */
    bool a7672sa_mqtt_subscribe_topics(a7672sa_handle_t handle, const char **topics,
                                       int n_topics, uint16_t qos, uint32_t timeout_ms);

    /**
     * @brief Check if MQTT is connected
     */
    bool a7672sa_mqtt_is_connected(a7672sa_handle_t handle);

    /* ============================================================================
     * HTTP Functions
     * ============================================================================ */

    /**
     * @brief HTTP request configuration
     */
    typedef struct
    {
        const char *url;
        a7672sa_http_method_t method;
        bool save_to_fs;
        bool ssl;
        // authmode 1 (valida cert do servidor com ca_name). false = authmode 0,
        // sem validação — único caminho quando a cadeia do servidor é ECDSA,
        // que o stack TLS do modem não suporta (falha 710 imediata).
        bool verify_server;
        const char *ca_name;
        const char *user_data;
        size_t user_data_size;
        uint32_t conn_timeout;
        uint32_t recv_timeout;
        const char *content_type;
        const char *accept_type;
        uint8_t read_mode;
        const char *post_data;
        size_t post_data_size;
    } a7672sa_http_request_config_t;

#define A7672SA_HTTP_REQUEST_CONFIG_DEFAULT() { \
    .url = NULL,                                \
    .method = A7672SA_HTTP_GET,                 \
    .save_to_fs = false,                        \
    .ssl = false,                               \
    .verify_server = true,                      \
    .ca_name = "ca.pem",                        \
    .user_data = "",                            \
    .user_data_size = 0,                        \
    .conn_timeout = 120,                        \
    .recv_timeout = 120,                        \
    .content_type = "text/plain",               \
    .accept_type = "*/*",                       \
    .read_mode = 0,                             \
    .post_data = "",                            \
    .post_data_size = 0,                        \
}

    /**
     * @brief Perform HTTP request
     *
     * @return HTTP status code or 0 on error
     */
    uint32_t a7672sa_http_request(a7672sa_handle_t handle, const a7672sa_http_request_config_t *config,
                                  uint32_t timeout_ms);

    /**
     * @brief Perform HTTP request and save to file
     */
    uint32_t a7672sa_http_request_file(a7672sa_handle_t handle, const a7672sa_http_request_config_t *config,
                                       const char *filename, uint32_t timeout_ms);

    /**
     * @brief Read HTTP response
     */
    size_t a7672sa_http_read_response(a7672sa_handle_t handle, uint8_t *buffer,
                                      size_t read_size, size_t offset, uint32_t timeout_ms);

    /**
     * @brief Get HTTP response size
     */
    uint32_t a7672sa_http_response_size(a7672sa_handle_t handle);

    /**
     * @brief Get HTTP response header size
     */
    uint32_t a7672sa_http_response_header_size(a7672sa_handle_t handle);

    /**
     * @brief Get HTTP response ETag
     */
    esp_err_t a7672sa_http_response_etag(a7672sa_handle_t handle, char *buffer, size_t buffer_size);

    /**
     * @brief Terminate HTTP session
     */
    bool a7672sa_http_term(a7672sa_handle_t handle, uint32_t timeout_ms);

    /* ============================================================================
     * File System Functions
     * ============================================================================ */

    /**
     * @brief Open file on modem filesystem
     *
     * @param mode 0=create/open, 1=create/overwrite, 2=open read-only
     */
    bool a7672sa_fs_open(a7672sa_handle_t handle, const char *filename, uint16_t mode, uint32_t timeout_ms);

    /**
     * @brief Close file
     */
    bool a7672sa_fs_close(a7672sa_handle_t handle, uint32_t timeout_ms);

    /**
     * @brief Get file size
     */
    uint32_t a7672sa_fs_size(a7672sa_handle_t handle, const char *filename, uint32_t timeout_ms);

    /**
     * @brief Delete file
     */
    bool a7672sa_fs_delete(a7672sa_handle_t handle, const char *filename, uint32_t timeout_ms);

    /**
     * @brief Read from file
     */
    size_t a7672sa_fs_read(a7672sa_handle_t handle, uint8_t *buffer, size_t read_size, uint32_t timeout_ms);

    /**
     * @brief List files on modem filesystem
     */
    esp_err_t a7672sa_fs_list_files(a7672sa_handle_t handle, uint32_t timeout_ms);

    /* ============================================================================
     * Low-Level Functions
     * ============================================================================ */

    /**
     * @brief Send raw AT command
     */
    esp_err_t a7672sa_send_command(a7672sa_handle_t handle, const char *log_name, const char *data);

    /**
     * @brief Send raw binary data
     */
    esp_err_t a7672sa_send_data(a7672sa_handle_t handle, const char *log_name, const uint8_t *data, size_t len);

    /**
     * @brief Wait for AT response
     */
    bool a7672sa_wait_response(a7672sa_handle_t handle, uint32_t timeout_ms);

    /**
     * @brief Wait for input prompt (>)
     */
    bool a7672sa_wait_input(a7672sa_handle_t handle, uint32_t timeout_ms);

    /**
     * @brief Lock RX mutex
     */
    void a7672sa_rx_lock(a7672sa_handle_t handle, uint32_t timeout_ms);

    /**
     * @brief Unlock RX mutex
     */
    void a7672sa_rx_unlock(a7672sa_handle_t handle);

    /**
     * @brief Lock publish mutex
     */
    bool a7672sa_publish_lock(a7672sa_handle_t handle, uint32_t timeout_ms);

    /**
     * @brief Unlock publish mutex
     */
    void a7672sa_publish_unlock(a7672sa_handle_t handle);

    /**
     * @brief Reinitialize UART
     */
    esp_err_t a7672sa_reinit_uart(a7672sa_handle_t handle, uint32_t buffer_size, bool at_ready);

    /**
     * @brief Deinitialize UART
     */
    esp_err_t a7672sa_deinit_uart(a7672sa_handle_t handle);

    /**
     * @brief Stop UART tasks and resize driver buffer — does NOT recreate tasks.
     *        Use before direct binary reads (e.g. OTA via modem FS).
     */
    esp_err_t a7672sa_resize_uart(a7672sa_handle_t handle, uint32_t buffer_size);

    /**
     * @brief Set silent mode
     */
    void a7672sa_set_silent_mode(a7672sa_handle_t handle, bool mode);

    /**
     * @brief Enable or disable debug mode (shows AT commands sent and responses received)
     *
     * @param handle Modem handle
     * @param enable true to enable debug output, false to disable
     */
    void a7672sa_set_debug(a7672sa_handle_t handle, bool enable);

    /**
     * @brief Check if debug mode is enabled
     *
     * @param handle Modem handle
     * @return true if debug is enabled, false otherwise
     */
    bool a7672sa_is_debug_enabled(a7672sa_handle_t handle);

    /**
     * @brief Get AT response buffer (for parsing)
     */
    const char *a7672sa_get_response(a7672sa_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // A7672SA_H_
