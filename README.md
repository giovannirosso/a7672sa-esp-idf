# A7672SA — Driver ESP-IDF para o modem LTE SIMCOM A7672SA

Driver nativo ESP-IDF (C) para o modem LTE **SIMCOM A7672SA**, com suporte a MQTT, HTTP/HTTPS, sistema de arquivos do modem, SSL/TLS e sincronização NTP. Não depende de Arduino.

Este README é para quem quer usar o driver num projeto ESP-IDF ou contribuir com ele.

## Sumário

- [Características](#características)
- [Requisitos](#requisitos)
- [Instalação](#instalação)
- [Início rápido](#início-rápido)
- [Configuração](#configuração)
- [Arquitetura interna](#arquitetura-interna)
- [API](#api)
- [Tipos e enums](#tipos-e-enums)
- [Tratamento de erros](#tratamento-de-erros)
- [Limitações conhecidas](#limitações-conhecidas)
- [Projeto de exemplo](#projeto-de-exemplo)
- [Referência de comandos AT](#referência-de-comandos-at)
- [Licença](#licença)

---

## Características

- **100% ESP-IDF nativo**: sem Arduino e sem wrappers externos.
- **Arquitetura assíncrona**: duas tasks FreeRTOS dedicadas (RX e TX) com fila de comandos.
- **MQTT**: connect, publish, subscribe, QoS 0/1, SSL, múltiplos tópicos e callbacks.
- **HTTP/HTTPS**: GET, POST, PUT, DELETE e HEAD, com SSL, ETag e resposta salva no FS do modem.
- **Sistema de arquivos**: leitura e escrita no FS interno do modem.
- **SSL/TLS**: upload de certificado CA, SNI e validação opcional do servidor (HTTPS).
- **NTP**: sincronização de horário pela rede LTE.
- **Informações do SIM e da rede**: IMEI, ICCID, IMSI, nome da operadora, IP e qualidade de sinal.
- **Payloads binários preservados**: o parser trabalha por comprimento e não filtra bytes `\r`, `\n` ou `0x00`, o que permite payloads como Protobuf.
- **Handle opaco**: API com mutex interno por handle.

---

## Requisitos

- ESP-IDF >= 5.0
- ESP32 com uma UART livre e um GPIO para o pino de power/enable do modem
- Módulo SIMCOM A7672SA com SIM card ativo

---

## Instalação

O repositório é o próprio componente: a raiz contém `CMakeLists.txt`, `idf_component.yml`, `Kconfig`, `a7672sa.c` e `include/`.

### Opção 1: gerenciador de componentes do ESP-IDF (via git)

No `main/idf_component.yml` do seu projeto:

```yaml
dependencies:
  a7672sa:
    git: "https://github.com/giovannirosso/a7672sa-esp-idf.git"
    version: "main"  # ou uma tag/commit específico
```

### Opção 2: submódulo git

```bash
git submodule add https://github.com/giovannirosso/a7672sa-esp-idf.git components/a7672sa
```

O ESP-IDF detecta automaticamente os componentes dentro de `components/`.

### Opção 3: cópia

Copie a raiz deste repositório para `components/a7672sa` no seu projeto.

---

## Início rápido

```c
#include "a7672sa.h"

static void on_mqtt_message(a7672sa_mqtt_message_t *msg)
{
    printf("Tópico: %s | Payload (%d bytes)\n", msg->topic, (int)msg->length);
}

static void on_mqtt_status(a7672sa_mqtt_status_t status)
{
    if (status == A7672SA_MQTT_CONNECTED)
        printf("MQTT conectado\n");
    else if (status == A7672SA_MQTT_DISCONNECTED)
        printf("MQTT desconectado\n");
}

void app_main(void)
{
    a7672sa_handle_t modem = NULL;

    a7672sa_config_t cfg = {
        .tx_pin         = GPIO_NUM_17,
        .rx_pin         = GPIO_NUM_16,
        .en_pin         = GPIO_NUM_27,
        .uart_num       = UART_NUM_1,
        .baud_rate      = 115200,
        .rx_buffer_size = 2048,
    };

    ESP_ERROR_CHECK(a7672sa_init(&cfg, &modem));
    a7672sa_set_mqtt_message_callback(modem, on_mqtt_message);
    a7672sa_set_mqtt_status_callback(modem, on_mqtt_status);
    ESP_ERROR_CHECK(a7672sa_begin(modem));

    // Aguarda o modem ficar pronto (power-on + handshake AT)
    while (!a7672sa_is_ready(modem))
        vTaskDelay(pdMS_TO_TICKS(500));

    // Verifica o SIM e configura o APN da sua operadora
    a7672sa_sim_ready(modem, 5000);
    a7672sa_set_apn(modem, "claro.com.br", "", "", 15000);
    a7672sa_wait_network(modem, 60000);

    // Conecta ao broker MQTT (sem SSL, sem autenticação)
    a7672sa_mqtt_connect(modem,
        "broker.example.com", 1883,
        "meu-device-id", true,
        NULL, NULL,
        false, NULL, 60, 30000);

    const char *hello = "olá mundo";
    a7672sa_mqtt_publish(modem, "dispositivo/status",
                         (const uint8_t *)hello, strlen(hello), 0, 5000);

    a7672sa_mqtt_subscribe(modem, "dispositivo/cmd", 0, 5000);

    // As mensagens recebidas chegam pelo callback
    while (true)
        vTaskDelay(pdMS_TO_TICKS(1000));
}
```

APNs comuns no Brasil: TIM `timbrasil.br`, Claro `claro.com.br`, Vivo `zap.vivo.com.br`, Oi `gprs.oi.com.br`.

---

## Configuração

### Pinos e UART

Os pinos, a porta UART, o baud rate e o tamanho do buffer RX são definidos em tempo de execução pela struct `a7672sa_config_t` passada para `a7672sa_init()`.

### Menuconfig

```
idf.py menuconfig
  → Component config
      → A7672SA LTE Modem Configuration
```

| Opção | Padrão | Descrição |
|-------|--------|-----------|
| `A7672SA_DEBUG_LOG` | `n` | Log verboso dos comandos AT enviados e recebidos |

---

## Arquitetura interna

```
                      ┌──────────────────────────────────┐
 Aplicação            │          a7672sa_handle_t        │
 (seu código)  ──────▶│                                  │
                      │  Mutex rx_guard (buffer AT)      │
                      │  Mutex publish_guard (MQTT pub)  │
                      │  Queue uart_queue (comandos)     │
                      │                                  │
                      │  ┌─────────┐   ┌─────────┐       │
                      │  │ TX Task │   │ RX Task │       │
                      │  │(dequeue │   │(lê UART,│       │
                      │  │ → UART) │   │ parseia)│       │
                      │  └────┬────┘   └────┬────┘       │
                      └───────┼─────────────┼────────────┘
                              │             │
                          UART_TX       UART_RX
                              │             │
                         ┌────┴─────────────┴─────┐
                         │  Modem SIMCOM A7672SA  │
                         │  (comandos AT)         │
                         └────────────────────────┘
```

### Fluxo de um comando AT

1. A função da API envia o comando com `a7672sa_send_command()`, que o coloca em `uart_queue`.
2. A **task TX** retira o comando da fila e escreve na UART.
3. A **task RX** lê bytes continuamente, acumula em `at_response` e chama o parser.
4. O parser seta flags (`at_ok`, `at_error`, `at_input`) e dispara callbacks quando aplicável.
5. A função da API espera a flag com poll de 10 ms, respeitando `timeout_ms`.

### Inicialização do modem (`a7672sa_begin`)

1. Configura o GPIO de enable.
2. Faz power-cycle: EN=1, espera 2 s, EN=0, espera 5 s.
3. Aloca o buffer de resposta.
4. Cria `rx_guard`, `publish_guard` e `uart_queue`.
5. Cria as tasks RX e TX.
6. A task TX envia `AT\r\n` a cada 500 ms até receber `OK`.
7. Envia `ATE0`, `AT+CMQTTSTART` e `AT+CGEREP=2,1`.

### Tasks FreeRTOS

| Task | Prioridade | Stack (bytes) |
|---|---|---|
| `a7672sa_rx` | `configMAX_PRIORITIES - 5` | 4608 |
| `a7672sa_tx` | `configMAX_PRIORITIES - 6` | 4096 |

---

## API

### Inicialização e ciclo de vida

```c
// Cria o handle e configura UART + GPIO. Chame antes de qualquer outra função.
esp_err_t a7672sa_init(const a7672sa_config_t *config, a7672sa_handle_t *out_handle);

// Retorna o handle singleton (se já inicializado via a7672sa_init).
a7672sa_handle_t a7672sa_get_instance(void);

// Liga o modem, cria as tasks RX/TX e aguarda "AT OK".
esp_err_t a7672sa_begin(a7672sa_handle_t handle);

// Para as tasks e desliga o modem (pino EN).
esp_err_t a7672sa_stop(a7672sa_handle_t handle);

// Libera todos os recursos do handle.
esp_err_t a7672sa_deinit(a7672sa_handle_t handle);

// Reinicia o modem e aguarda até timeout_ms para ele ficar pronto.
esp_err_t a7672sa_restart(a7672sa_handle_t handle, uint32_t timeout_ms);
```

### Callbacks

```c
// Mensagem MQTT recebida (tópico + payload binário).
// O payload é temporário: copie-o se precisar dele fora do callback.
esp_err_t a7672sa_set_mqtt_message_callback(a7672sa_handle_t handle,
                                             a7672sa_mqtt_message_cb_t callback);

// Mudança de estado do MQTT (conectado, desconectado, timeout...).
esp_err_t a7672sa_set_mqtt_status_callback(a7672sa_handle_t handle,
                                            a7672sa_mqtt_status_cb_t callback);

// Eventos de registro PS (rede de dados).
esp_err_t a7672sa_set_ps_reg_callback(a7672sa_handle_t handle,
                                       a7672sa_ps_reg_event_cb_t callback);
```

Assinaturas:

```c
typedef void (*a7672sa_mqtt_message_cb_t)(a7672sa_mqtt_message_t *message);
typedef void (*a7672sa_mqtt_status_cb_t)(a7672sa_mqtt_status_t status);
typedef void (*a7672sa_ps_reg_event_cb_t)(a7672sa_registration_status_t status);
```

### Status e diagnóstico

```c
bool a7672sa_is_ready(a7672sa_handle_t handle);                // Modem respondeu AT OK
bool a7672sa_ps_ready(a7672sa_handle_t handle);                // Rede de dados (EPS ou PS) pronta
bool a7672sa_cs_ready(a7672sa_handle_t handle);                // Rede CS (voz) pronta
bool a7672sa_is_pdn_active(a7672sa_handle_t handle, int cid);  // PDN ativo para o CID

bool a7672sa_test_at(a7672sa_handle_t handle, uint32_t timeout_ms);                    // AT
bool a7672sa_sim_ready(a7672sa_handle_t handle, uint32_t timeout_ms);                  // AT+CPIN?
int  a7672sa_signal_quality(a7672sa_handle_t handle, uint32_t timeout_ms);             // AT+CSQ (0–31)
bool a7672sa_ping(a7672sa_handle_t handle, const char *host, uint32_t timeout_ms);     // AT+CPING

a7672sa_registration_status_t a7672sa_cs_registration(a7672sa_handle_t handle);
a7672sa_registration_status_t a7672sa_ps_registration(a7672sa_handle_t handle);
a7672sa_registration_status_t a7672sa_eps_registration(a7672sa_handle_t handle);  // LTE
```

### Rede e APN

```c
// Configura o APN (AT+CGDCONT, AT+CGATT, AT+CGACT).
bool a7672sa_set_apn(a7672sa_handle_t handle,
                     const char *apn, const char *user, const char *password,
                     uint32_t timeout_ms);

// Modo de rede: automático, só GSM, só WCDMA ou só LTE.
bool a7672sa_set_network_mode(a7672sa_handle_t handle,
                              a7672sa_network_mode_t mode, uint32_t timeout_ms);

// Seleção manual ou automática de operadora (AT+COPS).
bool a7672sa_set_operator(a7672sa_handle_t handle, bool automatic,
                          const a7672sa_network_operator_t *op, uint32_t timeout_ms);

// Lista as operadoras disponíveis (AT+COPS=?). Pode levar até 3 minutos.
esp_err_t a7672sa_get_operator_list(a7672sa_handle_t handle,
                                    a7672sa_network_operator_t *operators,
                                    size_t max_operators, size_t *out_count,
                                    uint32_t timeout_ms);

// Aguarda registro EPS ou PS (polling de 1 s).
bool a7672sa_wait_network(a7672sa_handle_t handle, uint32_t timeout_ms);
```

### Informações do dispositivo

```c
esp_err_t a7672sa_get_imei(a7672sa_handle_t handle, char *buf, size_t len, uint32_t timeout_ms);
esp_err_t a7672sa_get_imsi(a7672sa_handle_t handle, char *buf, size_t len, uint32_t timeout_ms);
esp_err_t a7672sa_get_iccid(a7672sa_handle_t handle, char *buf, size_t len, uint32_t timeout_ms);
esp_err_t a7672sa_get_provider_name(a7672sa_handle_t handle, char *buf, size_t len, uint32_t timeout_ms);
esp_err_t a7672sa_get_local_ip(a7672sa_handle_t handle, a7672sa_ip_addr_t *ip, uint32_t timeout_ms);
esp_err_t a7672sa_get_local_ipv6(a7672sa_handle_t handle, char *buf, size_t len, uint32_t timeout_ms);
```

### MQTT

```c
// Conecta ao broker. ssl=true requer ca_name (nome do certificado no modem).
bool a7672sa_mqtt_connect(a7672sa_handle_t handle,
    const char *host, uint16_t port,
    const char *client_id, bool clean_session,
    const char *username, const char *password,
    bool ssl, const char *ca_name,
    uint16_t keepalive_s, uint32_t timeout_ms);

// Desconecta (AT+CMQTTDISC + AT+CMQTTREL + AT+CMQTTSTOP).
bool a7672sa_mqtt_disconnect(a7672sa_handle_t handle, uint32_t timeout_ms);

// Libera apenas o cliente (sem STOP).
bool a7672sa_mqtt_release_client(a7672sa_handle_t handle, uint32_t timeout_ms);

// Publica. Payload binário: nenhum byte é filtrado.
bool a7672sa_mqtt_publish(a7672sa_handle_t handle,
    const char *topic, const uint8_t *data, size_t len,
    uint16_t qos, uint32_t timeout_ms);

// Assina um tópico.
bool a7672sa_mqtt_subscribe(a7672sa_handle_t handle,
    const char *topic, uint16_t qos, uint32_t timeout_ms);

// Assina vários tópicos numa única chamada.
bool a7672sa_mqtt_subscribe_topics(a7672sa_handle_t handle,
    const char **topics, int n_topics, uint16_t qos, uint32_t timeout_ms);

bool a7672sa_mqtt_is_connected(a7672sa_handle_t handle);
```

Publicação MQTT em três etapas AT:

```
AT+CMQTTTOPIC=0,<len>   →  (aguarda ">")  →  envia o tópico  →  OK
AT+CMQTTPAYLOAD=0,<len> →  (aguarda ">")  →  envia o payload →  OK
AT+CMQTTPUB=0,<qos>,60  →  aguarda +CMQTTPUB: 0,0
```

### HTTP / HTTPS

```c
// Executa a requisição. Retorna o status HTTP (ex.: 200).
// A resposta fica no modem; leia com a7672sa_http_read_response().
uint32_t a7672sa_http_request(a7672sa_handle_t handle,
    const a7672sa_http_request_config_t *config, uint32_t timeout_ms);

// Variante que salva a resposta em arquivo no FS do modem.
uint32_t a7672sa_http_request_file(a7672sa_handle_t handle,
    const a7672sa_http_request_config_t *config,
    const char *filename, uint32_t timeout_ms);

// Lê um trecho da resposta (AT+HTTPREAD).
size_t a7672sa_http_read_response(a7672sa_handle_t handle,
    uint8_t *buffer, size_t read_size, size_t offset, uint32_t timeout_ms);

uint32_t  a7672sa_http_response_size(a7672sa_handle_t handle);
uint32_t  a7672sa_http_response_header_size(a7672sa_handle_t handle);
esp_err_t a7672sa_http_response_etag(a7672sa_handle_t handle, char *buf, size_t len);

// Encerra a sessão HTTP (AT+HTTPTERM). Chame após cada requisição.
bool a7672sa_http_term(a7672sa_handle_t handle, uint32_t timeout_ms);
```

Inicialize a configuração com `A7672SA_HTTP_REQUEST_CONFIG_DEFAULT()` e ajuste os campos necessários:

```c
typedef struct
{
    const char *url;
    a7672sa_http_method_t method;  // GET, POST, PUT, DELETE, HEAD
    bool save_to_fs;               // Salva a resposta no FS do modem
    bool ssl;
    bool verify_server;            // true: valida o certificado do servidor com ca_name
    const char *ca_name;           // Nome do certificado CA no modem (se ssl=true)
    const char *user_data;         // Cabeçalho customizado (AT+HTTPPARA="USERDATA")
    size_t user_data_size;
    uint32_t conn_timeout;         // segundos
    uint32_t recv_timeout;         // segundos
    const char *content_type;      // ex.: "application/json"
    const char *accept_type;
    uint8_t read_mode;
    const char *post_data;         // Corpo do POST
    size_t post_data_size;
} a7672sa_http_request_config_t;
```

Exemplo:

```c
a7672sa_http_request_config_t http = A7672SA_HTTP_REQUEST_CONFIG_DEFAULT();
http.url = "http://httpbin.org/get";
uint32_t status = a7672sa_http_request(modem, &http, 30000);
a7672sa_http_term(modem, 5000);
```

### Sistema de arquivos do modem

```c
// Abre um arquivo no modem. mode: 0=cria/abre, 1=cria/sobrescreve, 2=leitura.
bool     a7672sa_fs_open(a7672sa_handle_t handle, const char *filename, uint16_t mode, uint32_t timeout_ms);
bool     a7672sa_fs_close(a7672sa_handle_t handle, uint32_t timeout_ms);
uint32_t a7672sa_fs_size(a7672sa_handle_t handle, const char *filename, uint32_t timeout_ms);
bool     a7672sa_fs_delete(a7672sa_handle_t handle, const char *filename, uint32_t timeout_ms);

// Leitura binária.
size_t   a7672sa_fs_read(a7672sa_handle_t handle, uint8_t *buffer, size_t read_size, uint32_t timeout_ms);

esp_err_t a7672sa_fs_list_files(a7672sa_handle_t handle, uint32_t timeout_ms);
```

### SSL/TLS

```c
// Faz upload de um certificado CA para o modem (AT+CCERTDOWN).
// ca_name é o identificador usado depois nas conexões SSL (MQTT e HTTP).
bool a7672sa_set_ca_cert(a7672sa_handle_t handle,
    const char *ca_cert, const char *ca_name,
    size_t cert_size, uint32_t timeout_ms);
```

Configuração aplicada automaticamente:

| Conexão | SNI | Versão TLS | Validação do servidor |
|---|---|---|---|
| MQTT (`ssl=true`) | Sim | TLS 1.2 (`sslversion 3`) | **Não** (`authmode 0`), veja [Limitações conhecidas](#limitações-conhecidas) |
| HTTP (`ssl=true`) | Sim | Todas (`sslversion 4`) | Conforme `verify_server` (`authmode 1` ou `0`) |

### NTP

```c
// Configura o servidor NTP e o fuso horário (em quartos de hora; ex.: -12 = UTC-3).
bool a7672sa_set_ntp_server(a7672sa_handle_t handle,
    const char *ntp_server, int time_zone, uint32_t timeout_ms);

// Retorna o timestamp UNIX sincronizado pelo modem (AT+CCLK?).
time_t a7672sa_get_ntp_time(a7672sa_handle_t handle, uint32_t timeout_ms);
```

Sincronizando o relógio do sistema:

```c
a7672sa_set_ntp_server(modem, "pool.ntp.org", -12, 10000);  // UTC-3
time_t t = a7672sa_get_ntp_time(modem, 10000);
struct timeval tv = { .tv_sec = t };
settimeofday(&tv, NULL);
```

### Baixo nível

Funções para controle fino da UART e do buffer AT, úteis para OTA via LTE ou transferências grandes de arquivo.

```c
// Envia um comando AT (string) ou dados binários.
esp_err_t a7672sa_send_command(a7672sa_handle_t handle, const char *log_name, const char *data);
esp_err_t a7672sa_send_data(a7672sa_handle_t handle, const char *log_name, const uint8_t *data, size_t len);

// Aguarda "OK" ou "ERROR" no buffer de resposta.
bool a7672sa_wait_response(a7672sa_handle_t handle, uint32_t timeout_ms);

// Aguarda o prompt ">" (modo de entrada do modem).
bool a7672sa_wait_input(a7672sa_handle_t handle, uint32_t timeout_ms);

// Trava/destrava o buffer RX (mutex rx_guard).
// Necessário ao usar send_command + wait_response manualmente.
void a7672sa_rx_lock(a7672sa_handle_t handle, uint32_t timeout_ms);
void a7672sa_rx_unlock(a7672sa_handle_t handle);

// Trava/destrava o mutex de publicação MQTT.
bool a7672sa_publish_lock(a7672sa_handle_t handle, uint32_t timeout_ms);
void a7672sa_publish_unlock(a7672sa_handle_t handle);

// Redimensiona o buffer RX (ex.: OTA binário que precisa de buffer maior).
esp_err_t a7672sa_resize_uart(a7672sa_handle_t handle, uint32_t buffer_size);
esp_err_t a7672sa_reinit_uart(a7672sa_handle_t handle, uint32_t buffer_size, bool at_ready);
esp_err_t a7672sa_deinit_uart(a7672sa_handle_t handle);

// Conteúdo atual do buffer de resposta AT.
const char *a7672sa_get_response(a7672sa_handle_t handle);

// Modo silencioso: suprime a configuração automática de URC (legado).
void a7672sa_set_silent_mode(a7672sa_handle_t handle, bool mode);

// Log de debug dos comandos AT.
void a7672sa_set_debug(a7672sa_handle_t handle, bool enable);
bool a7672sa_is_debug_enabled(a7672sa_handle_t handle);
```

---

## Tipos e enums

```c
// Handle opaco: não acesse os membros internos.
typedef struct a7672sa_handle_s *a7672sa_handle_t;

typedef struct {
    gpio_num_t  tx_pin;
    gpio_num_t  rx_pin;
    gpio_num_t  en_pin;
    uart_port_t uart_num;
    int32_t     baud_rate;
    uint32_t    rx_buffer_size;
} a7672sa_config_t;

typedef struct {
    char    *topic;
    uint8_t *payload;
    size_t   length;
} a7672sa_mqtt_message_t;

typedef struct {
    uint32_t status_code;
    size_t   header_size;
    size_t   content_size;
    char     etag[33];
} a7672sa_http_response_t;

typedef struct {
    int  status;
    char long_name[32];
    char short_name[16];
    char numeric_code[8];
    int  access_tech;
} a7672sa_network_operator_t;

typedef struct {
    uint8_t octets[4];
} a7672sa_ip_addr_t;

typedef enum {
    A7672SA_REG_NOT_REGISTERED           = 0,
    A7672SA_REG_REGISTERED_HOME          = 1,
    A7672SA_REG_NOT_REGISTERED_SEARCHING = 2,
    A7672SA_REG_REGISTRATION_DENIED      = 3,
    A7672SA_REG_UNKNOWN                  = 4,
    A7672SA_REG_REGISTERED_ROAMING       = 5,
} a7672sa_registration_status_t;

typedef enum {
    A7672SA_MQTT_CONNECTED    = 0,
    A7672SA_MQTT_TIMEOUT      = 1,
    A7672SA_MQTT_CLIENT_USED  = 2,
    A7672SA_MQTT_DISCONNECTED = 3,
} a7672sa_mqtt_status_t;

typedef enum {
    A7672SA_HTTP_GET    = 0,
    A7672SA_HTTP_POST   = 1,
    A7672SA_HTTP_HEAD   = 2,
    A7672SA_HTTP_DELETE = 3,
    A7672SA_HTTP_PUT    = 4,
} a7672sa_http_method_t;

typedef enum {
    A7672SA_NETWORK_AUTOMATIC  = 2,
    A7672SA_NETWORK_GSM_ONLY   = 13,
    A7672SA_NETWORK_WCDMA_ONLY = 14,
    A7672SA_NETWORK_LTE_ONLY   = 38,
} a7672sa_network_mode_t;
```

---

## Tratamento de erros

| Condição | Comportamento |
|----------|---------------|
| `ERROR` / `CME ERROR` / `CMS ERROR` | `at_error=true`; a função retorna `false` ou `ESP_FAIL` |
| `+CMQTTSTART: 19` | Tratado como OK (MQTT já iniciado) |
| `+CMQTTCONNECT: 0,<code>` com code≠0 | Desconexão automática (DISC→REL→STOP) antes de retornar falha |
| `+CMQTTCONNLOST:` | Dispara `on_mqtt_status(A7672SA_MQTT_DISCONNECTED)` |
| `CGEV: EPS PDN DEACT <cid>` | `pdn_active[cid]=false`; dispara o callback PS |
| `CGEV: ME DETACH` | Todos os PDNs são marcados como inativos |
| Timeout em `wait_response` | Retorna `false`; o buffer é limpo para o próximo comando |

Códigos de erro de `+CMQTTCONNECT`:

| Código | Significado |
|--------|-------------|
| 0 | Sucesso |
| 1 | Protocolo inaceitável |
| 2 | Identificador de cliente rejeitado |
| 3 | Servidor indisponível |
| 4 | Usuário/senha incorretos |
| 5 | Não autorizado |
| 6 | Timeout de conexão |

---

## Limitações conhecidas

- **MQTT com SSL não valida o certificado do servidor**: a conexão MQTT usa `AT+CSSLCFG="authmode",0,0`. A conexão é cifrada, mas não protege contra man-in-the-middle. O `ca_name` é aplicado, mas não é verificado.
- **Cadeias ECDSA em HTTPS**: o stack TLS do modem não suporta cadeias de certificado ECDSA (falha 710 imediata). Nesses servidores, use `verify_server = false`.
- **Prioridade das tasks após `a7672sa_reinit_uart()`**: as tasks são recriadas com prioridades menores (`configMAX_PRIORITIES - 10` e `- 11`) do que na inicialização (`- 5` e `- 6`).

---

## Projeto de exemplo

`examples/basic/` é um projeto ESP-IDF completo que usa o componente da raiz do repositório (via `EXTRA_COMPONENT_DIRS`) e demonstra:

1. Inicialização do modem com callbacks registrados.
2. Verificação do SIM e configuração do APN (padrão: TIM Brasil).
3. Espera pelo registro na rede LTE.
4. Leitura de IMEI, ICCID, operadora, IP e qualidade de sinal.
5. Sincronização NTP com `settimeofday()`.
6. Ping para `8.8.8.8`.
7. Requisição HTTP para `httpbin.org`.
8. Conexão MQTT em `test.mosquitto.org:1883`, subscribe em `esp32/a7672sa/rx` e publish de "hello" e pong.
9. Monitoramento do sinal a cada 30 s.

Ajuste os pinos (`LTE_TX_PIN`, `LTE_RX_PIN`, `LTE_EN_PIN`) e o APN em `examples/basic/main/main.c`, e então:

```bash
cd examples/basic
idf.py set-target esp32
idf.py build
idf.py -p <PORTA> flash monitor
```

---

## Referência de comandos AT

Principais comandos usados pelo driver. Para os detalhes, consulte o *A76XX Series AT Command Manual*, disponível no site da SIMCom.

| Comando | Função |
|---------|--------|
| `AT` | Teste de comunicação |
| `ATE0` | Desabilita o echo |
| `AT+CPIN?` | Status do SIM |
| `AT+CSQ` | Qualidade de sinal (RSSI) |
| `AT+CREG/CGREG/CEREG` | Registro CS/PS/EPS |
| `AT+CGDCONT/CGATT/CGACT` | Configuração de APN e ativação PDP |
| `AT+CNMP` | Modo de rede (2G/3G/4G) |
| `AT+COPS` | Seleção de operadora |
| `AT+CGPADDR` | IP local |
| `AT+CGSN` | IMEI |
| `AT+CIMI` | IMSI |
| `AT+CICCID` | ICCID |
| `AT+CSPN?` | Nome da operadora |
| `AT+CPING` | Ping |
| `AT+CNTP` | Configuração NTP |
| `AT+CCLK?` | Horário da rede |
| `AT+CMQTTSTART/STOP` | Sessão MQTT |
| `AT+CMQTTACCQ/REL` | Acquire/release do cliente MQTT |
| `AT+CMQTTCONNECT/DISC` | Conexão/desconexão do broker |
| `AT+CMQTTTOPIC/PAYLOAD/PUB` | Publicação MQTT (3 etapas) |
| `AT+CMQTTSUBTOPIC/SUB` | Subscribe MQTT |
| `AT+HTTPINIT/TERM` | Sessão HTTP |
| `AT+HTTPPARA` | Parâmetros HTTP |
| `AT+HTTPACTION` | Executa a requisição HTTP |
| `AT+HTTPREAD` | Lê a resposta HTTP |
| `AT+HTTPDATA` | Upload do corpo do POST |
| `AT+CSSLCFG` | Configuração SSL |
| `AT+CCERTDOWN` | Upload de certificado CA |
| `AT+FSOPEN/CLOSE/READ/DEL` | Operações de arquivo |
| `AT+FSATTRI` | Tamanho de arquivo |
| `AT+FSLS` | Lista arquivos |
| `AT+CGEREP` | Relatório de eventos de pacote |

---

## Licença

Distribuído sob a licença MIT. Veja [LICENSE](./LICENSE).

Copyright (c) 2026 Mindtech Tecnologia e Giovanni de Rosso Unruh.

Autor: Giovanni de Rosso Unruh.
