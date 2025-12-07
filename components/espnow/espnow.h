#ifndef ESPNOW_H
#define ESPNOW_H

#include "esp_now.h"
#include "esp_crc.h"

#define ESPNOW_MAX_LENGTH 1500

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

extern uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN];
extern uint8_t receiver_mac_addr[ESP_NOW_ETH_ALEN];

typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

typedef enum {
    ACK,
    REQUEST
} espnow_msg_type_t;

typedef enum {
    ESPNOW_BROADCAST,
    ESPNOW_UNICAST,
    ESPNOW_MAX,
} espnow_data_type_t;

typedef struct __attribute__((packed)) {
    espnow_msg_type_t type;
    uint16_t seq_num;   // Sequence number to keep track of messages
    uint16_t crc;       // CRC checksum for data integrity
    size_t len;
    uint8_t data[ESPNOW_MAX_LENGTH];   // Message content
} espnow_data_t;


typedef struct {
    uint8_t *buffer;                        // Buffer containing the packet to send
    size_t len;                             // Length of the packet
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];     // Destination MAC address
} espnow_send_param_t;


void espnow_init(void);
void espnow_deinit(espnow_send_param_t *send_param);
void wifi_init(void);
esp_err_t espnow_send_once(const uint8_t *send_addr, const espnow_data_t *data_to_send);

#endif