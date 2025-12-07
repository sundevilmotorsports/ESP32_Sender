#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_crc.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include "espnow.h"
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <stdlib.h>

#define ESPNOW_WIFI_MODE    WIFI_MODE_STA
#define ESPNOW_WIFI_IF      ESP_IF_WIFI_STA
#define ESPNOW_CHANNEL      1
#define ACK_SEND_DELAY      CONFIG_ACK_SEND_DELAY
#define PAYLOAD_BUFFER_SIZE 50  // Max size of the data payload

#define SEND_QUEUE_SIZE     CONFIG_SEND_QUEUE_SIZE
#define RECV_QUEUE_SIZE     CONFIG_RECV_QUEUE_SIZE
#define MAX_QUEUE_DELAY 512

#if CONFIG_ESPNOW_ROLE_STATION
    #define ESPNOW_WIFI_MODE WIFI_MODE_STA
    #define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
    #define ROLE             "STATION"
    static const char *TAG = "ESPNOW_STATION";
#else
    #define ESPNOW_WIFI_MODE WIFI_MODE_STA
    #define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
    #define ROLE             "RECEIVER"
    static const char *TAG = "ESPNOW_RECEIVER";
#endif

static QueueHandle_t espnow_send_queue = NULL;
static QueueHandle_t espnow_recv_queue = NULL;

// Broadcast + Receiver MAC Address
uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t receiver_mac_addr[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    if (tx_info == NULL || tx_info->des_addr == NULL) {
        ESP_LOGE(TAG, "Send CB error: Argument is NULL");
        return;
    }

    // The MAC address is inside the tx_info struct in this version
    const uint8_t *mac_addr = tx_info->des_addr;

    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "Send to " MACSTR " successful", MAC2STR(mac_addr));
    } else {
        ESP_LOGW(TAG, "Send to " MACSTR " failed, status: %d", MAC2STR(mac_addr), status);
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr)) {
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(espnow_recv_queue, &evt, MAX_QUEUE_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        return;
    }
}

esp_err_t espnow_send_once(const uint8_t *send_addr, const espnow_data_t *data_to_send) {
    if (data_to_send == NULL) {
        ESP_LOGE(TAG, "Data to send is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Send the data
    esp_err_t result = esp_now_send((const uint8_t *)send_addr, (const uint8_t *)data_to_send, sizeof(espnow_data_t));
    
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW single send error: %s", esp_err_to_name(result));
    }
    
    return result;
}

static void espnow_send_ack_task(void *pvParameters) {
    uint32_t message_count = 0;
    espnow_data_t *packet = malloc(sizeof(espnow_data_t));
    if (packet == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for ACK packet");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        int payload_len = snprintf((char*)packet->data, sizeof(packet->data), "ACK; Seq Num: %lu", message_count);
        packet->len = payload_len;
        packet->type = ACK;
        packet->seq_num = message_count++;
        packet->crc = 0;
        size_t total_size = sizeof(espnow_data_t) - sizeof(packet->data) + packet->len;
        packet->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)packet, total_size);

        if (esp_now_send(s_broadcast_mac, (uint8_t *)packet, total_size) != ESP_OK) {
            ESP_LOGE(TAG, "ESP-NOW send error");
        }

        vTaskDelay(pdMS_TO_TICKS(ACK_SEND_DELAY));
    }
    free(packet);
    vTaskDelete(NULL);
}

static void espnow_recv_task(void *pvParameters) {
    espnow_event_t evt;
    int recv_seq = 0;

    while(1) {
        if (xQueueReceive(espnow_recv_queue, &evt, portMAX_DELAY) == pdTRUE) {
            espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
            espnow_data_t *packet = (espnow_data_t *)evt.info.recv_cb.data;
            if (packet->type == ACK) {
                ESP_LOGI(TAG, "Received ACK from: "MACSTR"", MAC2STR(recv_cb->mac_addr));

                if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                    if (peer == NULL) {
                        ESP_LOGE(TAG, "Malloc peer information fail");
                        esp_now_deinit();
                        vTaskDelete(NULL);
                    }
                    memset(peer, 0, sizeof(esp_now_peer_info_t));
                    peer->channel = CONFIG_ESPNOW_CHANNEL;
                    peer->ifidx = ESPNOW_WIFI_IF;
                    peer->encrypt = false;
                    memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                    memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                    free(peer);
                } 
                // Change receiver mac addr
                memcpy(receiver_mac_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
            } else if (packet->type == REQUEST) {
                ESP_LOGI(TAG, "Receive %dth send f}rom: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                // for (size_t i = 0; i < recv_cb->data_len-4; i++) {
                //     if (isprint(recv_cb->data[i])) {
                //         printf("%c", recv_cb->data[i]);
                //     }
                // }
                printf("Received Message:  %.*s\n", packet->len, (char*)packet->data);
                recv_seq++;
            } else {
                ESP_LOGE(TAG, "INCORRECT PACKET TYPE DETECTED: %d", packet->type);
                esp_now_deinit();
                vTaskDelete(NULL);
            }
            if (recv_cb->data) {
                free(recv_cb->data);
                recv_cb->data = NULL;
            }
        }
    }
}

void espnow_init(void) {
    ESP_ERROR_CHECK(esp_now_init());

    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
        ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
    #endif

    esp_now_peer_info_t peer_info = {};
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.ifidx = ESPNOW_WIFI_IF;
    peer_info.encrypt = false;
    memcpy(peer_info.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    // Start sends for ACKs
    espnow_send_param_t *send_param = malloc(sizeof(espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for send parameters");
        esp_now_deinit();
        return;
    }
    memset(send_param, 0, sizeof(espnow_send_param_t));

    send_param->len = sizeof(espnow_data_t);
    send_param->buffer = malloc(send_param->len);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for send buffer");
        free(send_param);
        esp_now_deinit();
        return;
    }

    memcpy(send_param->dest_mac, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    espnow_recv_queue = xQueueCreate(CONFIG_RECV_QUEUE_SIZE, sizeof(espnow_event_t));

    xTaskCreate(espnow_recv_task, "espnow_recv_task", 4096, NULL, 4, NULL);
    if (strcmp(ROLE, "RECEIVER") == 0) {
        xTaskCreate(espnow_send_ack_task, "espnow_send_ack_task", 4096, NULL, 4, NULL);
    }
}

void espnow_deinit(espnow_send_param_t *send_param) {
    if (send_param != NULL) {
        free(send_param->buffer);
        free(send_param);
    }
    esp_now_deinit();
}