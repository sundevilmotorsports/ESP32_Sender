#include <stdio.h>
#include <string.h>
#include "can.h"
#include "espnow.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_crc.h"
#include "freertos/semphr.h"
#include "esp_random.h"

#define USE_REAL_DATA false

#define SEND_INTERVAL_MS 50

#define ID_DRS          0x01
#define ID_IMU_GYRO     0x02
#define ID_IMU_ACCEL    0x03
#define ID_WHEEL_FL     0x04
#define ID_WHEEL_FR     0x05
#define ID_WHEEL_RR     0x06
#define ID_WHEEL_RL     0x07
#define ID_SG_FL        0x08
#define ID_SG_FR        0x09
#define ID_SG_RR        0x0A
#define ID_SG_RL        0x0B
#define ID_ENG_F0       0x0C // rpm oil
#define ID_ENG_F1       0x0D // tps, wspd
#define ID_ENG_F2       0x0E // aps
#define ID_SHIFTER      0x0F

static const char *TAG = "SENDER_MAIN";
static uint16_t s_seq_num = 0;

uint8_t general[ESPNOW_MAX_LENGTH];
size_t  general_len = 0;
SemaphoreHandle_t general_mutex;

uint8_t f0[3];
uint8_t f1[3];
uint8_t f2[3];

static void send_general_packet(void) {
    if (general_len == 0) return;
    espnow_data_t pkt;
    pkt.type    = (uint8_t)4; // Telemetry
    pkt.seq_num = s_seq_num++;
    pkt.crc     = 0;
    pkt.len     = (uint8_t)(general_len > ESPNOW_MAX_LENGTH ? ESPNOW_MAX_LENGTH : general_len);
    memcpy(pkt.data, general, pkt.len);
    // CRC over header + data: type(1)+seq_num(2)+crc(2)+len(1)+data(len) = 6+len bytes
    size_t total_size = 6 + pkt.len;
    pkt.crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)&pkt, total_size);
    esp_now_send(s_broadcast_mac, (uint8_t *)&pkt, total_size);
}

void pack_general(uint8_t header_id, uint8_t *data, size_t len) {
    if (xSemaphoreTake(general_mutex, portMAX_DELAY)) {
        if ((general_len + 1 + len) < ESPNOW_MAX_LENGTH) {
            general[general_len++] = header_id;
            memcpy(&general[general_len], data, len);
            general_len += len;
        } else {
            // general is full — flush it first, then start fresh
            send_general_packet();
            general_len = 0;
            memset(general, 0, ESPNOW_MAX_LENGTH);
            if ((1 + len) < ESPNOW_MAX_LENGTH) {
                general[general_len++] = header_id;
                memcpy(&general[general_len], data, len);
                general_len += len;
            }
        }
        xSemaphoreGive(general_mutex);
    }
}

static void process_can_message(twai_frame_t *message) {
    uint8_t data[8];
    memcpy(data, message->buffer, message->header.dlc);
    switch(message->header.id) {
        case 0x35F:
            pack_general(ID_DRS, data, 1);
            break;
        //IMU Data Handling
        case 0x360:
            pack_general(ID_IMU_GYRO, data, 6);
            break;
        case 0x361:
            pack_general(ID_IMU_ACCEL, data, 6);
            break;
        case 0x363:
            //Front Left Wheel Board
            pack_general(ID_WHEEL_FL, data, 6);
            break;
        case 0x364:
            //Front Right Wheel Board
            pack_general(ID_WHEEL_FR, data, 6);
            break;
            
        case 0x365:
            //Rear Right Wheel Board
            pack_general(ID_WHEEL_RR, data, 6);
            break;
            
        case 0x366:
            //Rear Left Wheel Board
            pack_general(ID_WHEEL_RL, data, 6);
            break;
            
        case 0x4e2:
            //Front Left String Gauge
            pack_general(ID_SG_FL, data, 2);
            break;
            
        case 0x4e3:
            //Front Right String Gauge
            pack_general(ID_SG_FR, data, 2);
            break;
            
        case 0x4e4:
            //Rear Right String Gauge
            pack_general(ID_SG_RR, data, 2);
            break;
            
        case 0x4e5:
            //Rear Left String Gauge
            pack_general(ID_SG_RL, data, 2);
            break;
            
        case 0x3e8:
            //Engine CAN Stream 2
            switch(data[0]){
                case 0x0:
                    // engine_speed = message->data[1] << 8 | message->data[2];
                    // packet->ect = data[3];
                    // // oilTemp = message->data[4];
                    // packet->oilPress = data[5] << 8 | data[6];
                    f0[0] = data[3];
                    f0[1] = data[5];
                    f0[2] = data[6];
                    pack_general(ID_ENG_F0, f0, 3);
                    break;

                case 0x1:
                    // packet->tps = data[2];
                    // packet->driven_wspd = data[4] << 8 | data[5];
                    f1[0] = data[2];
                    f1[1] = data[4];
                    f1[2] = data[5];
                    pack_general(ID_ENG_F1, f1, 3);
                    break;
                    
                case 0x2:
                // packet->aps = data[1];
                    f2[0] = data[1];
                    pack_general(ID_ENG_F2, f2, 1);
                    break;
            }
            break;

        case 0x40:
            // Shifter Data
            // packet->shift0 = data[0];
            // packet->shift1 = data[1];
            // packet->shift2 = data[2];
            // if((packet->shift1 != 1) | (packet->shift2 != 1)) {
            //     TXDAT[1] = packet->shift1;
            //     TXDAT[2] = packet->shift2;
            // }
            pack_general(ID_SHIFTER, data, 3);
            break;
    }
}

void clear_general_task(void *pvParameter) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
        if (xSemaphoreTake(general_mutex, portMAX_DELAY)) {
            if (general_len > 0) {
                send_general_packet();
                general_len = 0;
                memset(general, 0, ESPNOW_MAX_LENGTH);
            }
            xSemaphoreGive(general_mutex);
        }
    }
}

void fake_can_generator_task(void *pvParameter) {
    uint8_t dummy[8];

    while(1) {
        // fill up dummy with 8 random bytes
        *(uint32_t*)dummy = esp_random(); 
        *(uint32_t*)(dummy+4) = esp_random();
        int msg_type = esp_random() % 15; //0-14

        switch(msg_type) {
            case 0: pack_general(ID_DRS, dummy, 1); break;
            case 1: pack_general(ID_IMU_GYRO, dummy, 6); break;
            case 2: pack_general(ID_IMU_ACCEL, dummy, 6); break;
            case 3: pack_general(ID_WHEEL_FL, dummy, 6); break;
            case 4: pack_general(ID_WHEEL_FR, dummy, 6); break;
            case 5: pack_general(ID_WHEEL_RR, dummy, 6); break;
            case 6: pack_general(ID_WHEEL_RL, dummy, 6); break;
            case 7: pack_general(ID_SG_FL, dummy, 2); break;
            case 8: pack_general(ID_SG_FR, dummy, 2); break;
            case 9: pack_general(ID_SG_RR, dummy, 2); break;
            case 10: pack_general(ID_SG_RL, dummy, 2); break;
            case 11: pack_general(ID_ENG_F0, dummy, 3); break;
            case 12: pack_general(ID_ENG_F1, dummy, 3); break;
            case 13: pack_general(ID_ENG_F2, dummy, 1); break;
            case 14: pack_general(ID_SHIFTER, dummy, 3); break;
        }

        // Sleep between 2ms and 12ms
        vTaskDelay(pdMS_TO_TICKS((esp_random() % 10) + 2));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    wifi_init();
    espnow_init();
    general_mutex = xSemaphoreCreateMutex();
    #if USE_REAL_DATA
        ESP_LOGI(TAG, "USING REAL DATA");
        can_init(process_can_message);
        xTaskCreate(clear_general_task, "clear general task", 4096, NULL, 5, NULL);
    #else
        ESP_LOGI(TAG, "USING FAKE DATA");
        xTaskCreate(fake_can_generator_task, "fake_can", 4096, NULL, 5, NULL);
        xTaskCreate(clear_general_task, "clear general task", 4096, NULL, 5, NULL);
    #endif

}
