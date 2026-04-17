#include <stdio.h>
#include <string.h>
#include "can.h"
#include "espnow.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_crc.h"
#include "freertos/semphr.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#define USE_REAL_DATA true

#define SEND_INTERVAL_MS 20   // 50 Hz

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
#define ID_ENG_F0       0x0C // rpm, ect, oil_temp, oil_press, neutral
#define ID_ENG_F1       0x0D // lambda, tps, gear, driven_wspd, oil_press
#define ID_ENG_F2       0x0E // aps, fuel_press
#define ID_SHIFTER      0x0F

// Fixed byte offsets within the 66-byte telemetry state buffer.
// Receiver must mirror these exactly.
#define OFFSET_DRS        0   //  1 byte
#define OFFSET_IMU_GYRO   1   //  6 bytes
#define OFFSET_IMU_ACCEL  7   //  6 bytes
#define OFFSET_WHEEL_FL  13   //  6 bytes
#define OFFSET_WHEEL_FR  19   //  6 bytes
#define OFFSET_WHEEL_RR  25   //  6 bytes
#define OFFSET_WHEEL_RL  31   //  6 bytes
#define OFFSET_SG_FL     37   //  2 bytes
#define OFFSET_SG_FR     39   //  2 bytes
#define OFFSET_SG_RR     41   //  2 bytes
#define OFFSET_SG_RL     43   //  2 bytes
#define OFFSET_ENG_F0    45   //  7 bytes
#define OFFSET_ENG_F1    52   //  7 bytes
#define OFFSET_ENG_F2    59   //  4 bytes
#define OFFSET_SHIFTER   63   //  3 bytes
#define TELEM_PACKET_SIZE 66

#define USER_LED GPIO_NUM_4

#define AMP_EN_PIN  GPIO_NUM_1
#define AMP_EN      1

static const char *TAG = "SENDER_MAIN";
static uint16_t s_seq_num = 0;
static int64_t can_msg_time = 0;

static uint8_t telemetry_state[TELEM_PACKET_SIZE];
static SemaphoreHandle_t state_mutex;

typedef struct { uint8_t id; uint8_t offset; uint8_t len; } sensor_slot_t;

static const sensor_slot_t sensor_slots[] = {
    { ID_DRS,       OFFSET_DRS,       1 },
    { ID_IMU_GYRO,  OFFSET_IMU_GYRO,  6 },
    { ID_IMU_ACCEL, OFFSET_IMU_ACCEL, 6 },
    { ID_WHEEL_FL,  OFFSET_WHEEL_FL,  6 },
    { ID_WHEEL_FR,  OFFSET_WHEEL_FR,  6 },
    { ID_WHEEL_RR,  OFFSET_WHEEL_RR,  6 },
    { ID_WHEEL_RL,  OFFSET_WHEEL_RL,  6 },
    { ID_SG_FL,     OFFSET_SG_FL,     2 },
    { ID_SG_FR,     OFFSET_SG_FR,     2 },
    { ID_SG_RR,     OFFSET_SG_RR,     2 },
    { ID_SG_RL,     OFFSET_SG_RL,     2 },
    { ID_ENG_F0,    OFFSET_ENG_F0,    7 },
    { ID_ENG_F1,    OFFSET_ENG_F1,    7 },
    { ID_ENG_F2,    OFFSET_ENG_F2,    4 },
    { ID_SHIFTER,   OFFSET_SHIFTER,   3 },
};
static const int NUM_SLOTS = sizeof(sensor_slots) / sizeof(sensor_slots[0]);

static void update_sensor(uint8_t sensor_id, const uint8_t *data, size_t len) {
    for (int i = 0; i < NUM_SLOTS; i++) {
        if (sensor_slots[i].id == sensor_id) {
            size_t copy_len = len < sensor_slots[i].len ? len : sensor_slots[i].len;
            if (xSemaphoreTake(state_mutex, portMAX_DELAY)) {
                memcpy(&telemetry_state[sensor_slots[i].offset], data, copy_len);
                xSemaphoreGive(state_mutex);
            }
            return;
        }
    }
    // Unknown sensor ID — no fixed slot, drop it
}

static void send_telemetry(void) {
    espnow_data_t pkt;
    pkt.type    = (uint8_t)5; // ESPNOW_TELEMETRY
    pkt.seq_num = s_seq_num++;
    pkt.crc     = 0;
    pkt.len     = TELEM_PACKET_SIZE;
    if (xSemaphoreTake(state_mutex, portMAX_DELAY)) {
        memcpy(pkt.data, telemetry_state, TELEM_PACKET_SIZE);
        xSemaphoreGive(state_mutex);
    }
    size_t total_size = 6 + TELEM_PACKET_SIZE;
    pkt.crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)&pkt, total_size);
    esp_now_send(s_broadcast_mac, (uint8_t *)&pkt, total_size);
}

static void process_can_message(twai_frame_t *message) {
    uint8_t data[8];
    can_msg_time = esp_timer_get_time() / 1000;
    memcpy(data, message->buffer, message->header.dlc);
    switch (message->header.id) {
        case 0x35F:
            update_sensor(ID_DRS, data, 1);
            break;
        case 0x360:
            update_sensor(ID_IMU_GYRO, data, 6);
            break;
        case 0x361:
            update_sensor(ID_IMU_ACCEL, data, 6);
            break;
        case 0x363:
            update_sensor(ID_WHEEL_FL, data, 6);
            break;
        case 0x364:
            update_sensor(ID_WHEEL_FR, data, 6);
            break;
        case 0x365:
            update_sensor(ID_WHEEL_RR, data, 6);
            break;
        case 0x366:
            update_sensor(ID_WHEEL_RL, data, 6);
            break;
        case 0x4e2:
            update_sensor(ID_SG_FL, data, 2);
            break;
        case 0x4e3:
            update_sensor(ID_SG_FR, data, 2);
            break;
        case 0x4e4:
            update_sensor(ID_SG_RR, data, 2);
            break;
        case 0x4e5:
            update_sensor(ID_SG_RL, data, 2);
            break;
        case 0x3e8:
            // Engine CAN stream — byte 0 is sub-frame ID
            switch (data[0]) {
                case 0x0: {
                    // engine_speed(2), ect(1), oil_temp(1), oil_press(2), neutral(1) = 7 bytes
                    uint16_t engine_speed = (uint16_t)((data[1] << 8) | data[2]);
                    uint8_t  ect          = data[3];
                    uint8_t  oil_temp     = data[4];
                    uint16_t oil_press    = (uint16_t)((data[5] << 8) | data[6]);
                    uint8_t  neutral      = data[7];
                    uint8_t pkt[7] = {
                        (uint8_t)(engine_speed >> 8), (uint8_t)(engine_speed & 0xFF),
                        ect, oil_temp,
                        (uint8_t)(oil_press >> 8), (uint8_t)(oil_press & 0xFF),
                        neutral
                    };
                    update_sensor(ID_ENG_F0, pkt, 7);
                    break;
                }
                case 0x1: {
                    // lambda(1), tps(1), gear(1), driven_wspd(2), oil_press(2) = 7 bytes
                    uint8_t  lambda      = data[1];
                    uint8_t  tps         = data[2];
                    uint8_t  gear        = data[3];
                    uint16_t driven_wspd = (uint16_t)((data[4] << 8) | data[5]);
                    uint16_t oil_press   = (uint16_t)((data[6] << 8) | data[7]);
                    uint8_t pkt[7] = {
                        lambda, tps, gear,
                        (uint8_t)(driven_wspd >> 8), (uint8_t)(driven_wspd & 0xFF),
                        (uint8_t)(oil_press >> 8),   (uint8_t)(oil_press & 0xFF)
                    };
                    update_sensor(ID_ENG_F1, pkt, 7);
                    break;
                }
                case 0x2: {
                    // aps(2), fuel_press(2) = 4 bytes
                    uint16_t aps        = (uint16_t)((data[1] << 8) | data[2]);
                    uint16_t fuel_press = (uint16_t)((data[3] << 8) | data[4]);
                    uint8_t pkt[4] = {
                        (uint8_t)(aps >> 8),        (uint8_t)(aps & 0xFF),
                        (uint8_t)(fuel_press >> 8), (uint8_t)(fuel_press & 0xFF)
                    };
                    update_sensor(ID_ENG_F2, pkt, 4);
                    break;
                }
            }
            break;
        case 0x40:
            update_sensor(ID_SHIFTER, data, 3);
            break;
        default:
            // Unknown CAN IDs have no fixed slot — drop
            break;
    }
}

void send_telemetry_task(void *pvParameter) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
        send_telemetry();
    }
}

void fake_can_generator_task(void *pvParameter) {
    uint8_t dummy[8];
    while (1) {
        *(uint32_t*)dummy       = esp_random();
        *(uint32_t*)(dummy + 4) = esp_random();
        int msg_type = esp_random() % 15;
        switch (msg_type) {
            case 0:  update_sensor(ID_DRS,       dummy, 1); break;
            case 1:  update_sensor(ID_IMU_GYRO,  dummy, 6); break;
            case 2:  update_sensor(ID_IMU_ACCEL, dummy, 6); break;
            case 3:  update_sensor(ID_WHEEL_FL,  dummy, 6); break;
            case 4:  update_sensor(ID_WHEEL_FR,  dummy, 6); break;
            case 5:  update_sensor(ID_WHEEL_RR,  dummy, 6); break;
            case 6:  update_sensor(ID_WHEEL_RL,  dummy, 6); break;
            case 7:  update_sensor(ID_SG_FL,     dummy, 2); break;
            case 8:  update_sensor(ID_SG_FR,     dummy, 2); break;
            case 9:  update_sensor(ID_SG_RR,     dummy, 2); break;
            case 10: update_sensor(ID_SG_RL,     dummy, 2); break;
            case 11: update_sensor(ID_ENG_F0,    dummy, 7); break;
            case 12: update_sensor(ID_ENG_F1,    dummy, 7); break;
            case 13: update_sensor(ID_ENG_F2,    dummy, 4); break;
            case 14: update_sensor(ID_SHIFTER,   dummy, 3); break;
        }
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

    gpio_reset_pin(AMP_EN_PIN);
    gpio_set_direction(AMP_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(AMP_EN_PIN, AMP_EN);

    gpio_reset_pin(USER_LED);
    gpio_set_direction(USER_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(USER_LED, 0);

    memset(telemetry_state, 0, sizeof(telemetry_state));
    state_mutex = xSemaphoreCreateMutex();

#if USE_REAL_DATA
    ESP_LOGI(TAG, "USING REAL DATA");
    can_init(process_can_message);
#else
    ESP_LOGI(TAG, "USING FAKE DATA");
    xTaskCreate(fake_can_generator_task, "fake_can", 4096, NULL, 5, NULL);
#endif
    xTaskCreate(send_telemetry_task, "send_telemetry", 4096, NULL, 5, NULL);
    for(;;){
        if(esp_timer_get_time() / 1000 - can_msg_time > 1){
            gpio_set_level(USER_LED, 0);
            // ESP_LOGW(TAG,"NO CAN IN LAST 100 MS");
        } else {
            gpio_set_level(USER_LED, 1);
            // ESP_LOGI(TAG,"CAN RECIVED");
        }
    }
}
