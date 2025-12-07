#include "can.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"


static const char *TAG = "CAN";

#define BITRATE 1000000

//Switched these, spotted a possible issue with schematic naming
#define TX GPIO_NUM_4
#define RX GPIO_NUM_18

twai_node_handle_t hfdcan = NULL;

// Queue to store received messages
static QueueHandle_t rx_queue;

// Semaphore for synchronization
static SemaphoreHandle_t rx_sem;

static can_message_callback_t process = NULL;

uint32_t can_msg_count = 0;

static bool can_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    can_msg_count++;
    uint8_t recv_buff[64];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        // Create a safe copy with embedded data
        typedef struct {
            twai_frame_header_t header;
            uint8_t data[64];  // Actual data copy, not pointer
        } safe_can_frame_t;
        
        safe_can_frame_t safe_frame;
        safe_frame.header = rx_frame.header;
        memcpy(safe_frame.data, rx_frame.buffer, rx_frame.header.dlc);
        
        xQueueSendFromISR(rx_queue, &safe_frame, &xHigherPriorityTaskWoken);
        
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
    return false;
}

static void can_receive_task(void *pvParameters) {

    
    safe_can_frame_t rx_frame;
    
    while (1) {
        if (xQueueReceive(rx_queue, &rx_frame, pdMS_TO_TICKS(100)) == pdPASS) {
            // Convert to the format your callback expects
            twai_frame_t processed_frame = {
                .header = rx_frame.header,
                .buffer = rx_frame.data,  // Now points to valid memory
                .buffer_len = sizeof(rx_frame.data)
            };
            
            if (process != NULL) {
                process(&processed_frame);  // ✅ SAFE
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void can_init(can_message_callback_t callback_function){
    process = callback_function;
    // Create queue for received messages
    rx_queue = xQueueCreate(10, sizeof(safe_can_frame_t));
    
    // Create semaphore for RX notifications
    rx_sem = xSemaphoreCreateBinary();

        // Configure TWAI node with ISR callback
    const twai_onchip_node_config_t can_config = {
        .io_cfg = {
            .tx = TX,
            .rx = RX,
            .quanta_clk_out = -1,
            .bus_off_indicator = -1
        },
        .bit_timing = {
            .bitrate = BITRATE
        },
        .tx_queue_depth = 64,
    };
    const twai_event_callbacks_t callbacks = {
        .on_rx_done = can_rx_cb
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&can_config, &hfdcan));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(hfdcan, &callbacks, NULL));
    ESP_ERROR_CHECK(twai_node_enable(hfdcan));

    BaseType_t result = xTaskCreate(can_receive_task, "can_rx", 4096, NULL, 5, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create can_receive_task");
        return;
    }

}