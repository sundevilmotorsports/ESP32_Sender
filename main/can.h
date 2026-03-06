#ifndef CAN_H
#define CAN_H

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"

extern twai_node_handle_t hfdcan;
extern uint32_t can_msg_count;

typedef struct {
        twai_frame_header_t header;
        uint8_t data[64];
} safe_can_frame_t;

// Callback function type for message processing
typedef void (*can_message_callback_t)(twai_frame_t *message);


void can_init(can_message_callback_t callback_function);
esp_err_t can_send_frame(uint32_t id, const uint8_t *data, uint8_t len);
#endif