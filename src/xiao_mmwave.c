#include <string.h>
#include <inttypes.h>
#include "xiao_mmwave.h"

#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_check.h"

typedef struct current_command_s
{
    TaskHandle_t task_handle;
    uint16_t word;
    uint8_t response[BUFFER_SIZE];
} current_command_t;

static const char *TAG = "mmWave";
static const uint8_t data_prefix[] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t config_prefix[] = {0xFD, 0xFC, 0xFB, 0xFA};

static QueueHandle_t uart_queue;
static StatusFunction_t status_update_function;
static current_command_t current_command;

static void build_command_frame(uint16_t command_word, uint8_t **frame_data, size_t *frame_data_size)
{
    // 8 bytes before and 4 after frame_data
    size_t new_size = *frame_data_size + 12;
    size_t intra_frame_size = *frame_data_size + 2;

    // this raises kernel panic for some reason
    // uint8_t *frame = realloc(*frame_data, new_size);

    uint8_t *new_frame = malloc(new_size);
    // Shift elements to the right to make space for header
    memcpy(new_frame + 8, *frame_data, *frame_data_size);
    // this raises kernel panic for some reason
    // free(*frame_data);
    *frame_data = new_frame;
    *frame_data_size = new_size;

    memcpy(*frame_data, config_prefix, 4);

    (*frame_data)[4] = intra_frame_size & 0xFF;
    (*frame_data)[5] = (intra_frame_size >> 8) & 0xFF;

    (*frame_data)[6] = command_word & 0xFF;
    (*frame_data)[7] = (command_word >> 8) & 0xFF;

    // Initialize the last part of the frame
    (*frame_data)[*frame_data_size - 4] = 0x04;
    (*frame_data)[*frame_data_size - 3] = 0x03;
    (*frame_data)[*frame_data_size - 2] = 0x02;
    (*frame_data)[*frame_data_size - 1] = 0x01;
}

static uint16_t get_command_word(uint8_t *data)
{
    // as per documentation ACK replies with `Send command word | 0x0100`
    return (data[2] | (data[3] << 8)) & ~0x0100;
}

static uint8_t *send_command(const uint16_t command_word, uint8_t *data, size_t size)
{
    // Save this task's handle for notifications.
    current_command = (current_command_t){.task_handle = xTaskGetCurrentTaskHandle(), .word = command_word};

    build_command_frame(command_word, &data, &size);

    int bytes_written = uart_write_bytes(UART_PORT_NUM, data, size);
    if (bytes_written < 0)
    {
        ESP_LOGE(TAG, "Failed to write command: %d", bytes_written);
        // Clean up before exiting
        memset(&current_command, 0, sizeof(current_command));

        return NULL;
    }
    ESP_LOGI(TAG, "Command 0x%04X sent, waiting for response...", command_word);

    // Clear any previous notification state.
    xTaskNotifyStateClear(NULL);

    uint32_t notification_value = 0;
    BaseType_t ret = xTaskNotifyWait(0, 0, &notification_value, pdMS_TO_TICKS(1000));

    size_t data_size = (current_command.response[0] | (current_command.response[1] << 8)) - 2;
    uint16_t response_word = get_command_word(current_command.response);

    // we already checked this in uart event callback but better safe than sorry
    if (ret == pdTRUE && response_word == command_word)
    {
        uint8_t *response = malloc(data_size);
        // get rid of size and command word
        memcpy(response, current_command.response + 4, data_size);

        ESP_LOGI(TAG, "Received response (%" PRIuPTR " bytes):", data_size);
        ESP_LOG_BUFFER_HEXDUMP(TAG, response, data_size, ESP_LOG_INFO);

        memset(&current_command, 0, sizeof(current_command));

        return response;
    }

    ESP_LOGW(TAG, "Timed out waiting for command response");
    ESP_LOGW(TAG, "Current response word: 0x%04X (expected 0x%04X)", response_word, command_word);

    memset(&current_command, 0, sizeof(current_command));

    return NULL;
}

static void uart_event_task(void *arg)
{
    uart_event_t event;
    uint8_t data[BUFFER_SIZE];
    uint8_t circuit_break = 1;

    while (circuit_break)
    {
        // Wait for UART event
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA: // New data received
                if (event.size > 0)
                {
                    uart_read_bytes(UART_PORT_NUM, data, event.size, portMAX_DELAY);

                    if (memcmp(data, &data_prefix, 4) != 0 && memcmp(data, &config_prefix, 4) != 0)
                    {
                        ESP_LOGE(TAG, "Received unknown response from sensor");
                        break;
                    }

                    // get rid of frame details
                    size_t data_size = (data[4] | (data[5] << 8)) + 2;
                    memmove(data, data + 4, data_size);

                    // Notify the command task if it's waiting.
                    if (current_command.task_handle != NULL)
                    {
                        // as per documentation ACK replies with `Send command word | 0x0100`
                        uint16_t response_word = get_command_word(data);
                        if (response_word == current_command.word)
                        {
                            memcpy(current_command.response, data, data_size);
                            xTaskNotifyGive(current_command.task_handle);
                            break;
                        }
                    }

                    radar_status_t status_data;
                    memset(&status_data, 0, sizeof(radar_status_t));

                    // we don't need data size anymore
                    memmove(data, data + 2, data_size - 2);

                    status_data.radar_mode = data[0];
                    // data[1] is a 0xAA head
                    status_data.target_status = (target_status_t)data[2];
                    status_data.moving_target_distance = data[3] | (data[4] << 8);
                    status_data.moving_target_energy = data[5];
                    status_data.stationary_target_distance = data[6] | (data[7] << 8);
                    status_data.stationary_target_energy = data[8];
                    status_data.detection_distance = data[9] | (data[10] << 8);

                    if (status_data.radar_mode == 1)
                    {
                        /* Engineering mode */
                        // data[11] is a Maximum movement distance door N
                        // data[12] is a Maximum resting distance door N
                        for (int i = 0; i < 9; i++)
                        {
                            status_data.radar_move_power[i] = data[i + 13];
                            status_data.radar_static_power[i] = data[i + 22];
                        }
                    }

                    status_update_function(&status_data);
                }
                break;

            case UART_FIFO_OVF: // FIFO overflow (should not happen in normal conditions)
                ESP_LOGW(TAG, "UART FIFO Overflow!");
                uart_flush_input(UART_PORT_NUM);
                xQueueReset(uart_queue);
                break;

            case UART_BREAK:
                ESP_LOGW(TAG, "Stopping UART listener");
                uart_flush_input(UART_PORT_NUM);
                xQueueReset(uart_queue);
                circuit_break = 0;
                break;

            default:
                ESP_LOGD(TAG, "Unhandled UART event: %d", event.type);
                break;
            }
        }
    }
}

static esp_err_t enable_config_mode()
{
    uint8_t command_word = 0xFF;
    uint8_t command_value[] = {0x01, 0x00};

    uint8_t *response = send_command(command_word, command_value, 2);

    return response != NULL && response[0] == 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t disable_config_mode()
{
    uint8_t command_word = 0xFE;
    uint8_t command_value[] = {};

    uint8_t *response = send_command(command_word, command_value, 0);

    return response != NULL && response[0] == 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t set_detection_resolution(uint8_t resolution)
{
    if (resolution > 1)
    {
        return ESP_FAIL;
    }

    uint16_t command_word = 0xAA;
    uint8_t command_value[] = {resolution, 0x00};

    uint8_t *response = send_command(command_word, command_value, 2);

    return response != NULL && response[0] == 0 ? ESP_OK : ESP_FAIL;
}

/* Initialization functions */
esp_err_t xiao_mmwave_init(StatusFunction_t cb)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUFFER_SIZE * 2, BUFFER_SIZE, 10, &uart_queue, 0));

    ESP_LOGI(TAG, "UART initialized successfuly");

    status_update_function = cb;

    return (xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL) == pdTRUE) ? ESP_OK : ESP_FAIL;
}

esp_err_t xiao_mmwave_set_detection_distance(uint8_t distance, uint8_t times)
{
    ESP_RETURN_ON_ERROR(enable_config_mode(), TAG, "Failed to enter config mode");

    uint16_t command_word = 0x60;
    uint8_t command_value[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    command_value[2] = distance;
    command_value[8] = distance;

    command_value[14] = times & 0xFF;
    command_value[15] = (times >> 8) & 0xFF;
    command_value[16] = (times >> 16) & 0xFF;
    command_value[17] = (times >> 24) & 0xFF;

    uint8_t *response = send_command(command_word, command_value, 18);

    ESP_RETURN_ON_ERROR(disable_config_mode(), TAG, "Failed to exit config mode");

    return response != NULL && response[0] == 0 ? ESP_OK : ESP_FAIL;
}

uint8_t xiao_mmwave_get_detection_resolution(uint16_t distance)
{
    ESP_RETURN_ON_ERROR(enable_config_mode(), TAG, "Failed to enter config mode");

    uint16_t command_word = 0xAB;
    uint8_t command_value[] = {};

    uint8_t *response = send_command(command_word, command_value, 0);

    if (response == NULL || response[0] == 1)
    {
        return ESP_FAIL;
    }

    // FIXME make it prettier
    uint8_t resolution = response[2] == 0 ? 75 : 20;

    if (distance > 160 && resolution == 20)
    {
        ESP_RETURN_ON_ERROR(set_detection_resolution(0), TAG, "Failed to set detection resolution");
        resolution = 75;
    }
    else if (distance <= 160 && resolution == 75)
    {
        ESP_RETURN_ON_ERROR(set_detection_resolution(1), TAG, "Failed to set detection resolution");
        resolution = 20;
    }

    ESP_RETURN_ON_ERROR(disable_config_mode(), TAG, "Failed to exit config mode");

    return resolution;
}
