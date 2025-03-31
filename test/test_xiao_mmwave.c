#include <string.h>
#include "unity.h"
#include "Mockuart.h"
#include "Mockqueue.h"
#include "Mocktask.h"
#include "Mockidf_additions.h"

#include "xiao_mmwave.h"

QueueHandle_t mock_queue;
TaskHandle_t task_handle;
uart_event_t event;

void sensor_init(void);
void sensor_driver_value_update(radar_status_t *status);
void assert_uart_event(uint8_t *frame_data, size_t frame_data_size);
BaseType_t create_uart_queue_task(TaskFunction_t pxTaskCode, const char *const pcName, const uint32_t ulStackDepth, void *const pvParameters, UBaseType_t uxPriority, TaskHandle_t *const pxCreatedTask, const BaseType_t xCoreID, int cmock_num_calls);
BaseType_t queue_event_gen(QueueHandle_t xQueue, void *const pvBuffer, TickType_t xTicksToWait, int cmock_num_calls);
BaseType_t prepare_for_command(UBaseType_t uxIndexToWaitOn, uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait, int cmock_num_calls);

int detection_distance_event_payload(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait, int cmock_num_calls);
int detection_resolution_get_set_event_payload(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait, int cmock_num_calls);
int detection_resolution_get_event_payload(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait, int cmock_num_calls);
int radar_status_payload(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait, int cmock_num_calls);

static const char *status_to_string(target_status_t status)
{
    switch (status)
    {
    case TARGET_NO_TARGET:
        return "NO_TARGET";
    case TARGET_BOTH_TARGETS:
        return "BOTH_TARGETS";
    case TARGET_MOVING_TARGET:
        return "MOVING_TARGET";
    case TARGET_STATIONARY_TARGET:
        return "STATIC_TARGET";
    default:
        return "ERROR_FRAME";
    }
}

void setUp(void)
{
    // we use it to track number of events in each test
    event.size = 0;
    xTaskCreatePinnedToCore_Stub(create_uart_queue_task);
    xQueueReceive_Stub(queue_event_gen);
}

TEST_CASE("init", "[xiao_mmwave]")
{
    event.type = UART_BREAK;
    sensor_init();
}

TEST_CASE("radar status", "[xiao_mmwave]")
{
    uart_read_bytes_Stub(radar_status_payload);

    event.type = UART_DATA;
    event.size++;
    sensor_init();
}

TEST_CASE("set_detection_distance", "[xiao_mmwave]")
{
    // values from the documentation
    uint8_t gate = 8;
    uint8_t times = 5;
    uint8_t enable_config_frame[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    uint8_t disable_config_frame[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    uint8_t detection_distance_frame[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x14, 0x00, 0x60, 0x00, 0x00, 0x00, gate, 0x00, 0x00, 0x00, 0x01, 0x00, gate, 0x00, 0x00, 0x00, 0x02, 0x00, times, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};

    xTaskGenericNotifyWait_AddCallback(prepare_for_command);
    uart_read_bytes_Stub(detection_distance_event_payload);

    assert_uart_event(enable_config_frame, sizeof(enable_config_frame));
    assert_uart_event(detection_distance_frame, sizeof(detection_distance_frame));
    assert_uart_event(disable_config_frame, sizeof(disable_config_frame));

    TEST_ASSERT_EQUAL(ESP_OK, xiao_mmwave_set_detection_distance(gate, times));
}

TEST_CASE("get_detection_resolution long", "[xiao_mmwave]")
{
    // values from the documentation
    uint8_t enable_config_frame[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    uint8_t disable_config_frame[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    uint8_t get_detection_resolution_frame[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xAB, 0x00, 0x04, 0x03, 0x02, 0x01};
    uint8_t set_detection_resolution_frame[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xAA, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};

    xTaskGenericNotifyWait_AddCallback(prepare_for_command);
    uart_read_bytes_Stub(detection_resolution_get_set_event_payload);

    assert_uart_event(enable_config_frame, sizeof(enable_config_frame));
    assert_uart_event(get_detection_resolution_frame, sizeof(get_detection_resolution_frame));
    assert_uart_event(set_detection_resolution_frame, sizeof(set_detection_resolution_frame));
    assert_uart_event(disable_config_frame, sizeof(disable_config_frame));

    TEST_ASSERT_EQUAL_UINT8(75, xiao_mmwave_get_detection_resolution(300));
}

TEST_CASE("get_detection_resolution short", "[xiao_mmwave]")
{
    // values from the documentation
    uint8_t enable_config_frame[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    uint8_t disable_config_frame[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    uint8_t get_detection_resolution_frame[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xAB, 0x00, 0x04, 0x03, 0x02, 0x01};

    xTaskGenericNotifyWait_AddCallback(prepare_for_command);
    uart_read_bytes_Stub(detection_resolution_get_event_payload);

    assert_uart_event(enable_config_frame, sizeof(enable_config_frame));
    assert_uart_event(get_detection_resolution_frame, sizeof(get_detection_resolution_frame));
    // no set here because we mocked device default value to be 0x01
    assert_uart_event(disable_config_frame, sizeof(disable_config_frame));

    TEST_ASSERT_EQUAL_UINT8(20, xiao_mmwave_get_detection_resolution(100));
}

void sensor_init(void)
{
    uart_config_t expected_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config_ExpectAndReturn(UART_NUM_1, &expected_config, ESP_OK);
    uart_set_pin_ExpectAndReturn(UART_NUM_1, GPIO_NUM_21, GPIO_NUM_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, ESP_OK);
    uart_driver_install_ExpectAndReturn(UART_NUM_1, 512, 256, 10, &mock_queue, 0, ESP_OK);
    xQueueReceive_ExpectAnyArgsAndReturn(pdTRUE);

    // when test ends, queue should terminate
    uart_flush_input_ExpectAndReturn(UART_NUM_1, ESP_OK);
    xQueueGenericReset_ExpectAndReturn(mock_queue, pdFALSE, pdTRUE);

    TEST_ASSERT_EQUAL(ESP_OK, xiao_mmwave_init(sensor_driver_value_update));
}

void sensor_driver_value_update(radar_status_t *status)
{
    TEST_ASSERT_EQUAL_UINT8(0x02, status->target_status);
    TEST_ASSERT_EQUAL_UINT8(0x10, status->detection_distance);
    TEST_ASSERT_EQUAL_UINT8(0x51, status->moving_target_distance);
    TEST_ASSERT_EQUAL_UINT8(0x06, status->moving_target_energy);
    TEST_ASSERT_EQUAL_UINT8(0x36, status->stationary_target_distance);
    TEST_ASSERT_EQUAL_UINT8(0x3B, status->stationary_target_energy);
}

void assert_uart_event(uint8_t *frame_data, size_t frame_data_size)
{
    uint32_t notification_value = 0;
    // anything except NULL
    task_handle = malloc(1);

    xTaskGetCurrentTaskHandle_ExpectAndReturn(task_handle);
    uart_write_bytes_ExpectAndReturn(UART_NUM_1, frame_data, frame_data_size, frame_data_size);
    xTaskGenericNotifyStateClear_ExpectAnyArgsAndReturn(pdTRUE);
    xTaskGenericNotifyWait_ExpectAndReturn(tskDEFAULT_INDEX_TO_NOTIFY, 0, 0, &notification_value, pdMS_TO_TICKS(1000), pdTRUE);
    xTaskGenericNotify_ExpectAnyArgsAndReturn(pdTRUE);
}

BaseType_t create_uart_queue_task(TaskFunction_t pxTaskCode, const char *const pcName, const uint32_t ulStackDepth, void *const pvParameters, UBaseType_t uxPriority, TaskHandle_t *const pxCreatedTask, const BaseType_t xCoreID, int cmock_num_calls)
{
    pxTaskCode(NULL);

    return pdTRUE;
}

BaseType_t queue_event_gen(QueueHandle_t xQueue, void *const pvBuffer, TickType_t xTicksToWait, int cmock_num_calls)
{
    memcpy(pvBuffer, &event, sizeof(uart_event_t));
    return pdTRUE;
}

BaseType_t prepare_for_command(UBaseType_t uxIndexToWaitOn, uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait, int cmock_num_calls)
{
    event.type = UART_DATA;
    event.size++;
    sensor_init();

    return pdTRUE;
}

int config_event_payload(void *buf)
{
    uint8_t data[] = {0xFD, 0xFC, 0xFB, 0xFA,
                      0x08, 0x00,
                      0xFF, 0x01, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00,
                      0x04, 0x03, 0x02, 0x01};
    memcpy(buf, &data, sizeof(data));

    return sizeof(data);
}

int config_exit_event_payload(void *buf)
{
    uint8_t data[] = {0xFD, 0xFC, 0xFB, 0xFA,
                      0x04, 0x00,
                      0xFE, 0x01, 0x00, 0x00,
                      0x04, 0x03, 0x02, 0x01};
    memcpy(buf, &data, sizeof(data));

    return sizeof(data);
}

int detection_distance_event_payload(uart_port_t uart_num, void *buf, uint32_t event_num, TickType_t ticks_to_wait, int cmock_num_calls)
{
    // after each response we have to break loop and start it again before next request
    event.type = UART_BREAK;

    if (event_num == 1)
    {
        return config_event_payload(buf);
    }

    if (event_num == 3)
    {
        return config_exit_event_payload(buf);
    }

    uint8_t data[] = {0xFD, 0xFC, 0xFB, 0xFA,
                      0x04, 0x00,
                      0x60, 0x01, 0x00, 0x00,
                      0x04, 0x03, 0x02, 0x01};
    memcpy(buf, &data, sizeof(data));

    return sizeof(data);
}

int detection_resolution_get_event_payload(uart_port_t uart_num, void *buf, uint32_t event_num, TickType_t ticks_to_wait, int cmock_num_calls)
{
    // after each response we have to break loop and start it again before next request
    event.type = UART_BREAK;

    if (event_num == 1)
    {
        return config_event_payload(buf);
    }

    if (event_num == 3)
    {
        return config_exit_event_payload(buf);
    }

    uint8_t data[] = {0xFD, 0xFC, 0xFB, 0xFA,
                      0x06, 0x00,
                      0xAB, 0x01, 0x00, 0x00, 0x01, 0x00,
                      0x04, 0x03, 0x02, 0x01};
    memcpy(buf, &data, sizeof(data));

    return sizeof(data);
}

int detection_resolution_get_set_event_payload(uart_port_t uart_num, void *buf, uint32_t event_num, TickType_t ticks_to_wait, int cmock_num_calls)
{
    // after each response we have to break loop and start it again before next request
    event.type = UART_BREAK;

    if (event_num == 1)
    {
        return config_event_payload(buf);
    }

    if (event_num == 2)
    {
        // first we're getting current value
        uint8_t data[] = {0xFD, 0xFC, 0xFB, 0xFA,
                          0x06, 0x00,
                          0xAB, 0x01, 0x00, 0x00, 0x01, 0x00,
                          0x04, 0x03, 0x02, 0x01};
        memcpy(buf, &data, sizeof(data));

        return sizeof(data);
    }

    if (event_num == 4)
    {
        return config_exit_event_payload(buf);
    }

    uint8_t data[] = {0xFD, 0xFC, 0xFB, 0xFA,
                      0x04, 0x00,
                      0xAA, 0x01, 0x00, 0x00,
                      0x04, 0x03, 0x02, 0x01};
    memcpy(buf, &data, sizeof(data));

    return sizeof(data);
}

int radar_status_payload(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait, int cmock_num_calls)
{
    // after each response we have to break loop and start it again before next request
    event.type = UART_BREAK;

    uint8_t data[] = {0xF4, 0xF3, 0xF2, 0xF1,
                      0x0D, 0x00,
                      0x02, 0xAA,
                      0x02, 0x51, 0x00, 0x06, 0x36, 0x00, 0x3B, 0x10, 0x00,
                      0x55, 0x00,
                      0xF8, 0xF7, 0xF6, 0xF5};
    memcpy(buf, &data, sizeof(data));

    return sizeof(data);
}