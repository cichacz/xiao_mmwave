#include "driver/uart.h"
#include "driver/gpio.h"

#define BUFFER_SIZE 256

// UART port and configuration.
#define UART_PORT_NUM UART_NUM_1
#define UART_BAUD_RATE 115200
#define UART_TX_PIN (GPIO_NUM_21)
#define UART_RX_PIN (GPIO_NUM_2)

/* Enumerations for target and command statuses */
typedef enum target_status_e
{
    TARGET_NO_TARGET = 0,
    TARGET_MOVING_TARGET = 0x01,
    TARGET_STATIONARY_TARGET = 0x02,
    TARGET_BOTH_TARGETS = 0x03,
    TARGET_ERROR_FRAME = 0x04
} target_status_t;

typedef struct radar_status_s
{
    int radar_mode;
    target_status_t target_status;
    uint16_t moving_target_distance;
    uint8_t moving_target_energy;
    uint16_t stationary_target_distance;
    uint8_t stationary_target_energy;
    uint16_t detection_distance;
    uint8_t radar_move_power[9];
    uint8_t radar_static_power[9];
} radar_status_t;

typedef void (*StatusFunction_t)(radar_status_t *status);

#ifdef __cplusplus
extern "C"
{
#endif

    /* Initialization functions */
    esp_err_t xiao_mmwave_init(StatusFunction_t cb);

    esp_err_t xiao_mmwave_set_detection_distance(uint8_t gate, uint8_t times);

    uint8_t xiao_mmwave_get_detection_resolution(uint16_t distance);

#ifdef __cplusplus
}
#endif
