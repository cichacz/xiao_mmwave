# NOTE: This kind of mocking currently works on Linux targets only.
#       On Espressif chips, too many dependencies are missing at the moment.
message(STATUS "building DRIVER MOCKS (only UART driver)")

set(include_dirs
    "${IDF_PATH}/components/esp_driver_gpio/include/driver"
    "${IDF_PATH}/components/esp_driver_gpio/include"
    "${IDF_PATH}/components/esp_driver_uart/include/driver"
    "${IDF_PATH}/components/esp_driver_uart/include")

# Note: "esp_hal_uart" is only required for corresponding header files and their definitions
# here, they don't provide functionality when built for running on the host.
idf_component_mock(INCLUDE_DIRS ${include_dirs}
    REQUIRES esp_hal_uart
    MOCK_HEADER_FILES
    ${IDF_PATH}/components/esp_driver_uart/include/driver/uart.h)
