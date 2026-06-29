idf_component_register(
    SRCS "src/xiao_mmwave.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart esp_driver_gpio)
