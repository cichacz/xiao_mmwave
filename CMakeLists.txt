set(IDF_DRIVER_REQUIRES)

if("${IDF_VERSION_MAJOR}" VERSION_GREATER_EQUAL "6")

  list(APPEND IDF_DRIVER_REQUIRES esp_driver_gpio esp_driver_uart)

else()

  list(APPEND IDF_DRIVER_REQUIRES driver)

endif()

idf_component_register(SRCS "src/xiao_mmwave.c" INCLUDE_DIRS "include"
                       PRIV_REQUIRES ${IDF_DRIVER_REQUIRES})
