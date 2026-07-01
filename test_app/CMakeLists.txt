# This is the project CMakeLists.txt file for the test subproject
cmake_minimum_required(VERSION 3.16)

# Include the components directory of the main application:
#
set(EXTRA_COMPONENT_DIRS "../")

list(APPEND EXTRA_COMPONENT_DIRS "./mocks/soc/")
list(APPEND EXTRA_COMPONENT_DIRS "./mocks/driver/")
list(APPEND EXTRA_COMPONENT_DIRS "$ENV{IDF_PATH}/tools/mocks/freertos/")

if("${IDF_VERSION_MAJOR}" VERSION_EQUAL "5" AND "${IDF_VERSION_MINOR}" VERSION_EQUAL "4")

  add_compile_definitions(SOC_CPU_CORES_NUM=1)

endif()

add_compile_definitions(SOC_UART_WAKEUP_SUPPORT_ACTIVE_THRESH_MODE=1)
add_compile_definitions(SOC_UART_NUM=2)
add_compile_definitions(CONFIG_MMU_PAGE_SIZE=64)

set(TEST_COMPONENTS
    "xiao_mmwave"
    CACHE STRING "List of components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it
# depends on.
idf_build_set_property(MINIMAL_BUILD ON)

project(test_app)
