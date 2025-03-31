# This is the project CMakeLists.txt file for the test subproject
cmake_minimum_required(VERSION 3.16)

# Include the components directory of the main application:
#
set(EXTRA_COMPONENT_DIRS "../")
set(CMAKE_HOST_SYSTEM_NAME "Darwin")

list(APPEND EXTRA_COMPONENT_DIRS "./mocks/soc/")
list(APPEND EXTRA_COMPONENT_DIRS "./mocks/driver/")
list(APPEND EXTRA_COMPONENT_DIRS "$ENV{IDF_PATH}/tools/mocks/freertos/")

add_compile_definitions(SOC_CPU_CORES_NUM=1)
add_compile_definitions(SOC_UART_NUM=2)
add_compile_definitions(CONFIG_MMU_PAGE_SIZE=64)

set(TEST_COMPONENTS "xiao_mmwave" CACHE STRING "List of components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)

project(test_app)
