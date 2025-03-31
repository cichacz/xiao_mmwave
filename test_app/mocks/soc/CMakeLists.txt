# NOTE: This kind of mocking currently works on Linux targets only.
#       On Espressif chips, too many dependencies are missing at the moment.
message(STATUS "building SOC MOCKS (only types)")

idf_component_get_property(original_soc_dir soc COMPONENT_OVERRIDEN_DIR)

set(include_dirs
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/soc"
    "${original_soc_dir}/include"
    "${original_soc_dir}/${target}/include")

# Note: "hal" and "soc" are only required for corresponding header files and their definitions
# here, they don't provide functionality when built for running on the host.
idf_component_mock(INCLUDE_DIRS ${include_dirs}
    MOCK_HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/soc/clk_tree_defs.h)
