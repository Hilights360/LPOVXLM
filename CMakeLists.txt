cmake_minimum_required(VERSION 3.16)
set(IDF_TARGET esp32s3 CACHE STRING "POV controller target")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lpovxlm VERSION 1.2)

# Refresh identity on every application build, including incremental builds.
# Both supported build folders share the counter, but embed their own snapshot.
set(pov_build_header "${CMAKE_BINARY_DIR}/generated/pov_build_info.h")
idf_build_get_property(pov_project_version PROJECT_VER)
add_custom_target(pov_build_identity
    COMMAND "${CMAKE_COMMAND}"
        "-DCOUNTER_FILE=${CMAKE_SOURCE_DIR}/build/firmware_build_number.txt"
        "-DOUTPUT_DIR=${CMAKE_BINARY_DIR}/generated"
        "-DFIRMWARE_VERSION=${pov_project_version}"
        -P "${CMAKE_SOURCE_DIR}/cmake/build_identity.cmake"
    BYPRODUCTS "${pov_build_header}" "${CMAKE_BINARY_DIR}/generated/build_info.json"
    COMMENT "Updating firmware version, build number, and timestamp"
    VERBATIM)
idf_component_get_property(pov_main_lib main COMPONENT_LIB)
add_dependencies(${pov_main_lib} pov_build_identity)
target_include_directories(${pov_main_lib} PRIVATE "${CMAKE_BINARY_DIR}/generated")

# IDF's app descriptor uses __DATE__/__TIME__. Depending on the changing
# generated header also refreshes that object instead of retaining its first
# compilation time. This modifies only this project's build, not the IDF SDK.
idf_component_get_property(pov_app_desc_lib esp_app_format COMPONENT_LIB)
add_dependencies(${pov_app_desc_lib} pov_build_identity)
target_compile_options(${pov_app_desc_lib} PRIVATE "-include${pov_build_header}")

# Compile the SD driver's failure details; the SD tests enable only
# sdmmc_req at DEBUG temporarily. Normal firmware logging remains at INFO.
idf_component_get_property(pov_sdmmc_lib esp_driver_sdmmc COMPONENT_LIB)
target_compile_definitions(${pov_sdmmc_lib} PRIVATE LOG_LOCAL_LEVEL=4)
