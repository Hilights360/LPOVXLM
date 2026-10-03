foreach(required COUNTER_FILE OUTPUT_DIR FIRMWARE_VERSION)
    if(NOT DEFINED ${required} OR "${${required}}" STREQUAL "")
        message(FATAL_ERROR "Missing build identity setting: ${required}")
    endif()
endforeach()

get_filename_component(counter_dir "${COUNTER_FILE}" DIRECTORY)
file(MAKE_DIRECTORY "${counter_dir}" "${OUTPUT_DIR}")
# Serializes numbering if different build folders are built at the same time.
file(LOCK "${COUNTER_FILE}.lock" GUARD PROCESS TIMEOUT 30)
set(build_number 0)
if(EXISTS "${COUNTER_FILE}")
    file(READ "${COUNTER_FILE}" build_number)
    string(STRIP "${build_number}" build_number)
    if(NOT build_number MATCHES "^(0|[1-9][0-9]*)$")
        message(FATAL_ERROR "Invalid build number in ${COUNTER_FILE}")
    endif()
endif()
math(EXPR build_number "${build_number} + 1")
file(WRITE "${COUNTER_FILE}.tmp" "${build_number}\n")
file(RENAME "${COUNTER_FILE}.tmp" "${COUNTER_FILE}")

string(TIMESTAMP build_time "%Y-%m-%d %H:%M:%S")
string(TIMESTAMP build_time_utc "%Y-%m-%dT%H:%M:%SZ" UTC)
file(WRITE "${OUTPUT_DIR}/pov_build_info.h"
    "#pragma once\n"
    "#define LPOV_BUILD_NUMBER ${build_number}\n"
    "#define LPOV_BUILD_TIME \"${build_time}\"\n"
    "#define LPOV_BUILD_TIME_UTC \"${build_time_utc}\"\n")
file(WRITE "${OUTPUT_DIR}/build_info.json"
    "{\n"
    "  \"version\": \"${FIRMWARE_VERSION}\",\n"
    "  \"buildNumber\": ${build_number},\n"
    "  \"buildTime\": \"${build_time}\",\n"
    "  \"buildTimeUtc\": \"${build_time_utc}\"\n"
    "}\n")
message(STATUS "Firmware Ver${FIRMWARE_VERSION} | Build ${build_number} | ${build_time}")
