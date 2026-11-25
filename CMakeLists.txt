cmake_minimum_required(VERSION 3.22)

# You can set a HW here when e.g. building with the eclipse plugin
# set(HW_NAME "Wcore")

# Default build target Logic
if(NOT DEFINED HW_NAME OR "${HW_NAME}" STREQUAL "")
    if(DEFINED IDF_TARGET)
        if(IDF_TARGET STREQUAL "esp32")
            set(HW_NAME "Devkit ESP32 4MB")
        elseif(IDF_TARGET STREQUAL "esp32s3")
            set(HW_NAME "Devkit S3 FH4")
        elseif(IDF_TARGET STREQUAL "esp32c6")
            set(HW_NAME "Devkit C6 FH4")
        elseif(IDF_TARGET STREQUAL "esp32p4")
            set(HW_NAME "P4 NRW32N16")
        elseif(IDF_TARGET STREQUAL "esp32c3")
            set(HW_NAME "Devkit C3")
        elseif(IDF_TARGET STREQUAL "esp32s3_fh4")
            set(HW_NAME "Devkit S3 FH4")
        elseif(IDF_TARGET STREQUAL "esp32c6_fh4")
            set(HW_NAME "Devkit C6 FH4")
        elseif(IDF_TARGET STREQUAL "esp32p4_nrw32n16")
            set(HW_NAME "P4 NRW32N16")
        elseif(IDF_TARGET STREQUAL "esp32s31")
            set(HW_NAME "Devkit S31 N16R16V")
        endif()
    endif()

    if(NOT DEFINED HW_NAME OR "${HW_NAME}" STREQUAL "")
        set(HW_NAME "Devkit C3")
    endif()
endif()

# Look for hardware header files based on HW_NAME
file(GLOB_RECURSE HW_HEADER_PATHS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    main/hwconf/hw_*.h
)

foreach(HW_HEADER_PATH ${HW_HEADER_PATHS})
    file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_HEADER_PATH}" FILE_CONTENTS)
    set(IS_HW_MATCH false)
    set(FILE_HW_TARGET "")

    foreach(LINE IN LISTS FILE_CONTENTS)
        string(REGEX MATCH "^#define[ \t]+([A-Za-z_][A-Za-z0-9_]*)([ \t]+(.*))?$" MATCHES "${LINE}")
        if(MATCHES)
            set(DEF_KEY ${CMAKE_MATCH_1})
            set(DEF_VAL ${CMAKE_MATCH_3})

            # Remove surrounding quotes
            if(DEFINED DEF_VAL AND DEF_VAL MATCHES "^\".*\"$")
                string(REGEX REPLACE "^\"(.*)\"$" "\\1" DEF_VAL ${DEF_VAL})
            endif()

            if (DEF_KEY STREQUAL "HW_NAME" AND DEF_VAL STREQUAL "${HW_NAME}")
                set(IS_HW_MATCH true)
            endif()

            if (DEF_KEY STREQUAL "HW_TARGET")
                set(FILE_HW_TARGET ${DEF_VAL})
            endif()
        endif()
    endforeach()

    if(IS_HW_MATCH AND NOT "${FILE_HW_TARGET}" STREQUAL "")
        set(HW_TARGET ${FILE_HW_TARGET})
        set(HW_FILE_NAME ${HW_HEADER_PATH})
        break()
    endif()
endforeach()

if(DEFINED HW_NAME AND DEFINED HW_TARGET AND DEFINED HW_FILE_NAME)
    # HW_TARGET must include variant suffix, e.g. esp32c6_fh4.
    string(REGEX MATCH "^([a-z0-9]+)(_.+)?$" HW_TARGET_MATCH "${HW_TARGET}")
    if(HW_TARGET_MATCH)
        set(HW_IDF_TARGET ${CMAKE_MATCH_1})
    else()
        set(HW_IDF_TARGET ${HW_TARGET})
    endif()

    # Get file name without extension
    string(REGEX MATCH "([^/]+)$" HW_FILE ${HW_FILE_NAME})
    string(REPLACE ".h" "" HW_FILE ${HW_FILE})

    # IMPORTANT: Set IDF_TARGET before including project.cmake
    if(NOT DEFINED IDF_TARGET)
        SET(IDF_TARGET ${HW_IDF_TARGET})
    elseif(NOT IDF_TARGET STREQUAL HW_IDF_TARGET)
        message(WARNING "IDF_TARGET (${IDF_TARGET}) does not match HW_IDF_TARGET (${HW_IDF_TARGET}) derived from HW_TARGET (${HW_TARGET}) for ${HW_NAME}. This may cause build issues.")
    endif()
    
    # Pass through hw file paths
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_FILE_NAME}")

    set(SDKCONFIG_DEFAULTS_FILE "sdkconfig.defaults.${HW_IDF_TARGET}")
    set(HW_TARGET_SDKCONFIG_DEFAULTS_FILE "sdkconfig.defaults.${HW_TARGET}")

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_TARGET_SDKCONFIG_DEFAULTS_FILE}")
        set(SDKCONFIG_DEFAULTS_FILE "${HW_TARGET_SDKCONFIG_DEFAULTS_FILE}")
    else()
        message(STATUS "Using shared sdkconfig defaults ${SDKCONFIG_DEFAULTS_FILE} for ${HW_NAME}")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${SDKCONFIG_DEFAULTS_FILE}")
    set(SDKCONFIG_DEFAULTS ${SDKCONFIG_DEFAULTS_FILE})
else ()
    message(FATAL_ERROR "Build target info missing.\nHW_NAME: ${HW_NAME}\nHW_TARGET: ${HW_TARGET}\nHW_FILE: ${HW_FILE_NAME}") 
endif()

message("Processing target ${HW_NAME} on ${HW_TARGET} for file ${HW_FILE_NAME}")

# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
set(PROJECT_VER "6.0") 

if(SDKCONFIG)
    get_filename_component(sdkconfig_path "${SDKCONFIG}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
else()
    set(sdkconfig_path "${CMAKE_SOURCE_DIR}/sdkconfig")
endif()
set(sdkconfig_board_key "${HW_NAME}\n${HW_TARGET}\n")
set(sdkconfig_owner_file "${sdkconfig_path}.board")
set(sdkconfig_previous_board "")
if(EXISTS "${sdkconfig_owner_file}")
    file(READ "${sdkconfig_owner_file}" sdkconfig_previous_board)
endif()
if(NOT "${sdkconfig_previous_board}" STREQUAL "${sdkconfig_board_key}" AND EXISTS "${sdkconfig_path}")
    message(STATUS "Hardware changed or sdkconfig ownership unknown; regenerating ${sdkconfig_path} from ${SDKCONFIG_DEFAULTS}")
    file(REMOVE "${sdkconfig_path}")
endif()

# Include ESP-IDF project cmake
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(VESC_HW_FILE ${HW_FILE})

# ESP-IDF remainder boilerplate
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})

file(WRITE "${sdkconfig_owner_file}" "${sdkconfig_board_key}")

include(cmake/git_rev_parse.cmake)

if(DEFINED ENV{GIT_COMMIT_HASH})
    set(GIT_COMMIT_HASH $ENV{GIT_COMMIT_HASH})
else()
    git_describe(GIT_COMMIT_HASH ".")
endif()

if(DEFINED ENV{GIT_BRANCH_NAME})
    set(GIT_BRANCH_NAME $ENV{GIT_BRANCH_NAME})
else()
    git_rev_parse(GIT_BRANCH_NAME "." "--abbrev-ref" "HEAD")
endif()

idf_build_set_property(COMPILE_DEFINITIONS "GIT_COMMIT_HASH=\"${GIT_COMMIT_HASH}\"" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "GIT_BRANCH_NAME=\"${GIT_BRANCH_NAME}\"" APPEND)

if(DEFINED ENV{USER_GIT_COMMIT_HASH})
    idf_build_set_property(COMPILE_DEFINITIONS "USER_GIT_COMMIT_HASH=\"$ENV{USER_GIT_COMMIT_HASH}\"" APPEND)
endif()
if(DEFINED ENV{USER_GIT_BRANCH_NAME})
    idf_build_set_property(COMPILE_DEFINITIONS "USER_GIT_BRANCH_NAME=\"$ENV{USER_GIT_BRANCH_NAME}\"" APPEND)
endif()

idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
