set(COMPONENT_SRCS
"main.c"
"buffer.c"
"comm_uart.c"
"comm_usb.c"
"comm_can.c"
"packet.c"
"crc.c"
"commands.c"
"terminal.c"
"mempools.c"
"flash_helper.c"
"lispif/lispif.c"
"lispif/lispif_vesc_extensions.c"
"lispif/modules/can_extensions.c"
"lispif/modules/system_extensions.c"
"lispif/modules/eeprom_extensions.c"

"lispBM/src/env.c"
"lispBM/src/fundamental.c"
"lispBM/src/heap.c"
"lispBM/src/lbm_memory.c"
"lispBM/src/print.c"
"lispBM/src/stack.c"
"lispBM/src/symrepr.c"
"lispBM/src/tokpar.c"
"lispBM/src/extensions.c"
"lispBM/src/lispbm.c"
"lispBM/src/eval_cps.c"
"lispBM/src/lbm_channel.c"
"lispBM/src/lbm_c_interop.c"
"lispBM/src/lbm_custom_type.c"
"lispBM/src/lbm_flat_value.c"
"lispBM/src/lbm_defrag_mem.c"
"lispBM/src/lbm_image.c"
"lispBM/src/lbm_prof.c"
"lispBM/src/extensions/array_extensions.c"
"lispBM/src/extensions/math_extensions.c"
"lispBM/src/extensions/string_extensions.c"
"lispBM/src/extensions/mutex_extensions.c"
"lispBM/src/extensions/lbm_dyn_lib.c"
"lispBM/platform/freertos/src/platform_mutex.c"

"config/confsrc.c"
"hwconf/hw.c"
)

set(COMPONENT_ADD_INCLUDEDIRS
"."
"config"
"hwconf"
"hwconf/devkits"
"lispif"
"lispBM/include"
"lispBM/platform/freertos/include"
)

idf_component_register(SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    LBM_USE_DYN_FUNS LBM_USE_DYN_MACROS LBM_USE_DYN_LOOPS LBM_USE_DYN_ARRAYS
    LBM_USE_ERROR_LINENO LBM_USE_MACRO_REST_ARGS
)

if(NOT DEFINED ENV{HW_SRC} AND NOT DEFINED ENV{HW_HEADER})
    idf_build_get_property(hw_file VESC_HW_FILE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE HW_HEADER="${hw_file}.h" HW_SOURCE="${hw_file}.c")
else()
    if(NOT DEFINED ENV{HW_SRC})
        message(FATAL_ERROR "HW_SRC not defined while HW_HEADER is set. You must either set both or none.")
    endif()
    if(NOT DEFINED ENV{HW_HEADER})
        message(FATAL_ERROR "HW_HEADER not defined while HW_SRC is set. You must either set both or none.")
    endif()
    
    message(STATUS "Setting HW_SOURCE=\"$ENV{HW_SRC}\" and HW_HEADER=\"$ENV{HW_HEADER}\" definitions")
    
    add_compile_definitions(
        HW_SOURCE=\"$ENV{HW_SRC}\"
        HW_HEADER=\"$ENV{HW_HEADER}\"
    )
endif()
