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

"config/confsrc.c"
"hwconf/hw.c"
)

set(COMPONENT_ADD_INCLUDEDIRS
"."
"config"
"hwconf"
"hwconf/devkits"
)

idf_component_register(SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
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
