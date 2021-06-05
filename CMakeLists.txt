idf_component_register(SRCS "core/src/vl53l0x_api_calibration.c"
                            "core/src/vl53l0x_api_core.c"
                            "core/src/vl53l0x_api_ranging.c"
                            "core/src/vl53l0x_api_strings.c"
                            "core/src/vl53l0x_api.c"
                            "platform/src/vl53l0x_platform_log.c"
                            "platform/src/vl53l0x_platform.c"
                            "src/vl53l0x.c"
                       INCLUDE_DIRS "core/inc" "platform/inc" "src")
target_compile_definitions(${COMPONENT_LIB} PUBLIC _FREE_RTOS_=1)
message("building api")