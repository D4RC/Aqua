idf_component_register(SRCS "vl53l0x.c" "vl53l0x_platform.c" "vl53l0x_api.c" "vl53l0x_api_strings.c" "vl53l0x_api_ranging.c" "vl53l0x_api_core.c" "vl53l0x_api_calibration.c"
                    INCLUDE_DIRS "include")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-variable)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-maybe-uninitialized)
