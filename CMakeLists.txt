file(GLOB_RECURSE SOURCES src/*.c)
idf_component_register(SRCS ${SOURCES} "driver/bme280.c"
                       INCLUDE_DIRS "include" "driver"
                       PRIV_INCLUDE_DIRS "src"
                       REQUIRES driver freertos esp32)