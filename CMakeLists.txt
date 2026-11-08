# ESP-IDF (all ESP32 chips, 4.x / 5.x) and ESP8266_RTOS_SDK.
# IDF_TARGET is a build property: read it with idf_build_get_property.
idf_build_get_property(_ds_target IDF_TARGET)
if(_ds_target STREQUAL "esp8266")
    idf_component_register(SRCS "onewire.c" "ds18b20.c"
                        INCLUDE_DIRS "."
                        REQUIRES esp8266 freertos)
else()
    idf_component_register(SRCS "onewire.c" "ds18b20.c"
                        INCLUDE_DIRS "."
                        REQUIRES driver esp_rom freertos)
endif()
