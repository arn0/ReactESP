set(COMPONENT_SRCDIRS
    src
)

set(COMPONENT_ADD_INCLUDEDIRS
    src
)

set(COMPONENT_REQUIRES
    arduino-esp32
    esp_timer
    driver
)

register_component()
