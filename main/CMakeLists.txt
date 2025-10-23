file(
    GLOB_RECURSE
        SOURCES
            ./*.c
)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS
        ./
    REQUIRES
        vfs
        driver
        nvs_flash
        esp_timer
)