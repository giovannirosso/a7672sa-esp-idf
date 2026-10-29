idf_component_register(
    SRCS "a7672sa.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
    PRIV_REQUIRES esp_event
)
