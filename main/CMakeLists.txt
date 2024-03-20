idf_component_register(SRCS "main.c" "setup.c"
        INCLUDE_DIRS "."
        PRIV_REQUIRES bme680 driver esp_timer pthread)