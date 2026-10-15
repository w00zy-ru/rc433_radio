idf_component_register(SRCS "rc433_receiver.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_gpio esp_timer esp_ringbuf)
