idf_component_register(
    SRCS
        main.c
        gpio_drv.c
        timer_1ms.c
        webserver.c
        config.c
        arch.c
        switch_ota.c
    INCLUDE_DIRS "." include
)