idf_component_register(SRCS "gtw_mb_master.c" "tcp_master.c" "serial_master.c" "gtw_params.c"
                    PRIV_REQUIRES esp-modbus main
                    INCLUDE_DIRS "." include)
