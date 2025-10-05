idf_component_register(
SRCS
    ser_slave_iec.c
    tcp_slave_iec.c
    gtw_iec_slave.c
PRIV_REQUIRES main
INCLUDE_DIRS "." include)