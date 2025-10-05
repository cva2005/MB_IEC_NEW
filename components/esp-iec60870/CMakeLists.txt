set(srcs
    #iec_ports/common/iec_transaction.c
    iec_ports/common/iec_port_event.c
    iec_ports/common/iec_port_other.c
    iec_ports/common/iec_port_timer.c
    iec_ports/serial/iec_port_serial.c
    iec_ports/tcp/iec_port_tcp_driver.c
    iec_ports/tcp/iec_port_tcp_slave.c
    iec_ports/tcp/iec_port_tcp_utils.c
    iec_ports/tcp/socket_esp.c
    iec_ports/iec_slave.c
    controller/esp_iec_slave.c
    controller/esp_iec_slave_serial.c
    controller/esp_iec_slave_tcp.c
    controller/iecc_serial_slave.c
    controller/iecc_tcp_slave.c
    iec_transports/ser/iec_ser_slave.c
    iec_transports/tcp/iec_tcp_slave.c
    file-service/file_server.c
    iec60870/apl/cpXXtime2a.c
    iec60870/cs101/cs101_asdu.c
    iec60870/cs101/cs101_bcr.c
    iec60870/cs101/cs101_information_objects.c
    iec60870/cs101/cs101_master.c
    iec60870/cs101/cs101_master_connection.c
    iec60870/cs101/cs101_queue.c
    iec60870/cs101/cs101_slave.c
    iec60870/cs104/cs104_connection.c
    iec60870/cs104/cs104_frame.c
    iec60870/cs104/cs104_slave.c
    iec60870/link_layer/buffer_frame.c
    iec60870/link_layer/link_layer.c
    iec60870/link_layer/serial_transceiver_ft_1_2.c
    iec60870/frame.c
    iec60870/lib60870_common.c
    common/linked_list.c
    #hal/memory/lib_memory.c
    )

set(include_dirs
    hal/inc
    include/api
    include/internal
    controller
    controller/include
    iec_ports/include
    iec_transports
    )

set(priv_include_dirs
    file-service
    hal/inc
    ../../main/include
    iec_ports/include
    iec_ports/common
    iec_ports/serial
    iec_ports/tcp
    iec_transports
    iec_transports/tcp
    iec_transports/ser
    )

list(APPEND include_dirs
    )

set(requires driver)
set(priv_requires esp_netif esp_event vfs)

# esp_timer component was introduced in v4.2
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER "4.1")
    list(APPEND requires esp_timer)
endif()

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${include_dirs}"
                    PRIV_INCLUDE_DIRS "${priv_include_dirs}"
                    PRIV_REQUIRES main
                    REQUIRES ${requires} 
                    PRIV_REQUIRES ${priv_requires})