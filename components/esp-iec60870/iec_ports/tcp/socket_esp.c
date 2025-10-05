#include "hal_socket.h"
#include "iecc_slave.h"
#include "iec_port_common.h"
#include "iec_port_tcp_common.h"

static iecs_controller_iface_t *iface;

Socket
TcpSocket_create()
{
    return NULL;
}

void
Socket_destroy(Socket self)
{
}

void Socket_ready(void **ctx)
{
    iface = *ctx;
}

int
Socket_write(Socket self, uint8_t* buf, int size)
{
    iec_port_base_t *inst = iface->iec_base->port_obj;
    if (iecs_port_tcp_send_data(inst, buf, size) == true)
        return size;
    return 0;
}