#ifndef _WEBSERVER
#define _WEBSERVER

typedef enum
{
    RST_NOT_ACTIVE = 0,
    RST_ONLY_CMD = 1,
    FACTORY_LOAD = 2,
    ROLLBACK_CMD = 3,
    RST_DEFL_CFG = 4,
    CLR_ARCH_CMD = 5,
    LD_UTC_CMD = 6,
} reset_state_t;

void webserver_init(void);
void webserver_start(void);
void clr_rst_state(void);
reset_state_t get_rst_state(void);
bool is_reset_time_out(void);

#define SOFT_AP_USE		CONFIG_SOFT_AP_USE

#endif // !defined(_WEBSERVER)
