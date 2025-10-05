#ifndef _GTW_IEC_SLAVE
#define _GTW_IEC_SLAVE

typedef enum
{
    IEC_101_SER = 0,
    IEC_104_TCP = 1,
} conn_t;

void gtw_iec_slave_init(void);
void gtw_iec_slave_task(void *arg);
void printCP56Time2a(CP56Time2a time);
void rawMessageHandler(void *parameter, uint8_t *msg, int msgSize, bool sent);
bool clockSyncHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, CP56Time2a newTime);
bool interrogationHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, uint8_t qoi);
bool CounterInterrogationHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, QualifierOfCIC qcc);
bool asduHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu);
bool ReadHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, int ioa);
bool resetProcessHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, uint8_t qrp);
bool delayAcquisitionHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, CP16Time2a delayTime);
void iec_queue_send(void *slave, conn_t conn);
void end_of_init_send(void *slave, conn_t conn);
void check_utc_save(void);

#endif // !defined(__GTW_IEC_SLAVE)
