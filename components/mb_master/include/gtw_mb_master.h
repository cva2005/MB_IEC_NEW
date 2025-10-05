#pragma once

#include "gtw_params.h"

#ifdef __cplusplus
extern "C" {
#endif

void mb_master_register(void *handle, mb_connect_t conn_idx);
void mb_master_operation_func(void);

#ifdef __cplusplus
}
#endif
