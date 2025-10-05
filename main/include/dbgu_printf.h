#pragma once
#include "sdkconfig.h"
#if CONFIG_DBGU_PRINTF == 0
int printf(const char *fmt, ...){return 0;}
#endif