#pragma once

void ota_factory_reload(void);
void prepare_factory_reload(void);
bool prepare_rollback(void);
void ota_rollback(void);
void reboot_as_deep_sleep(void);
