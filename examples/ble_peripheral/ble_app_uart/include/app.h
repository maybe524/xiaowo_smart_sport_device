#pragma once

int app_bind_init(void);
int app_storage_init(void);
int app_vibr_init(void);
int app_accelerator_init(void);
int app_hr_oximeter_init(void);
int app_vibr_init(void);
int app_set_bind_num(uint8_t *num_array);

bool app_send_2host(uint8_t *data_array, uint16_t length);
bool app_get_bleconn_status(void);
