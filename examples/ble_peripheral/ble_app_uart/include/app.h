#pragma once

#define CMD_H2D_ID_BIND     0x01

struct command_gen {
    uint8_t id;
    uint8_t data[1];
};

int app_bind_init(void);
int app_storage_init(void);

int bind_mark_bind_num(uint8_t *num_array);
