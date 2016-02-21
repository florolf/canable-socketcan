#pragma once

#include <stdint.h>

void ems_process_msg(uint8_t *buf, uint16_t len);
void ems_submit_data(void);
