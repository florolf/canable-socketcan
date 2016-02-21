#pragma once

#include <stdint.h>
#include <stdbool.h>

struct can_msg {
	uint32_t id;
	bool eid, rtr;

	uint8_t dlc;
	uint8_t data[8];
};

void bxcan_init(uint16_t brp, uint8_t ts1, uint8_t ts2, uint8_t sjw);
void bxcan_stop(void);
int bxcan_send(struct can_msg *msg);
void bxcan_recv(struct can_msg *msg);
uint8_t bxcan_pending(void);
bool bxcan_is_active(void);
