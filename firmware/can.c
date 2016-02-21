#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>

#include "can.h"

static bool can_active = false;

void bxcan_init(uint16_t brp, uint8_t ts1, uint8_t ts2, uint8_t sjw)
{
	rcc_periph_reset_pulse(RST_CAN);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, GPIO8 | GPIO9);

	// enter initialization mode
	CAN_MCR(CAN1) |= CAN_MCR_INRQ;

	while(!(CAN_MSR(CAN1) & CAN_MSR_INAK)) // TODO: add timeout handling
		;

	// disable sleep mode
	CAN_MCR(CAN1) &= ~CAN_MCR_SLEEP;

	// FIFO handling
	CAN_MCR(CAN1) |= CAN_MCR_TXFP; // transmit FIFO elements chronologically
	CAN_MCR(CAN1) |= CAN_MCR_RFLM; // don't overwrite messages on RX overflow

	// configure timings
	CAN_BTR(CAN1) = (sjw - 1) << 24 |
	                (ts2 - 1) << 20 |
	                (ts1 - 1) << 16 |
	                (brp - 1);

	// configure one "accept all" filter in bank 0
	CAN_FS1R(CAN1) |= 1 << 0;
	CAN_FiR1(CAN1, 0) = 0;
	CAN_FiR2(CAN1, 0) = 0;

	CAN_FA1R(CAN1) |= 1 << 0;
	CAN_FMR(CAN1) &= ~CAN_FMR_FINIT;

	// leave init mode
	CAN_MCR(CAN1) &= ~CAN_MCR_INRQ;

	while((CAN_MSR(CAN1) & CAN_MSR_INAK)) // TODO: add timeout handling
		;

	can_active = true;
}

void bxcan_stop(void)
{
	CAN_MCR(CAN1) |= CAN_MCR_SLEEP;

	while(!(CAN_MSR(CAN1) & CAN_MSR_SLAK))
		;

	can_active = false;
}

bool bxcan_is_active(void)
{
	return can_active;
}

int bxcan_send(struct can_msg *msg)
{
	uint32_t mbox;

	uint32_t tsr = CAN_TSR(CAN1);
	if(tsr & CAN_TSR_TME0)
		mbox = CAN_MBOX0;
	else if(tsr & CAN_TSR_TME1)
		mbox = CAN_MBOX1;
	else if(tsr & CAN_TSR_TME2)
		mbox = CAN_MBOX2;
	else
		return -1;

	// prepare ID register
	uint32_t tmp = 0;
	tmp |= msg->rtr ? CAN_TIxR_RTR : 0;

	if(msg->eid) {
		tmp |= CAN_TIxR_IDE;

		tmp |= msg->id << CAN_TIxR_EXID_SHIFT;
	} else {
		tmp |= msg->id << CAN_TIxR_STID_SHIFT;
	}

	CAN_TIxR(CAN1, mbox) = tmp;

	// prepare length register
	tmp = CAN_TDTxR(CAN1, mbox);
	tmp = (tmp & ~CAN_TDTxR_DLC_MASK) | (msg->dlc & CAN_TDTxR_DLC_MASK);
	CAN_TDTxR(CAN1, mbox) = tmp;

	// prepare data registers
	tmp = 0;
	for(int i = 0; i < msg->dlc; i++) {
		if(i == 4) {
			CAN_TDLxR(CAN1, mbox) = tmp;
			tmp = 0;
		}

		tmp |= msg->data[i] << ((i % 4) * 8);
	}

	if(msg->dlc > 4)
		CAN_TDHxR(CAN1, mbox) = tmp;
	else
		CAN_TDLxR(CAN1, mbox) = tmp;

	// request transmission
	CAN_TIxR(CAN1, mbox) |= CAN_TIxR_TXRQ;

	return mbox;
}

uint8_t bxcan_pending(void)
{
	return CAN_RF0R(CAN1) & CAN_RF0R_FMP0_MASK;
}

void bxcan_recv(struct can_msg *msg)
{
	uint32_t tmp;

	tmp = CAN_RI0R(CAN1);
	msg->eid = !!(tmp & CAN_RIxR_IDE);
	msg->rtr = !!(tmp & CAN_RIxR_RTR);

	if(msg->eid) {
		msg->id = (tmp & CAN_RIxR_EXID_MASK) >> CAN_RIxR_EXID_SHIFT;
	} else
		msg->id = (tmp & CAN_RIxR_STID_MASK) >> CAN_RIxR_STID_SHIFT;

	msg->dlc = (CAN_RDT0R(CAN1) & CAN_RDTxR_DLC_MASK) >> CAN_RDTxR_DLC_SHIFT;

	tmp = CAN_RDL0R(CAN1);
	for(int i = 0; i < msg->dlc; i++) {
		if(i == 4)
			tmp = CAN_RDH0R(CAN1);

		msg->data[i] = tmp & 0xFF;
		tmp >>= 8;
	}

	// release mailbox
	CAN_RF0R(CAN1) |= CAN_RF0R_RFOM0;
	while(CAN_RF0R(CAN1) & CAN_RF0R_RFOM0)
		;
}
