#include <stdint.h>
#include <string.h>

#include <libopencm3/cm3/assert.h>
#include <libopencm3/stm32/can.h>

#include "can.h"
#include "led.h"
#include "usb.h"

#include "ems.h"

/* Messages from CPC to PC */
#define CPC_MSG_TYPE_CAN_FRAME       1  /* CAN data frame */
#define CPC_MSG_TYPE_RTR_FRAME       8  /* CAN remote frame */
#define CPC_MSG_TYPE_CAN_PARAMS      12 /* Actual CAN parameters */
#define CPC_MSG_TYPE_CAN_STATE       14 /* CAN state message */
#define CPC_MSG_TYPE_EXT_CAN_FRAME   16 /* Extended CAN data frame */
#define CPC_MSG_TYPE_EXT_RTR_FRAME   17 /* Extended remote frame */
#define CPC_MSG_TYPE_CONTROL         19 /* change interface behavior */
#define CPC_MSG_TYPE_CONFIRM         20 /* command processed confirmation */
#define CPC_MSG_TYPE_OVERRUN         21 /* overrun events */
#define CPC_MSG_TYPE_CAN_FRAME_ERROR 23 /* detected bus errors */
#define CPC_MSG_TYPE_ERR_COUNTER     25 /* RX/TX error counter */

/* Messages from the PC to the CPC interface  */
#define CPC_CMD_TYPE_CAN_FRAME     1   /* CAN data frame */
#define CPC_CMD_TYPE_CONTROL       3   /* control of interface behavior */
#define CPC_CMD_TYPE_CAN_PARAMS    6   /* set CAN parameters */
#define CPC_CMD_TYPE_RTR_FRAME     13  /* CAN remote frame */
#define CPC_CMD_TYPE_CAN_STATE     14  /* CAN state message */
#define CPC_CMD_TYPE_EXT_CAN_FRAME 15  /* Extended CAN data frame */
#define CPC_CMD_TYPE_EXT_RTR_FRAME 16  /* Extended CAN remote frame */
#define CPC_CMD_TYPE_CAN_EXIT      200 /* exit the CAN */

#define CPC_CMD_TYPE_INQ_ERR_COUNTER 25 /* request the CAN error counters */
#define CPC_CMD_TYPE_CLEAR_MSG_QUEUE 8  /* clear CPC_MSG queue */
#define CPC_CMD_TYPE_CLEAR_CMD_QUEUE 28 /* clear CPC_CMD queue */

/* Mode register NXP LPC2119/SJA1000 CAN Controller */
#define SJA1000_MOD_NORMAL 0x00
#define SJA1000_MOD_RM     0x01

/* Status register content */
#define SJA1000_SR_BS 0x80
#define SJA1000_SR_ES 0x40

/* ECC register NXP LPC2119/SJA1000 CAN Controller */
#define SJA1000_ECC_SEG   0x1F
#define SJA1000_ECC_DIR   0x20
#define SJA1000_ECC_ERR   0x06
#define SJA1000_ECC_BIT   0x00
#define SJA1000_ECC_FORM  0x40
#define SJA1000_ECC_STUFF 0x80
#define SJA1000_ECC_MASK  0xc0

#define CPC_CC_TYPE_SJA1000 2 /* Philips basic CAN controller */
#define CPC_CAN_ECODE_ERRFRAME 0x01 /* Ecode type */

static void ems_can_params(uint8_t *buf)
{
	// controller type must be SJA1000 (2)
	cm3_assert(buf[0] == CPC_CC_TYPE_SJA1000);

	if(buf[1] == SJA1000_MOD_RM) {
		bxcan_stop();
		return;
	}

	uint8_t btr0, btr1;
	btr0 = buf[10];
	btr1 = buf[11];

	uint8_t sja_brp, ts1, ts2, sjw;
	sja_brp = (btr0 & 0x3f) + 1;
	sjw = ((btr0 >> 6) & 0x03) + 1;
	ts1 = (btr1 & 0x0f) + 1;
	ts2 = ((btr1 >> 4) & 0x07) + 1;

	/*
	 * the ems_usb kernel module calculates the BRP based on a clock rate
	 * of 8 MHz, but we run on 48 MHz.
	 */

	uint16_t brp = 6 * sja_brp;

	bxcan_init(brp, ts1, ts2, sjw);
}

static void ems_control(uint8_t *buf) {
	/*
	 * The Linux kernel module only uses control commands during the
	 * initial enumeration of the adapter. Instead of implementing all the
	 * flags here, we simply behave as if we had understood them in the
	 * rest of the code.
	 *
	 * FIXME: Maybe we need to do this to be compatible with the Windows driver.
	 */

	(void) buf;
}

static void ems_can_msg(uint8_t type, uint8_t *buf)
{
	struct can_msg msg;

	msg.id = *(uint32_t*)buf;
	msg.dlc = buf[4];
	memcpy(msg.data, &buf[5], msg.dlc);

	msg.rtr = msg.eid = false;
	if(type == CPC_CMD_TYPE_EXT_CAN_FRAME || type == CPC_CMD_TYPE_EXT_RTR_FRAME)
		msg.eid = true;

	if(type == CPC_CMD_TYPE_RTR_FRAME || type == CPC_CMD_TYPE_EXT_RTR_FRAME)
		msg.rtr = true;

	int buffer;
	do {
		buffer = bxcan_send(&msg);
	} while(buffer < 0);

	led_toggle(LED_TX);
}

void ems_process_msg(uint8_t *buf, uint16_t len)
{
	if(len < 16)
		return;

	// skip empty header
	buf += 4;

	uint8_t type = buf[0];
	buf += 11; // skip to body

	switch(type) {
		case CPC_CMD_TYPE_CAN_PARAMS:
			ems_can_params(buf);
			return;

		case CPC_CMD_TYPE_CONTROL:
			ems_control(buf);
			return;

		case CPC_CMD_TYPE_EXT_RTR_FRAME:
		case CPC_CMD_TYPE_RTR_FRAME:
		case CPC_CMD_TYPE_EXT_CAN_FRAME:
		case CPC_CMD_TYPE_CAN_FRAME:
			ems_can_msg(type, buf);
			return;
	}
}

static uint8_t msgid = 0;

#define EMS_MSG_HEADER_LEN 11
static void ems_pack_msg_header(uint8_t *buf, uint8_t type, uint8_t len)
{
	buf[0] = type; // type
	buf[1] = len; // body length
	buf[2] = msgid++; // msgid

	memset(&buf[3], 0, 8); // timestamp, ignored
}

static uint8_t ems_pack_state(uint8_t *buf, uint16_t len)
{
	uint32_t esr = CAN_ESR(CAN1);

	if(esr & (CAN_ESR_EWGF | CAN_ESR_EPVF | CAN_ESR_BOFF))
		return 0;

	cm3_assert(len >= EMS_MSG_HEADER_LEN + 1);

	ems_pack_msg_header(buf, CPC_CMD_TYPE_CAN_STATE, 1);
	buf += EMS_MSG_HEADER_LEN;

	if(esr & CAN_ESR_BOFF)
		buf[0] = SJA1000_SR_BS;
	else if(esr & CAN_ESR_EPVF)
		buf[0] = 0x01;
	else if(esr & CAN_ESR_EWGF)
		buf[0] = SJA1000_SR_ES;

	return EMS_MSG_HEADER_LEN + 1;
}

static uint8_t ems_pack_error(uint8_t *buf, uint16_t len)
{
	uint32_t esr = CAN_ESR(CAN1);

	if((esr & CAN_ESR_LEC_MASK) == CAN_ESR_LEC_NO_ERROR)
		return 0;

	cm3_assert(len >= EMS_MSG_HEADER_LEN + 5);

	ems_pack_msg_header(buf, CPC_MSG_TYPE_CAN_FRAME_ERROR, 5);
	buf += EMS_MSG_HEADER_LEN;

	buf[0] = CPC_CAN_ECODE_ERRFRAME; // ecode
	buf[1] = CPC_CC_TYPE_SJA1000; // cc type

	// ecc
	switch(esr & CAN_ESR_LEC_MASK) {
		case CAN_ESR_LEC_DOM_ERROR:
		case CAN_ESR_LEC_REC_ERROR:
			buf[2] = SJA1000_ECC_BIT;
			break;
		case CAN_ESR_LEC_FORM_ERROR:
			buf[2] = SJA1000_ECC_FORM;
			break;
		case CAN_ESR_LEC_STUFF_ERROR:
			buf[2] = SJA1000_ECC_STUFF;
			break;
	}

	buf[3] = (esr >> 24) & 0xFF; // rxerr
	buf[4] = (esr >> 16) & 0xFF; // txerr

	return EMS_MSG_HEADER_LEN + 5;
}

static uint8_t ems_pack_overrun(uint8_t *buf, uint16_t len)
{
	if(!(CAN_RF0R(CAN1) & CAN_RF0R_FAVR0))
		return 0;

	cm3_assert(len >= EMS_MSG_HEADER_LEN + 2);

	ems_pack_msg_header(buf, CPC_MSG_TYPE_OVERRUN, 2);
	buf += EMS_MSG_HEADER_LEN;

	// message content is ignored by the Linux kernel module
	buf[0] = 0;
	buf[1] = 0;

	return EMS_MSG_HEADER_LEN + 2;
}

uint16_t ems_prepare_buffer(uint8_t *buf, uint16_t len)
{
	uint8_t *p = &buf[4];
	uint8_t msg_cnt = 0;
	uint8_t ret;

	if(!bxcan_is_active())
		return 0;

#if 0
	// pack bus state
	// FIXME: only do this on a status change
	ret = ems_pack_state(p, len);
	if(ret) {
		msg_cnt++;

		p += ret;
		len -= ret;
	}

	// pack error state
	// FIXME: only do this on a status change
	ret = ems_pack_error(p, len);
	if(ret) {
		msg_cnt++;

		p += ret;
		len -= ret;
	}

	// pack overflows
	ret = ems_pack_overrun(p, len);
	if(ret) {
		msg_cnt++;

		p += ret;
		len -= ret;
	}
#endif

	// pack messages
	while(bxcan_pending()) {
		led_toggle(LED_RX);

		struct can_msg msg;
		bxcan_recv(&msg);

		if(len < EMS_MSG_HEADER_LEN + 13)
			break;

		uint8_t type;
		if(msg.eid == false && msg.rtr == false)
			type = CPC_MSG_TYPE_CAN_FRAME;
		else if(msg.eid == false && msg.rtr == true)
			type = CPC_MSG_TYPE_RTR_FRAME;
		else if(msg.eid == true && msg.rtr == false)
			type = CPC_MSG_TYPE_EXT_CAN_FRAME;
		else if(msg.eid == true && msg.rtr == true)
			type = CPC_MSG_TYPE_EXT_RTR_FRAME;

		ems_pack_msg_header(p, type, 13);
		p += EMS_MSG_HEADER_LEN;

		msg_cnt++;
		*p++ = (msg.id      ) & 0xFF;
		*p++ = (msg.id >>  8) & 0xFF;
		*p++ = (msg.id >> 16) & 0xFF;
		*p++ = (msg.id >> 24) & 0xFF;

		*p++ = msg.dlc;

		memcpy(p, msg.data, msg.dlc);
		p += 8;

		len -= EMS_MSG_HEADER_LEN + 13;
	}

	buf[0] = msg_cnt;

	if(msg_cnt)
		return p - buf;
	else
		return 0;
}

void ems_submit_data(void)
{
	uint8_t buf[64] __attribute__ ((aligned(4)));

	uint16_t len = ems_prepare_buffer(buf, 64);

	if(len)
		usb_push_data(buf, len);
}
