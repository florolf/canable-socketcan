#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/can.h>

#include "hrtimer.h"
#include "led.h"
#include "usb.h"
#include "can.h"
#include "ems.h"

static void periphs_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_CAN);
}

void main(void)
{
	rcc_clock_setup_in_hsi48_out_48mhz();

	hrtimer_init();
	periphs_init();

	usb_init();
	led_init();

	while(1) {
		usb_poll();

		if(usb_can_push_data())
			ems_submit_data();
	}
}
