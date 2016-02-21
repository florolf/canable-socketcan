#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "hrtimer.h"

void hrtimer_init(void)
{
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_reset_pulse(RST_TIM2);

	TIM2_CR1 |= TIM_CR1_CEN;
}

void delay_ticks(uint32_t cnt)
{
	uint32_t timeout = ticks + cnt;

	while(time_before(ticks, timeout))
		;
}
