#pragma once

#include <stdint.h>
#include <libopencm3/stm32/timer.h>

void hrtimer_init(void);
void delay_ticks(uint32_t ticks);

#define HZ 48000000
#define ticks TIM2_CNT

#define us_to_ticks(x) ((x) * (HZ/1000000))
#define ms_to_ticks(x) ((x) * (HZ/1000))
#define time_before(x,y) (((int32_t)(y) - (int32_t)(x)) > 0)
#define time_after(x,y) time_before(y,x)
#define usleep(x) delay_ticks(us_to_ticks(x))
