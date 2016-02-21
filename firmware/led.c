#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "led.h"

void led_init(void)
{
	rcc_periph_reset_pulse(RST_GPIOB);

	led_off(LED_RX);
	led_off(LED_TX);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
			GPIO0 | GPIO1);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH,
			GPIO0 | GPIO1);
}

void led_on(enum led led)
{
	gpio_set(GPIOB, 1 << led);
}

void led_off(enum led led)
{
	gpio_clear(GPIOB, 1 << led);
}

void led_toggle(enum led led)
{
	gpio_toggle(GPIOB, 1 << led);
}
