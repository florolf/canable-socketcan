#pragma once

enum led {
	LED_TX = 0,
	LED_RX = 1,
};

void led_init(void);
void led_on(enum led led);
void led_off(enum led led);
void led_toggle(enum led led);
