#pragma once

void usb_init(void);
void usb_poll(void);
bool usb_can_push_data(void);
void usb_push_data(uint8_t *buf, uint16_t len);
