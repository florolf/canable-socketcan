#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/st_usbfs.h>

#include "usb.h"
#include "led.h"
#include "ems.h"

static usbd_device *usbd_dev;

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xff,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x12d6,
	.idProduct = 0x444,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_endpoint_descriptor transfer_endp[] = {
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x02,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 0,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x81,
		.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
		.wMaxPacketSize = 16,
		.bInterval = 128,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x82,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 0,
	},
};

const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 3,
	.bInterfaceClass = 0xFF,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = transfer_endp,
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
		.altsetting = &iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

char device_signature[25];
const char *usb_strings[] = {
	"linklayer",
	"CANtact (EMS compatibility firmware)",
	device_signature,
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

uint8_t enabled = 0;
static void bulk_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void) usbd_dev;
	(void) ep;

	ems_submit_data();
}

static void bulk_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	uint8_t buf[64] __attribute__ ((aligned(4)));

	(void)ep;

	uint16_t len = usbd_ep_read_packet(usbd_dev, 0x02, buf, 64);
	ems_process_msg(buf, len);
}

static void set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void) wValue;

	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_ep_setup(usbd_dev, 0x02, USB_ENDPOINT_ATTR_BULK, 64, bulk_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, bulk_tx_cb);
}

static void u32_to_hex(uint32_t val, char *out)
{
	out += 7;
	for(uint8_t i = 0; i < 8; i++) {
		uint8_t nibble = val & 0x0F;

		if(nibble < 10)
			*out = '0' + nibble;
		else
			*out = 'A' + nibble - 10;

		out--;
		val = val >> 4;
	}
}

bool usb_can_push_data(void)
{
	return (*USB_EP_REG(0x82) & USB_EP_TX_STAT) != USB_EP_TX_STAT_VALID;
}

void usb_push_data(uint8_t *buf, uint16_t len)
{
	int _;

	do {
		_ = usbd_ep_write_packet(usbd_dev, 0x82, buf, len);
	} while(_ == 0);
}

void usb_init(void)
{
	u32_to_hex(DESIG_UNIQUE_ID0, device_signature);
	u32_to_hex(DESIG_UNIQUE_ID1, device_signature + 8);
	u32_to_hex(DESIG_UNIQUE_ID2, device_signature + 16);
	device_signature[24] = 0;

	usbd_dev = usbd_init(&st_usbfs_v2_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, set_config);
}

void usb_poll(void)
{
	usbd_poll(usbd_dev);
}
