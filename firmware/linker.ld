MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 16K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 6K
}

INCLUDE ../libopencm3/lib/libopencm3_stm32f0.ld
