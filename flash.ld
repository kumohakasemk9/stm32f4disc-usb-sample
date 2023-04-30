/*
	STM32F4XX arm-none-eabi-g++ Project Template (RAM load)
	Linker script
	(C) 2023 Kumohakase
	You are free to destribute, modify without modifying this section.
	Please consider support me on kofi.com https://ko-fi.com/kumohakase
*/

MEMORY {
	/* Adjusted for STM32F4discovery, You need to modify it
		if your target device has different memory map */
	RAM (wx) : ORIGIN = 0x20000000 , LENGTH = 128K
}

SECTIONS {
	.text : {
		*(.vects); /* Vector Table should be top */
		*(.text*);
		*(.rodata*);
	} > RAM
	.data : {
		*(.data);
		*(.data.*);
		*(.init_array);
	} > RAM
	.bss : {
		__sbss = .; /* bss start address */
		*(.bss);
		*(.bss.*);
		*(COMMON);
		__ebss = .; /* bss end address */
	} > RAM
	__stack = ORIGIN(RAM) + LENGTH(RAM); /* Stack top address */
}
