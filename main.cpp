/*
	STM32F4XX arm-none-eabi-g++ Project Template (RAM load)
	Main routine
	(C) 2023 Kumohakase
	You are free to destribute, modify without modifying this section.
	Please consider support me on kofi.com https://ko-fi.com/kumohakase
*/

#include <stm32f4xx.h>

void delay(uint32_t);

int main() {
	//Blink GPIOD12 LED
	RCC->AHB1ENR |= 0x8;
	GPIOD->MODER |= 1 << 24;
	while(1) {
		GPIOD->BSRRL = 0x1000;
		delay(1000000);
		GPIOD->BSRRH = 0x1000;
		delay(1000000);
	}
}

void delay(uint32_t i) {
	for(uint32_t c = 0; c < i; c++);
}
