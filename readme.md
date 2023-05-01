STM32F4discovery USB sample project  (RAM load)
=====   
This is STM32F4discovery USB sample project 

How to use
=====
First you need some missing files. stm32f4xx.h and CMSIS. After downloading,   
Please modify makefile for correct include path.  
You have to make include path pointing correct dir to CMSIS includes and stm32f4xx.h.  
And please modify stm32f4xx.h or add preprocessor (-Dxxx) for   
make stm32f4xx.h know target device type.   
If no modification on makefile or stm32f4xx.h,   
This project file will target STM32F4Discovery (STM32F407) for default  
And program in main.c and pls make. You will get   
1. a.out - final binary output
2. report - Disassembly and section info
   
Please use arm-none-eabi-gdb to load it to your target board.
report will contain some useful info. You can watch disassembly and see what's going on   
in case of your program not responding.   

Function
=====
If you connect usb cable in OTG_FS (STM32F4's microB connector),    
This will add dummy device.   

License
=====
You are free to destribute, modify without modifying this section.  
Please consider support me on kofi.com https://ko-fi.com/kumohakase  
Licensed under Creative commons CC-BY https://creativecommons.org/licenses/by/4.0/

