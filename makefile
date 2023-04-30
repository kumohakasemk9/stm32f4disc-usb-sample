#	STM32F4XX arm-none-eabi-g++ Project Template (RAM load)
#	Makefile
#	(C) 2023 Kumohakase
#	You are free to destribute, modify without modifying this section.
#	Please consider support me on kofi.com https://ko-fi.com/kumohakase

CXX=arm-none-eabi-g++
OD=arm-none-eabi-objdump
CPU=-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mthumb
#You need to set CMSIS path to correct CMSIS path, if they are in different path
#You need to change -DSTM32F40XX to correct one if you have different target.
CXXFLAGS=$(CPU) -DSTM32F40XX -I . -I /home/owner/harddisk_home/programs/CMSIS_5-5.7.0/CMSIS/Core/Include/ -g3 -fno-exceptions
OBJS=main.o start.o

all: $(OBJS)
	$(CXX) $(CPU) -T flash.ld -nostartfiles $^
	$(OD) -hdS > report
	$(OD) -h

clean:
	rm -f $(OBJS) a.out report
