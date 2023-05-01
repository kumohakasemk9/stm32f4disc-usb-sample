/*
	STM32F4XX arm-none-eabi-g++ USB Test Project (RAM Load)
	Main routine
	(C) 2023 Kumohakase
	You are free to destribute, modify without modifying this section.
	Please consider support me on kofi.com https://ko-fi.com/kumohakase
*/
#define NAK 0 //Response NAK in Data Stage
#define PACKET_OR_ACK 1 //Response with packet or 0 length ack in Data Stage
#define STALL 2 //Response STALL in next data stage 

#define DESC_DEV 1
#define DESC_CONF 2
#define DESC_STR 3
#define DESC_IF 4
#define DESC_EP 5

#define REQ_SET_ADDR 5
#define REQ_GET_DESC 6
#define REQ_SET_CONF 9

#include <stm32f4xx.h>
#include <cstring>

//USB Standard request type
typedef struct {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} std_req_t;

uint8_t* EP0responsePacket;
uint8_t EP0responsePlan, EP0responsePacketLen;

//Device descriptor
const uint8_t dev_desc[18] = {18, DESC_DEV, 0, 2, 0, 0, 0, 64, 0, 0, 0, 0, 0, 0, 1, 2, 0, 1};

//Configuration Descriptor with interface descs ...
const uint8_t conf_desc[18] = {9, DESC_CONF, 18, 0, 1, 1, 0, 0xc0, 0, 9, DESC_IF, 0, 0, 0, 0, 0, 0, 0};

//String Descriptor 0
const uint8_t string_desc0[4] = {4, DESC_STR, 0, 0};

//String Descriptor 1 :KumoTech
const uint8_t string_desc1[18] = {18, DESC_STR, 'K', 0, 'u', 0, 'm', 0, 'o', 0, 'T', 0, 'e', 0, 'c', 0 ,'h', 0};

//String Descriptor 2 :STM32F4discovery USBTest Project
const uint8_t string_desc2[66] = 
	{66, DESC_STR, 'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0 ,'F', 0, '4', 0, 'D', 0, 'i', 0, 's', 0,
	'c', 0, 'o', 0, 'v', 0, 'e', 0, 'r', 0, 'y', 0 ,' ', 0, 'U', 0, 'S', 0, 'B', 0, 'T', 0,
	'e', 0, 's', 0, 't', 0, ' ', 0, 'P', 0, 'r', 0, 'o', 0, 'j', 0, 'e', 0, 'c', 0 ,'t', 0};

void delay(uint32_t);
void debug(char* = "\r\n");
void debugI(uint32_t, uint8_t = 4, uint8_t = 10);
void readUSBFIFO(uint8_t*, uint16_t);
void writeUSBFIFO(uint8_t*, uint16_t, uint32_t);
void dump(uint8_t*, uint8_t);
void USBreceiveHandler();
void USBsetupHandler(std_req_t);
void setUSBAddress(uint8_t);
void USBsetupDataStage();
void FlushTxFIFOs();

int main() {
	//PLL Config, CPU=72MHz, USB=48MHz In=8MHz
	RCC->CR |= 1 << 16; //External OSC On
	RCC->PLLCFGR = (3 << 24) + (1 << 22) + (72 << 6) + 4; //Q=3 P=0(/2) N=72 M=4
	RCC->CR |= 1 << 24; //PLL On
	while((RCC->CR & (1 << 25)) == 0); //Wait for PLL
	RCC->CFGR = 2; //Switch to PLL
	RCC->CR &= ~1; //Internal OSC Off

	//USART2 Config
	RCC->APB1ENR |= 1 << 17; //USART2 Enable
	RCC->AHB1ENR |= 1; //GPIOA Enable (has USART pin)
	GPIOA->MODER |= 2 << 4; //GPIOA2 is AF
	GPIOA->AFR[0] |= 0x700; //GPIOA2 is AF7
	USART2->BRR = (39 << 4) + 1; //39.0625 (115200 baud @ 72MHz)
	USART2->CR1 = (1 << 13) + 0x8; //Transmitter Enable + USART On
	debug("USART2 initialize completed.\r\n");

	//Blink GPIOD12 LED
	RCC->AHB1ENR |= 0x8;
	GPIOD->MODER |= 1 << 24;

	//OTG FS Config
	RCC->AHB2ENR |= 0x80; //OTG FS En
	GPIOA->MODER |= 0xa << 22; //GPIOD11,12 are AFIO
	GPIOA->AFR[1] |= 0xaa000; //GPIOD11,12 are AF10
	//34.17.1 Core Initialization
	//1. GAHBCFG will be kept to reset value
	//2. Force device mode, TRDT=6, TOCAL=7, HNP and SRP disabled
	OTG_FS->GUSBCFG = (1 << 30) + (6 << 10) + 7;
	//3. Not using interrupt
	//4. Forced to device mode
	//Device Initialization
	OTG_FS->DCFG = 7; //1. FS mode, Error on non zero length status out packet
	//2. Not using interrupt
	OTG_FS->GCCFG = (1 << 21) + (1 << 16); //3. Always assume Vbus=HIGH, USBSIE On


	while(1) {
		if(OTG_FS->GINTSTS & (1 << 12)) {
			OTG_FS->GINTSTS = 1 << 12; //Flag clear
			//Reset Interrupt
			debug("\033[41mUSB RESET\033[0m\r\n");
			//debug("USB Reset Completed.\r\n");
			//debug("OTG_FS_DIEPTXF0=");
			//debugI(OTG_FS->DIEPTXF0, 8, 16); //Reset value was 0x02000200 (different from reference manual!)
			//debug("\r\nOTG_FS_DIEPCTL0=");
			//debugI(OTG_FS->DIEPCTL0, 8, 16);
			//debug();
			setUSBAddress(0);
			FlushTxFIFOs();
		}

		if(OTG_FS->DIEPINT0 & 0x10) {
			//IN EP0 Received IN but no data in TxFIFO
			OTG_FS->DIEPINT0 = 0x10;
			debug("USB EP0 IN Host want more data (ITTXFE)\r\n");
			//OTG_FS->DOEPCTL0 = 1 << 31;
		}

		if(OTG_FS->DIEPINT0 & 0x1) {
			//IN EP0 XFRC (Tx Complete ?)
			OTG_FS->DIEPINT0 = 0x1;
			debug("USB EP0 IN Transmit confirmed (XFRC)\r\n");
		}

		if(OTG_FS->GINTSTS & 0x10) {
			//RXFLVL interrupt (Received packet from host)
			USBreceiveHandler();
		}
	}
}

//Called when data written to RxFIFO
void USBreceiveHandler() {
	//34.17.6 Operational model - Packet read
	uint32_t pinfo = OTG_FS->GRXSTSP; //1. Get some packet information
	uint8_t epnum = pinfo & 0xf; //Target endpoint number
	uint16_t bcnt = (pinfo >> 4) & 0x7ff; //Received packet length
	uint8_t pktsts = (pinfo >> 17) & 0xf; //Packet type
	if(pktsts == 6 && bcnt == 8 && epnum == 0) {
		//4b. Setup packet pattern
		uint8_t rbuf[8];
		readUSBFIFO(rbuf, 8); //Readout RxFIFO
		//Decode read packet into usb standard request form
		std_req_t r;
		memcpy(&r ,rbuf ,8);
		debug("\033[44mSETUP\033[0m ");
		dump(rbuf, 8);
		USBsetupHandler(r);
	} else if(pktsts == 4) {
		//4c. Setup stage done pattern
		//debug("SETUP Data Stage FIFO free space:");
		//debugI(OTG_FS->DTXFSTS0);
		//debug(" Plan=");
		//debugI(EP0responsePlan, 1);
		//debug(" Len=");
		//debugI(EP0responsePacketLen);
		//debug();
		USBsetupDataStage();
	} else {
		debug("RXFLVL - Pktsts = ");
		debugI(pktsts,1);
		debug("\r\n");
	}
}

//Called when SETUP received
void USBsetupHandler(std_req_t r) {
	if(r.bmRequestType == 0x80 && r.bRequest == REQ_GET_DESC && r.wIndex == 0) {
		//GET_DESCRIPTOR(6)
		uint8_t rDescType = (r.wValue >> 8) & 0xff; //wValue[15:8] required descriptor type
		uint8_t rDescIndex = r.wValue & 0xff; //wValue[7:0] required descriptor index
		EP0responsePlan = PACKET_OR_ACK; //Send packet in data stage
		if((rDescType == DESC_DEV || rDescType == DESC_CONF) && rDescIndex == 0) {
			uint8_t *s[] = {(uint8_t*)dev_desc, (uint8_t*)conf_desc};
			EP0responsePacket = s[rDescType - 1]; //Choose appropriate packet
		} else if(rDescType == DESC_STR && rDescIndex < 3) {
			uint8_t *s[] = {(uint8_t*)string_desc0, (uint8_t*)string_desc1, 
											(uint8_t*)string_desc2 };
			EP0responsePacket = s[rDescIndex];
		} else {
			//Bad descriptor type or too big descriptor index.
			EP0responsePlan = STALL;
		}
		if(EP0responsePlan != STALL) {
			//Choose appropriate size, most descriptor has length in offset 0
			EP0responsePacketLen = EP0responsePacket[0];
			if(rDescType == DESC_CONF) {
				//Special case: CONFIG descriptor
				EP0responsePacketLen = EP0responsePacket[2];
			}
			//If packet is larger than wLength, turncate sending size
			if(EP0responsePacketLen > r.wLength) {
				EP0responsePacketLen = r.wLength;
			}
		}
	} else if(r.bmRequestType == 0 && r.bRequest == REQ_SET_ADDR && r.wIndex == 0 && r.wLength == 0 && r.wValue < 128) {
		//SET_ADDRESS(5)
		setUSBAddress(r.wValue);
		EP0responsePlan = PACKET_OR_ACK; //Send ACK in data stage
		EP0responsePacketLen = 0; //Send 0 length
	} else if(r.bmRequestType == 0 && r.bRequest == REQ_SET_CONF && r.wValue == 1 && r.wIndex == 0 && r.wLength == 0) {
		//SET_CONFIGURATION(9) but only config 1 exists so wValue should be 0
		EP0responsePlan = PACKET_OR_ACK;
		EP0responsePacketLen = 0;
	} else {
		//malformed packet
		EP0responsePlan = STALL; //Send stall in data stage
	}
	//debug("OTG_FS_DIEPCTL0=");
	//debugI(OTG_FS->DIEPCTL0, 8, 16);
	//debug();
}

//USB SETUP Data stage Handler. Tx will done in here.
void USBsetupDataStage() {
	uint8_t NoFreeSpace = 0;
	//34.17.6 - IN - 2 : Check for TxFIFO free space
	if(EP0responsePacketLen != 0 && OTG_FS->DTXFSTS0 < EP0responsePacketLen / 4) {
		debug("\033[41mEP0 IN TxFIFO Full\033[0m\r\n");
		NoFreeSpace = 1;
	}
	if(EP0responsePlan == STALL || NoFreeSpace) {
			//If STALL requested or TxFIFO has not enough free space
			debug("\033[41mEP0 IN STALL Sent\033[0m\r\n");
			//34.17.6 Operational Model - In data transfers - Stalling
			OTG_FS->DIEPCTL0 |= 1 << 21; //1. Stall bit=1, auto clear on next setup
			FlushTxFIFOs(); //4. Flush TxFIFOs
		} else if (EP0responsePlan == PACKET_OR_ACK) {
			//If packet send or ACK requested
			//34.17.6 Operational Model - In transfer - Generic non-perodic in transfers
			// 1. Write packet length and count
			if(EP0responsePacketLen == 0) {
				debug("\033[42mEP0 IN ACK Sent\033[0m\r\n");
				OTG_FS->DIEPTSIZ0 = 1 << 19; //For Zero length packet
			} else {
				//ApplicationRequirements: Calculate desired value from packet lenght
				uint8_t pcnt = (EP0responsePacketLen / 64) + 1;
				OTG_FS->DIEPTSIZ0 = (pcnt << 19) + EP0responsePacketLen;
				debug("\033[42mEP0 IN Packet sent\033[0m\r\n");
				//debug("Sending non 0 length packet: ");
				//debugI(OTG_FS->DIEPTSIZ0, 8, 16);
				//debug();
			}
			OTG_FS->DIEPCTL0 |= (1 << 26) + (1 << 31); //2. Get ready for writing FIFO
			OTG_FS->DOEPCTL0 |= 1 << 26; //Finally I figured out, I must set SNAK of OTG_FS->DOEPCTL0, too.
			if(EP0responsePacketLen != 0) {
				writeUSBFIFO(EP0responsePacket, EP0responsePacketLen, OTG_FS_DFIFO0_BASE + 0x200); //Write data after setting those registers
			}
		}
		EP0responsePlan = NAK;
}

void delay(uint32_t i) {	
	for(uint32_t c = 0; c < i; c++);
}

//Print string d to USART2
void debug(char* d) {
	uint32_t i = 0;
	while(d[i]) {
		USART2->DR = d[i];
		while((USART2->SR & 0x80) == 0); //Wait for send finish
		i++;
	}
}

//Print number d  to USART2 in size of len as base(2:BIN, 10:DEC, 16:HEX)
void debugI(uint32_t d, uint8_t len, uint8_t base) {
	char buf[len + 1];
	uint32_t t = d;
	buf[len] = '\0';
	for(uint8_t i = 0; i < len; i++) {
		uint8_t e = t % base; //get last digit
		uint8_t c = len - 1 - i; //letter position for current digit
		if(e < 10) {
			buf[c] = e + '0'; //Convert integer 0 - 9 to ascii '0' - '9'
		} else {
			buf[c] = e - 10 + 'a'; //Convert integer 10 - 15 to 'a' - 'f'
		}
		if(c == 0 && e > base) {
			//Reached max digit, but need more digit to express number
			memset(buf, 'E', len - 1); //Indicate overflow by filling str with 'E'
		}
		t = t / base; //pop out last digit;
	}
	debug(buf);
}

//Read data from USB OTGFS RxFIFO and store into dst for len bytes
void readUSBFIFO(uint8_t *dst, uint16_t len) {
	//USB RxFIFO is shared, all data (regardless ep) will be in the FIFO
	//Start address is OTG_FS_DFIFO0_BASE (fixed), 32bit access required
	uint16_t i = 0;
	while(i < len) {
		//Convert uint32_t to uint8_t 0xaabbccdd -> 0xdd 0xcc 0xbb 0xaa
		uint32_t e = *OTG_FS_DFIFO0;
		for(uint8_t j = 0; j < 4; j++) {
			if(i + j < len) {
				dst[i + j] = e & 0xff;
			} else {
				break;
			}
			e = e >> 8;
		}
		i = i + 4;
	}
}

//like hexdump, dump data d for len bytes.
void dump(uint8_t *d, uint8_t len) {
	for(uint8_t i = 0; i < len; i++) {
		debugI(d[i], 2, 16);
		debug(" ");
		if(i % 16 == 15) {
			debug();
		}
	}
	if(len % 16 != 0) {
		debug();
	}
}

//Write to USB OTG_FS TxFIFO (addressed by dst) from s for len bytes
void writeUSBFIFO(uint8_t *s, uint16_t len, uint32_t dst) {
	/* TXFIFO is not shared, there is more than one TXFIFO areas,
		and it have to be assigned to endpoint by setting DIEPCTLx[25:22].
		TXFIFO start address can be changed through DIEPTXFx[15:0]
		and not fixed at all. (Fig. 395 confused me, Fig. 392 looks like telling truth.)
		Also DIEPTXFx are writable. so reference manual; lied to me.
		32bit access is required.
	*/
	uint32_t* d = (uint32_t*)dst;
	uint16_t i = 0;
	while(i < len) {
		//Convert uint8_t[4] to uint32_t: 0xaa 0xbb 0xcc 0xdd -> 0xddccbbaa
		uint32_t e = 0;
		for(uint8_t j = 0;j < 4; j++) {
			if(i + j < len) {
				e += s[i + j] << (8 * j); 
			} else {
				break;
			}
		}
		*d = e;
		i = i + 4;
	}
}

//Sett USB Address to addr by modifying OTG_FS_GCFG[10:4]
void setUSBAddress(uint8_t addr) {
	OTG_FS->DCFG &= ~0x7f0; //Clear bit4 - 10
	OTG_FS->DCFG |= addr << 4; //Set bit4 - 10 to addr
}

//Flush all TxFIFO
void FlushTxFIFOs() {
	OTG_FS->GRSTCTL = (1 << 10) + 0x20; //TXFNUM=0b10000, TXFFLUSH=1
}
