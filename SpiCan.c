// Pi PICO CAN BUS interface using a PI CAN2 
// P.J.Onion

// Inspired by the Arduino CAN library by Sandeep Mistry 

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19


#define MCP2515Reset    0xC0
#define MCP2515Modify   0x05
#define MCP2515Read     0x03
#define MCP2515Write    0x02

#define MCP2515CANCTRL                0x0F
#define MCP2515CNF3                   0x28
#define MCP2515CNF2                   0x29
#define MCP2515CNF1                   0x2A
#define MCP2515BFPCTRL                0x0c
#define MCP2515TXRTSCTRL              0x0d

#define MCP2515CANINTE                0x2b
#define MCP2515CANINTF                0x2c

#define MCP2515RXBnCTRL(n)            (0x60+(n*0x10))
#define MCP2515TXBnSIDH(n)            (0x31+(n*0x10))
#define MCP2515TXBnSIDL(n)            (0x32+(n*0x10))
#define MCP2515TXBnCTRL(n)            (0x30+(n*0x10))
#define MCP2515TXBnEID8(n)            (0x33+(n*0x10))
#define MCP2515TXBnEID0(n)            (0x34+(n*0x10))
#define MCP2515TXBnDLC(n)             (0x35+(n*0x10))
#define MCP2515TXBnD0(n)              (0x36+(n*0x10))
#define MCP2515RXBnSIDH(n)            (0x61+(n*0x10))
#define MCP2515RXBnSIDL(n)            (0x62+(n*0x10))
#define MCP2515RXBnDLC(n)             (0x65+(n*0x10))
#define MCP2515RXBnD0(n)              (0x66+(n*0x10))

#define MCP2515ConfigMode   0x80

#define BITRXnIE(n)     (0x1<<n)
#define BITTXnIF(n)     (0x4<<n)
#define BITRXnIF(n)     (0x1<<n)
#define BITSRR                   0x10
#define BITRXM0                  0x20
#define BITRXM1                  0x40
#define BITRTR                   0x40

const uint LED_PIN = 14;

void writeMCP2515Register(uint8_t registerNumber,uint8_t registerValue)
{
    uint8_t out;
    gpio_put(PIN_CS, 0);

    out = MCP2515Write;
    spi_write_blocking(SPI_PORT,&out,1);

    out = registerNumber;
    spi_write_blocking(SPI_PORT,&out,1);

    out = registerValue;
    spi_write_blocking(SPI_PORT,&out,1);

    gpio_put(PIN_CS, 1);

}


uint8_t readMCP2515Register(uint8_t registerNumber)
{
    uint8_t out,in;
    
    gpio_put(PIN_CS, 0);

    out =  MCP2515Read;
    spi_write_blocking(SPI_PORT,&out,1);

    out = registerNumber;
    spi_write_blocking(SPI_PORT,&out,1);

    out = 0x0;
    spi_write_read_blocking(SPI_PORT,&out,&in,1);

    gpio_put(PIN_CS, 1);

    return in;
}   

void modifyMCP2515Register(uint8_t registerNumber,uint8_t bitMask, uint8_t registerValue)
{
    uint8_t out;
    gpio_put(PIN_CS, 0);

    out = MCP2515Modify;
    spi_write_blocking(SPI_PORT,&out,1);

    out = registerNumber;
    spi_write_blocking(SPI_PORT,&out,1);

    out = bitMask;
    spi_write_blocking(SPI_PORT,&out,1);
    
    out = registerValue;
    spi_write_blocking(SPI_PORT,&out,1);

    gpio_put(PIN_CS, 1);

}




bool MCP2515Send(uint8_t packetLength,uint8_t *packetData,uint32_t canId)
{
    int n = 0;
    
    writeMCP2515Register(MCP2515TXBnSIDH(n), canId >> 3);
    writeMCP2515Register(MCP2515TXBnSIDL(n), canId << 5);
    writeMCP2515Register(MCP2515TXBnEID8(n), 0x00);
    writeMCP2515Register(MCP2515TXBnEID0(n), 0x00);
    writeMCP2515Register(MCP2515TXBnDLC(n), packetLength); // + BITRTR);  // RTR for test

    for (int i = 0; i < packetLength; i++) {
	writeMCP2515Register(MCP2515TXBnD0(n) + i, *packetData++);
    }

    
    writeMCP2515Register(MCP2515TXBnCTRL(n), 0x08);

    while (readMCP2515Register(MCP2515TXBnCTRL(n)) & 0x08) {
	if (readMCP2515Register(MCP2515TXBnCTRL(n)) & 0x10) {
	    // abort
	    //aborted = true;

	    modifyMCP2515Register(MCP2515CANCTRL, 0x10, 0x10);
	}
    }

    //if (aborted) {
	// clear abort command
//	modifyMCP2515Register(MCP2515CANCTRL, 0x10, 0x00);
    //  }

    modifyMCP2515Register(MCP2515CANINTF, BITTXnIF(n), 0x00);

    return (readMCP2515Register(MCP2515TXBnCTRL(n)) & 0x70) ? 0 : 1; 
}



typedef struct  {
    uint8_t data[8];
    uint8_t dlc;
    bool rtr;
    uint32_t id;
} CANPacket;

CANPacket CANPackets[8];
int CANPacketNextRead = 0;
volatile int CANPacketNextWrite = 0;


void gpioCallback(uint gpio,uint32_t events)
{
 
    CANPacket *packet;
    gpio_put(LED_PIN, 1);  
    if(gpio==15)
    {
	if(readMCP2515Register(MCP2515CANINTF) != 0)
	{
	    uint8_t intf,rxDlc;
	    int n;

	    packet = &CANPackets[CANPacketNextWrite];
	    
	    intf = readMCP2515Register(MCP2515CANINTF);
	    
	    if (intf & BITRXnIF(0))
	    {
		n = 0;
	    }
	    else if (intf & BITRXnIF(1))
	    {
		n = 1;
	    }
	    else
	    {
		return;
	    }

	    packet->id = ((readMCP2515Register(MCP2515RXBnSIDH(n)) << 3) & 0x07f8) |
		  ((readMCP2515Register(MCP2515RXBnSIDL(n)) >> 5) & 0x07);
	    packet->rtr = (readMCP2515Register(MCP2515RXBnSIDL(n)) & BITSRR) ? true : false;
	    rxDlc = packet->dlc = readMCP2515Register(MCP2515RXBnDLC(n)) & 0x0f;


	    if(rxDlc > 0)
	    {
		for(int i=0;i<rxDlc;i++)
		{
		    packet->data[i] = readMCP2515Register(MCP2515RXBnD0(n)+i);
		}
		//printf("Data bytes : ");
		//for(int i=0;i<rxDlc;i++)
		//{
		//    printf("%02X ",data[i] & 0xFF);
		//}
		//printf("\n");
		

	    }

	    modifyMCP2515Register(MCP2515CANINTF, BITRXnIF(n), 0x00);
	    CANPacketNextWrite += 1;
	    CANPacketNextWrite &= 0xF;
	}
    }
    gpio_put(LED_PIN, 0);
}
    



int main()
{
    
    uint8_t mode,data[8];
    
    stdio_init_all();

    gpio_set_irq_enabled_with_callback(15,GPIO_IRQ_EDGE_FALL,true,&gpioCallback);

    
    // SPI initialisation. This example will use SPI at 3MHz as there are some
    // long jumper leads involved..
    spi_init(SPI_PORT, 3000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);


    writeMCP2515Register(MCP2515CANCTRL,MCP2515ConfigMode);
    mode = readMCP2515Register(MCP2515CANCTRL);
    printf("Read back 0x%02x\n",mode);
    
    // Fixed config for 16Mhz clock and 125kHz CAN rate 
    writeMCP2515Register(MCP2515CNF1,0x03);
    writeMCP2515Register(MCP2515CNF2,0xF0);
    writeMCP2515Register(MCP2515CNF3,0x86);
    
    writeMCP2515Register(MCP2515CANINTE,BITRXnIE(0)|BITRXnIE(1));

    writeMCP2515Register(MCP2515BFPCTRL, 0x00);
    writeMCP2515Register(MCP2515TXRTSCTRL, 0x00);
    writeMCP2515Register(MCP2515RXBnCTRL(0), BITRXM1 | BITRXM0);
    writeMCP2515Register(MCP2515RXBnCTRL(1), BITRXM1 | BITRXM0);

    writeMCP2515Register(MCP2515CANCTRL, 0x00);
    mode = readMCP2515Register(MCP2515CANCTRL);
    printf("Read back 0x%02x\n",mode);

    data[0] = 0x55;
    
    MCP2515Send(1,data,1);
    
    puts("Hello, world!");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
	//      gpio_put(LED_PIN, 1);
	//      sleep_ms(100);
        //      gpio_put(LED_PIN, 0);
        //sleep_ms(100);
	if(CANPacketNextRead != CANPacketNextWrite)
	{
	    int dlc;
	    CANPacket *packet  = &CANPackets[CANPacketNextRead];
	    printf("id = 0x%04X\n",packet->id);
	    printf("rtr = %s\n",packet->rtr ? "true":"false");
	    printf("dlc = 0x%02x\n",dlc = packet->dlc);
	    CANPacketNextRead += 1;
	    CANPacketNextRead &= 0xF;

	    for(int i = 0; i < dlc; i++)
	    {
		printf("%02X ",packet->data[i] & 0xFF);
	    }
	    printf("\n");
	}
	tight_loop_contents();
	
    }
    return 0;
}
