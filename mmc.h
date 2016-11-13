/*#######################################################################################
Connect ARM to MMC/SD 

Copyright (C) 2004 Ulrich Radig
#######################################################################################*/

#ifndef _MMC_H_
 #define _MMC_H_

#include <avr/io.h>	

#define SPI_Mode			1		//1 = Hardware SPI | 0 = Software SPI
//#define SPI_Mode			0

#define MMC_Write          PORTB	//Port an der die MMC/SD-Karte angeschlossen ist also des SPI
#define MMC_Read           PINB
#define MMC_Direction_REG	DDRB	

#define SPI_DI             3		// MISO Port Pin an dem Data Output der MMC/SD-Karte angeschlossen ist
#define SPI_DO             2		// MOSI Port Pin an dem Data Input der MMC/SD-Karte angeschlossen ist
#define SPI_Clock          1		// SCK Port Pin an dem die Clock der MMC/SD-Karte angeschlossen ist (clk)
#define MMC_Chip_Select		0		// SS Port Pin an dem Chip Select der MMC/SD-Karte angeschlossen ist
#define SPI_SS				4		//Nicht Benutz muss aber definiert werden


//Prototypes

volatile uint8_t w=0;
extern unsigned char mmc_read_byte(void);

extern void mmc_write_byte(unsigned char);

extern void mmc_read_block(unsigned char *,unsigned char *,unsigned in);

extern unsigned char mmc_init(void);

extern unsigned char mmc_read_sector (unsigned long,unsigned char *);

extern unsigned char mmc_write_sector (unsigned long,unsigned char *);

extern unsigned char mmc_write_command (unsigned char *);

extern unsigned char mmc_read_csd (unsigned char *);

extern unsigned char mmc_read_cid (unsigned char *);

//set MMC_Chip_Select to high (MMC/SD-Karte Inaktiv)
#define MMC_Disable()   SPI_PORT |= (1<<SPI_SS_PIN) // HI
//set MMC_Chip_Select to low (MMC/SD-Karte Aktiv)
#define MMC_Enable()   SPI_PORT &= ~(1<<SPI_SS_PIN) // LO

#define nop()  __asm__ __volatile__ ("nop" ::)

#endif //_MMC_H_


