/*
 * 	Doku, siehe http://www.mikrocontroller.net/articles/AVR_FAT32
 *  Neuste Version: http://www.mikrocontroller.net/svnbrowser/avr-fat32/
 *	Autor: Daniel R.
 */

#include "mmc_config.h"
#include "mmc.h"

// Definitions for MMC/SDC command 
#define CMD0	(0)			// GO_IDLE_STATE 
#define CMD1	(1)			// SEND_OP_COND (MMC) 
#define	ACMD41	(41)		// SEND_OP_COND (SDC)
#define CMD8	(8)			// SEND_IF_COND 
#define CMD9	(9)			// SEND_CSD 
#define CMD10	(10)		// SEND_CID 
#define CMD12	(12)		// STOP_TRANSMISSION 
#define ACMD13	(0x80+13)	// SD_STATUS (SDC) 
#define CMD16	(16)		// SET_BLOCKLEN 
#define CMD17	(17)		// READ_SINGLE_BLOCK 
#define CMD18	(18)		// READ_MULTIPLE_BLOCK 
#define CMD23	(23)		// SET_BLOCK_COUNT (MMC) 
#define	ACMD23	(0x80+23)	// SET_WR_BLK_ERASE_COUNT (SDC) 
#define CMD24	(24)		// WRITE_BLOCK 
#define CMD25	(25)		// WRITE_MULTIPLE_BLOCK 
#define CMD55	(55)		// APP_CMD 
#define CMD58	(58)		// READ_OCR 

// Card type flags (CardType) 
#define CT_MMC		0x01			// MMC ver 3 
#define CT_SD1		0x02			// SD ver 1 
#define CT_SD2		0x04			// SD ver 2 
#define CT_SDC		(CT_SD1|CT_SD2)	// SD 
#define CT_BLOCK	0x08			// Block addressing 

// **********************************************************************************************************************************
// funktionsprototypen von funktionen die nur in dieser datei benutzt werden !

static unsigned char 	mmc_enable(void);
static void             mmc_disable(void);
static unsigned char 	mmc_wait_ready (void);
static unsigned char 	mmc_send_cmd (	unsigned char cmd,	unsigned long arg);

// beginn -> hardware abhaengiger teil !
#define MMC_CS_LOW 		MMC_Write &= ~(1<<SPI_SS)		// Set pin B2 to 0
#define MMC_CS_HIGH		MMC_Write |= (1<<SPI_SS)		// Set pin B2 to 1

static void 			spi_init(void);
#if (MMC_MAX_SPEED==TRUE)
	static void 			spi_maxSpeed(void);
#endif
static void 			spi_write_byte(unsigned char byte);
static unsigned char 	spi_read_byte(void);


// ** CHaN ***************************************************************************
BYTE Buff[4096];	/* Working buffer */
static volatile
DSTATUS Stat = STA_NOINIT;	/* Disk status */

static volatile
char Timer1, Timer2;	/* 100Hz decrement timer */

static
BYTE CardType;			/* Card type flags (b0:MMC, b1:SDv1, b2:SDv2, b3:Block addressing) */


// *****************************************************************************
static void spi_init(void){
 	
	// port configuration der mmc/sd/sdhc karte
   /*
	MMC_Direction_REG &=~(1<<SPI_MI);         // miso auf input
	MMC_Direction_REG |= (1<<SPI_CLK);      	// clock auf output
	MMC_Direction_REG |= (1<<SPI_MO);         // mosi auf output
	MMC_Direction_REG |= (1<<SPI_SS);			// chip select auf output
*/
	// hardware spi: bus clock = idle low, spi clock / 128 , spi master mode
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);

   MMC_Write |= (1<<SPI_SS);       	// chip selet auf high, karte anwaehlen
}



#if (MMC_MAX_SPEED==TRUE)
// *****************************************************************************
static void spi_maxSpeed(){
	
	//SPI Bus auf max Geschwindigkeit
	SPCR &= ~((1<<SPR0) | (1<<SPR1));
	SPSR |= (1<<SPI2X);
}
#endif

// *****************************************************************************
static void spi_write_byte(unsigned char byte){
	

	#if (MMC_SOFT_SPI==TRUE)
		unsigned char a;
	#endif

	// mmc/sd in hardware spi
	#if (MMC_SOFT_SPI==FALSE)
		SPDR = byte;    						//Sendet ein Byte
		loop_until_bit_is_set(SPSR,SPIF);

	// mmc/sd in software spi
	#else
		for (a=8; a>0; a--){					//das Byte wird Bitweise nacheinander Gesendet MSB First
			if (bit_is_set(byte,(a-1))>0){		//Ist Bit a in Byte gesetzt
				MMC_Write |= (1<<SPI_MO); 	//Set Output High
			}
			else{
				MMC_Write &= ~(1<<SPI_MO); 	//Set Output Low
			}
			MMC_Write |= (1<<SPI_CLK); 		//setzt Clock Impuls wieder auf (High)
			MMC_Write &= ~(1<<SPI_CLK);		//erzeugt ein Clock Impuls (LOW)
		}
		MMC_Write |= (1<<SPI_MO);				//setzt Output wieder auf High
	#endif
}


// *****************************************************************************
static unsigned char spi_read_byte(void){
	
	// mmc/sd in hardware spi
	#if (MMC_SOFT_SPI==FALSE)
	  SPDR = 0xff;
	  loop_until_bit_is_set(SPSR,SPIF);
	  return (SPDR);

	// mmc/sd in software spi
	#else
		unsigned char Byte=0;
	    unsigned char a;
		for (a=8; a>0; a--){							//das Byte wird Bitweise nacheinander Empangen MSB First
			MMC_Write |=(1<<SPI_CLK);					//setzt Clock Impuls wieder auf (High)
			if (bit_is_set(MMC_Read,SPI_MI) > 0){ 	//Lesen des Pegels von MMC_MISO
				Byte |= (1<<(a-1));
			}
			else{
				Byte &=~(1<<(a-1));
			}
			MMC_Write &=~(1<<SPI_CLK); 				//erzeugt ein Clock Impuls (Low)
		}
		return (Byte);
	#endif
}



// ende <- hardware abhaengiger teil !








// **********************************************************************************************************************************
unsigned char mmc_init (void){

	unsigned char cmd, ty, ocr[4];
	unsigned short n, j;

	spi_init();
	mmc_disable();

	for (n = 100; n; n--) spi_read_byte();    					// 80+ dummy clocks

	ty = 0;
	j=100;
	do
   {
      
      if (mmc_send_cmd(CMD0, 0) == 1)
      {      					// Enter Idle state
         j=0;
         TimingDelay = 100;            						// Initialization timeout of 1000 msec
         
         if (mmc_send_cmd(CMD8, 0x1AA) == 1)
         {  				// SDv2?
            lcd_gotoxy(0,1);
            lcd_putc('s');
            for (n = 0; n < 4; n++)
            {
               ocr[n] = spi_read_byte();    				// Get trailing return value of R7 resp
               //lcd_puthex(ocr[n]);
            }
            lcd_putc(' ');
            if (ocr[2] == 0x01 && ocr[3] == 0xAA)// The card can work at vdd range of 2.7-3.6V
            {
               lcd_putc('b');
               uint8_t c55 = 0;
               uint8_t c41 = 0;
               while (TimingDelay  && mmc_send_cmd(ACMD41, 1UL << 30));
                  lcd_putc('y');
               {  						// Wait for leaving idle state (ACMD41 with HCS bit)
                  c55 = mmc_send_cmd(CMD55, 0);
                  //c41 = mmc_send_cmd(ACMD41, 1UL << 30);
                  
                  //if (!c41)
                  //if(!mmc_send_cmd(ACMD41, 1UL << 30))
                  //   break;
               }
               
               
               lcd_putc('c');
               lcd_puthex(c55);
               //lcd_puthex(c41);
               lcd_putc(' ');
               uint8_t c58 = 13;
               TimingDelay=0xFFFF;
               uint16_t cnt=0;
               while(TimingDelay)
               {
                  cnt++;
                  c58 = mmc_send_cmd(CMD58, 0);
                  //lcd_putc('*');
                  //lcd_puthex(c58);
                  //lcd_putc('*');
                  //TimingDelay=100;
            //      if (mmc_send_cmd(CMD58, 0) == 0x00)
                  if (c58 == 0x00)
                  {    // Check CCS bit in the OCR
                     //lcd_puthex(c58);
                     for (n = 0; n < 4; n++)
                     {
                        ocr[n] = spi_read_byte();
                        //lcd_putc('s');
                     }
                     ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;  // SDv2
                     //lcd_putc('B');
                     break;
                  }
               }
               lcd_putc('x');
               lcd_puthex(c58);
               
               lcd_putc('*');
               lcd_putint(cnt);
               lcd_putc('*');
            }
         }
         else
         {        									// SDv1 or MMCv3
            if (mmc_send_cmd(ACMD41, 0) <= 1)
            {
               lcd_putc('d');
               ty = CT_SD1;
               cmd = ACMD41;  								// SDv1
            }
            else
            {
               lcd_putc('e');
               ty = CT_MMC;
               cmd = CMD1;    								// MMCv3
            }
            while (TimingDelay && mmc_send_cmd(cmd, 0));    // Wait for leaving idle state
         }
         lcd_putc('f');
         if(ty != (CT_SD2 | CT_BLOCK))
         {
            while(TimingDelay && (mmc_send_cmd(CMD16, 512) != 0));
         }
         if(!TimingDelay)
         {
            ty = 0;
            lcd_putc('g');
         }
      }
      else
      {
         j--;
      }
   }while(j>0);
   lcd_putc('h');
	fat.card_type = ty;
	mmc_disable();

	if( fat.card_type == 0 )
   {
      lcd_putc('i');
		return FALSE;
	}
	#if (MMC_MAX_SPEED==TRUE)
		spi_maxSpeed();
	#endif
   lcd_putc('k');
	return TRUE;
}

// **********************************************************************************************************************************
static unsigned char mmc_send_cmd (	unsigned char cmd,	unsigned long arg){
	
	unsigned char n, res;
	// Select the card and wait for ready 
	mmc_disable();
	if ( FALSE == mmc_enable() )
   {
		return 0xFF;
	}
	// Send command packet 
	spi_write_byte(0x40 | cmd);						// Start + Command index 
	spi_write_byte( (unsigned char)(arg >> 24) );	// Argument[31..24]
	spi_write_byte( (unsigned char)(arg >> 16) );	// Argument[23..16]
	spi_write_byte( (unsigned char)(arg >> 8) );	// Argument[15..8]
	spi_write_byte( (unsigned char)arg );			// Argument[7..0]
	n = 0x01;										// Dummy CRC + Stop 
	if (cmd == CMD0) n = 0x95;						// Valid CRC for CMD0(0) 
	if (cmd == CMD8) n = 0x87;						// Valid CRC for CMD8(0x1AA) 
	spi_write_byte(n);

	// Receive command response 
	if (cmd == CMD12) spi_read_byte();				// Skip a stuff byte when stop reading 
	n = 10;											// Wait for a valid response in timeout of 10 attempts 
	do
		res = spi_read_byte();
	while ( (res & 0x80) && --n );

	return res;										// Return with the response value 
}





// **********************************************************************************************************************************
static unsigned char mmc_enable(){
      
   MMC_CS_LOW;
   if( !mmc_wait_ready() ){
   	  mmc_disable();
	  return FALSE;
   }

   return TRUE;
}

// **********************************************************************************************************************************
static void mmc_disable(){

   MMC_CS_HIGH;   
   spi_read_byte();
}


#if (MMC_MULTI_BLOCK==TRUE && MMC_OVER_WRITE == FALSE)
// **********************************************************************************************************************************
// stopt multiblock lesen
// **********************************************************************************************************************************
unsigned char mmc_multi_block_stop_read (void){

	unsigned char cmd[] = {0x40+12,0x00,0x00,0x00,0x00,0xFF};	// CMD12 (stop_transmission), response R1b (kein fehler, dann 0)
	unsigned char response;

	response = mmc_write_command (cmd);		// r1 antwort auf cmd12

	response = mmc_read_byte();				// dummy byte nach cmd12

	mmc_disable();
	return response;
}


// **********************************************************************************************************************************
// stop multiblock schreiben
// **********************************************************************************************************************************
unsigned char mmc_multi_block_stop_write (void){

	unsigned char cmd[] = {0x40+13,0x00,0x00,0x00,0x00,0xFF};	// CMD13 (send_status), response R2
	unsigned char response;

	mmc_write_byte(0xFD);					// stop token

	mmc_wait_ready();

	response=mmc_write_command (cmd);		// cmd13, alles ok?

	mmc_wait_ready();

	mmc_disable();
	return response;
}


// **********************************************************************************************************************************
// starten von multi block read. ab sektor addr wird der reihe nach gelesen. also addr++ usw...
// **********************************************************************************************************************************
unsigned char mmc_multi_block_start_read (unsigned long int addr){

	unsigned char cmd[] = {0x40+18,0x00,0x00,0x00,0x00,0xFF};	// CMD18 (read_multiple_block), response R1
	unsigned char response;

	mmc_enable();

	// addressiertung bei mmc und sd (standart < 2.0) in bytes, also muss sektor auf byte adresse umgerechnet werden.
	// sd standart > 2.0, adressierung in sektoren, also 512 byte bloecke
	if(card_type==0) addr = addr << 9; //addr = addr * 512, nur wenn mmc/sd karte vorliegt

	cmd[1] = ((addr & 0xFF000000) >>24 );
	cmd[2] = ((addr & 0x00FF0000) >>16 );
	cmd[3] = ((addr & 0x0000FF00) >>8 );
	cmd[4] = (addr &  0x000000FF);

	mmc_wait_ready ();

	response=mmc_write_command (cmd);		// commando senden und response speichern

	while (mmc_read_byte() != 0xFE){		// warten auf start byte
		nop();
	};

	return response;
}


// **********************************************************************************************************************************
//multi block lesen von sektoren. bei aufruf wird immer ein sektor gelesen und immer der reihe nach
// **********************************************************************************************************************************
void mmc_multi_block_read_sector (unsigned char *Buffer){

	unsigned short a; 							// einfacher zähler fuer bytes eines sektors

	// mmc/sd in hardware spi, block lesen
	#if (MMC_SOFT_SPI==FALSE)
	   unsigned char tmp; 						// hilfs variable zur optimierung
	   a=512;
	   SPDR = 0xff;								// dummy byte
		do{										// 512er block lesen
			loop_until_bit_is_set(SPSR,SPIF);
			tmp=SPDR;
			SPDR = 0xff;						// dummy byte
			*Buffer=tmp;
			Buffer++;
		}while(--a);

	// mmc/sd/sdhc in software spi, block lesen
	#else
		a=512;
		do{
			*Buffer++ = mmc_read_byte();
		}while(--a);
	#endif

	mmc_read_byte();						// crc byte
	mmc_read_byte();						// crc byte

	while (mmc_read_byte() != 0xFE){		// warten auf start byte 0xFE, damit fängt jede datenuebertragung an...
		nop();
		}
}


// **********************************************************************************************************************************
// starten von multi block write. ab sektor addr wird der reihe nach geschrieben. also addr++ usw...
// **********************************************************************************************************************************
unsigned char mmc_multi_block_start_write (unsigned long int addr){

	unsigned char cmd[] = {0x40+25,0x00,0x00,0x00,0x00,0xFF};	// CMD25 (write_multiple_block),response R1
	unsigned char response;

	mmc_enable();

	// addressiertung bei mmc und sd (standart < 2.0) in bytes, also muss sektor auf byte adresse umgerechnet werden.
	// sd standart > 2.0, adressierung in sektoren, also 512 byte bloecke
	if(card_type==0) addr = addr << 9; //addr = addr * 512

	cmd[1] = ((addr & 0xFF000000) >>24 );
	cmd[2] = ((addr & 0x00FF0000) >>16 );
	cmd[3] = ((addr & 0x0000FF00) >>8 );
	cmd[4] = (addr &  0x000000FF);

	response=mmc_write_command (cmd);		// commando senden und response speichern

	return response;
}


// **********************************************************************************************************************************
//multi block schreiben von sektoren. bei aufruf wird immer ein sektor geschrieben immer der reihe nach
// **********************************************************************************************************************************
unsigned char mmc_multi_block_write_sector (unsigned char *Buffer){

	unsigned short a;			// einfacher zaehler fuer bytes eines sektors
	unsigned char response;

	mmc_write_byte(0xFC);

	// mmc/sd in hardware spi, block schreiben
	#if (MMC_SOFT_SPI==FALSE)
		unsigned char tmp;			// hilfs variable zur optimierung
		a=512;				// do while konstrukt weils schneller geht
		tmp=*Buffer;			// holt neues byte aus ram in register
		Buffer++;			// zeigt auf naechstes byte
		do{
			SPDR = tmp;    //Sendet ein Byte
			tmp=*Buffer;	// holt schonmal neues aus ram in register
			Buffer++;
			loop_until_bit_is_set(SPSR,SPIF);
		}while(--a);

	// mmc/sd in software spi, block schreiben
	#else
		a=512;
		do{
			mmc_write_byte(*Buffer++);
		}while(--a);
	#endif

	//CRC-Bytes schreiben
	mmc_write_byte(0xFF); //Schreibt Dummy CRC
	mmc_write_byte(0xFF); //CRC Code wird nicht benutzt

	response=mmc_read_byte();

	mmc_wait_ready();

	if ((response&0x1F) == 0x05 ){			// daten von der karte angenommen, alles ok.
		return TRUE;
	}

	return FALSE;							// daten nicht angenommen... hiernach muss stop token gesendet werden !

}



#endif

// **********************************************************************************************************************************
// wartet darauf, dass die mmc karte in idle geht
// **********************************************************************************************************************************
static unsigned char mmc_wait_ready (void){

	TimingDelay = 50;

	do{
		if(	 spi_read_byte() == 0xFF ) return TRUE;
	}while ( TimingDelay );

	return FALSE;
}




// **********************************************************************************************************************************
// Routine zum schreiben eines Blocks(512Byte) auf die MMC/SD-Karte
// **********************************************************************************************************************************
unsigned char mmc_write_sector (unsigned long addr,unsigned char *buffer){

	unsigned char resp;
	unsigned char retrys;
	unsigned short count;
   	
	if ( !(fat.card_type & CT_BLOCK) ){
		addr *= 512;				// Convert to byte address if needed 
	}
	
	if ( mmc_send_cmd(CMD24, addr) != 0){ 	// enables card		
		return FALSE;
	}

	if ( FALSE == mmc_wait_ready() ){		
		return FALSE;
	}

	spi_write_byte(0xFE);			// Xmit data token 
	
	count = 512;
	do {							// Xmit the 512 byte data block to MMC 
		spi_write_byte(*buffer++);		
	} while (--count);
	
	spi_write_byte(0xFF);			// CRC (Dummy) 
	spi_write_byte(0xFF);
	
	retrys = 20;			
	do{
		resp = spi_read_byte();		// Reveive data response, 20 retrys if not acepted
	}while( (resp & 0x1F) != 0x05 && --retrys);
	
	if ( retrys == 0){				// If not accepted, return with error 		
		return FALSE;
	}
	
	mmc_disable();

	return TRUE;
}


// **********************************************************************************************************************************
// Routine zum lesen eines Blocks(512Byte) von der MMC/SD-Karte
// **********************************************************************************************************************************
unsigned char mmc_read_sector (unsigned long addr,unsigned char *buffer){

	unsigned char token;
	unsigned short count;
	
	if ( !(fat.card_type & CT_BLOCK) ) addr *= 512;	// Convert to byte address if needed

	if ( mmc_send_cmd(CMD17, addr) != 0 ){
		return FALSE;	
	}

	TimingDelay = 20;
	do {							// Wait for data packet in timeout of 200ms 
		token = spi_read_byte();
	} while ( (token == 0xFF) && TimingDelay );
	
	if(token != 0xFE){
		return FALSE;				// If not valid data token, retutn with error 
	}

	count = 512;
	do {							// Receive the data block into buffer 
		*buffer++ = spi_read_byte();
	} while (--count);

	spi_read_byte();				// Discard CRC 
	spi_read_byte();

	mmc_disable();

	return TRUE;					// Return with success 
}



// **********************************************************************************************************************************
#if(MMC_STATUS_INFO == TRUE)
unsigned char mmc_present(void) {
	  return get_pin_present() == 0x00;
  }
#endif


// **********************************************************************************************************************************
#if(MMC_STATUS_INFO == TRUE && MMC_WRITE == TRUE)
unsigned char mmc_protected(void) {
	  return get_pin_protected() != 0x00;
  }
#endif



