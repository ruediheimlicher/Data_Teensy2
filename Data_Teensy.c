//
//  RC_PPM.c
//
//
//  Created by Sysadmin on 20.07.13
//  Copyright Ruedi Heimlicher 2013. All rights reserved.
//


#include <string.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <math.h>
#include <stdlib.h>


#include "lcd.c"
//#include "adc.c"
#include "version.c"
#include "usb_rawhid.c"
#include "defines.h"

//#include "spi.c"
#include "spi_adc.c"

//#include "spi_slave.c"
#include "soft_SPI.c"

#include "ds18x20.c"

// MMC
#include "diskio.c"
#include "ff.c"

#include "mmc.c"

// USB
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define LOOPDELAY 5

#define SERVOMAX  4400
#define SERVOMIN  1400


#define USB_DATENBREITE 32

#define CODE_OFFSET  4
#define ROTARY_OFFSET  10

/*
const char wertearray[] PROGMEM = {TASTE1,TASTE2,TASTE3,TASTE4,TASTE5,TASTE6,TASTE7,TASTE8,TASTE9,TASTE_L,TASTE0,TASTE_R};
*/

 uint16_t key_state;				// debounced key state:
// bit = 1: key pressed
uint16_t key_press;				// key press detect
volatile uint16_t tscounter =0;

extern volatile uint8_t isrcontrol;

volatile uint8_t do_output=0;
static volatile uint8_t testbuffer[USB_DATENBREITE]={};


static volatile uint8_t buffer[USB_DATENBREITE]={};


static volatile uint8_t recvbuffer[USB_DATENBREITE]={};

static volatile uint8_t sendbuffer[USB_DATENBREITE]={};

volatile uint8_t outbuffer[USB_DATENBREITE]={};
volatile uint8_t inbuffer[USB_DATENBREITE]={};

static volatile uint8_t kontrollbuffer[USB_DATENBREITE]={};

static volatile uint8_t eeprombuffer[USB_DATENBREITE]={};

#define TIMER0_STARTWERT	0x40

#define EEPROM_STARTADRESSE   0x7FF

volatile uint8_t timer0startwert=TIMER0_STARTWERT;

volatile uint8_t rxbuffer[USB_DATENBREITE];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
volatile uint8_t txbuffer[USB_DATENBREITE];


void delay_ms(unsigned int ms);

static volatile uint8_t             displaystatus=0x00; // Tasks fuer Display
 volatile uint8_t                   eepromsavestatus = 0;


static volatile uint16_t            displaycounter=0;

volatile uint8_t                    in_taskcounter=0;
volatile uint8_t                    out_taskcounter=0;



//static volatile uint8_t             substatus=0x00; // Tasks fuer Sub

static volatile uint8_t             usbstatus=0x00;

volatile uint8_t                    spistatus=0x00; // was ist zu tun infolge spi

//static volatile uint8_t             eepromstatus=0x00;
static volatile uint8_t             potstatus=0x00; // Bit 7 gesetzt, Mittelwerte setzen
static volatile uint8_t             impulscounter=0x00;

static volatile uint8_t             masterstatus = 0;

static volatile uint8_t             tastaturstatus = 0;

volatile uint8_t status=0;

volatile uint8_t                    PWM=0;
static volatile uint8_t             pwmposition=0;
static volatile uint8_t             pwmdivider=0;


static volatile uint8_t spi_rxbuffer[SPI_BUFSIZE];
static volatile uint8_t spi_txbuffer[SPI_BUFSIZE];
static volatile uint8_t spi_rxdata=0;

static volatile uint8_t inindex=0;

volatile char SPI_data='0';
volatile char SPI_dataArray[SPI_BUFSIZE];
volatile uint16_t Pot_Array[SPI_BUFSIZE];

volatile uint16_t Mitte_Array[8];

//volatile uint8_t Level_Array[8]; // Levels fuer Kanaele, 1 byte pro kanal
//volatile uint8_t Expo_Array[8]; // Levels fuer Kanaele, 1 byte pro kanal

volatile uint16_t Mix_Array[8];// Mixings, 2 8-bit-bytes pro Mixing


volatile uint16_t RAM_Array[SPI_BUFSIZE];

volatile uint8_t testdataarray[8]={};
volatile uint16_t teststartadresse=0xA0;


volatile uint16_t Batteriespannung =0;

volatile uint16_t adc_counter =0; // zaehlt Impulspakete bis wieder die Batteriespannung gelesen werden soll

volatile short int received=0;

volatile uint16_t abschnittnummer=0;

volatile uint16_t usbcount=0;

volatile uint16_t minwert=0xFFFF;
volatile uint16_t maxwert=0;

volatile uint16_t eepromstartadresse=0;

volatile uint16_t inchecksumme=0;

volatile uint16_t bytechecksumme=0;
volatile uint16_t outchecksumme=0;

volatile uint8_t eeprom_databyte=0;
volatile uint8_t anzahlpakete=0;
//volatile uint8_t usb_readcount = 0;

volatile uint8_t  eeprom_indata=0;

volatile    uint8_t task_in=0;      // Task von RC_PPM
volatile    uint8_t task_indata=0;  // Taskdata von RC_PPM

volatile    uint8_t task_out=0;     // Task an RC_PPM
volatile    uint8_t task_outdata=0; // Taskdata an RC_PPM

// Mark Screen

//#define CLOCK_DIV 15 // timer0 1 Hz bei Teilung /4 in ISR 16 MHz
#define CLOCK_DIV 8 // timer0 1 Hz bei Teilung /4 in ISR 8 MHz
#define BLINK_DIV 4 // timer0 1 Hz bei Teilung /4 in ISR 8 MHz



volatile uint16_t                TastaturCount=0;
volatile uint16_t                manuellcounter=0; // Counter fuer Timeout
volatile uint8_t                 startcounter=0; // timeout-counter beim Start von Settings, schneller als manuellcounter. Ermoeglicht Dreifachklick auf Taste 5
volatile uint16_t                mscounter=0; // Counter fuer ms in timer-ISR
volatile uint8_t                 blinkcounter=0;


uint8_t                          ServoimpulsSchrittweite=10;
uint16_t                         Servoposition[]={1000,1250,1500,1750,2000,1750,1500,1250};




//volatile uint16_t                SPI_Data_counter; // Zaehler fuer Update des screens



volatile uint16_t Tastenwert=0;
volatile uint16_t Trimmtastenwert=0;
volatile uint8_t adcswitch=0;


/*
 #define TASTE1		15
 #define TASTE2		23
 #define TASTE3		34
 #define TASTE4		51
 #define TASTE5		72
 #define TASTE6		94
 #define TASTE7		120
 #define TASTE8		141
 #define TASTE9		155
 #define TASTE_L	168
 #define TASTE0		178
 #define TASTE_R	194
 
 */
//const char wertearray[] PROGMEM = {TASTE1,TASTE2,TASTE3,TASTE4,TASTE5,TASTE6,TASTE7,TASTE8,TASTE9,TASTE_L,TASTE0,TASTE_R};

/*

static inline
uint16_t key_no( uint8_t adcval )
{
   uint16_t num = 0x1000;
   PGM_P pointer = wertearray;
   
   
   while( adcval < pgm_read_byte(pointer))
   {
      pointer++;
      num >>= 1;
   }
   return num & ~0x1000;
}

uint16_t get_key_press( uint16_t key_mask )
{
   cli();
   key_mask &= key_press;		// read key(s)
   key_press ^= key_mask;		// clear key(s)
   sei();
   return key_mask;
}

*/


#pragma mark 1-wire

//#define MAXSENSORS 2
static uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
static int16_t gTempdata[MAXSENSORS]; // temperature times 10
static uint8_t gTemp_measurementstatus=0; // 0=ok,1=error
static int8_t gNsensors=0;

uint8_t search_sensors(void)
{
   uint8_t i;
   uint8_t id[OW_ROMCODE_SIZE];
   uint8_t diff, nSensors;
   
   ow_reset();
   
   nSensors = 0;
   
   diff = OW_SEARCH_FIRST;
   while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS )
   {
      DS18X20_find_sensor( &diff, &id[0] );
      
      if( diff == OW_PRESENCE_ERR )
      {
         lcd_gotoxy(0,1);
         lcd_puts("No Sensor found\0" );
         
         delay_ms(800);
         lcd_clr_line(1);
         break;
      }
      
      if( diff == OW_DATA_ERR )
      {
         lcd_gotoxy(10,0);
         lcd_puts("BusErr\0" );
         lcd_puthex(diff);
         return OW_DATA_ERR;
         break;
      }
      //lcd_gotoxy(4,1);
      
      for ( i=0; i < OW_ROMCODE_SIZE; i++ )
      {
         //lcd_gotoxy(15,1);
         //lcd_puthex(id[i]);
         
         gSensorIDs[nSensors][i] = id[i];
         //delay_ms(100);
      }
      
      nSensors++;
   }
   
   return nSensors;
}

// start a measurement for all sensors on the bus:
void start_temp_meas(void)
{
   
   gTemp_measurementstatus=0;
   if ( DS18X20_start_meas(NULL) != DS18X20_OK)
   {
      gTemp_measurementstatus=1;
   }
}

// read the latest measurement off the scratchpad of the ds18x20 sensor
// and store it in gTempdata
void read_temp_meas(void){
   uint8_t i;
   uint8_t subzero, cel, cel_frac_bits;
   for ( i=0; i<gNsensors; i++ )
   {
      
      if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero,
                             &cel, &cel_frac_bits) == DS18X20_OK )
      {
         gTempdata[i]=cel*10;
         gTempdata[i]+=DS18X20_frac_bits_decimal(cel_frac_bits);
         if (subzero)
         {
            gTempdata[i]=-gTempdata[i];
         }
      }
      else
      {
         gTempdata[i]=0;
      }
   }
}


// Code 1_wire end



void startTimer2(void)
{
   //timer2
   TCNT2   = 0;
   //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode
   TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8
   //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt
   TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt
   //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64
   TCCR2A = 0x00;
   
   sei();
}

void stopTimer2(void)
{
   TCCR2B = 0;
}

void Master_Init(void)
{
	LOOPLEDDDR |=(1<<LOOPLED);
	LOOPLEDPORT |= (1<<LOOPLED);	// HI


	//Pin 0 von   als Ausgang fuer OSZI
	OSZIPORTDDR |= (1<<PULSA);	//Pin 0 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSA);		// HI
	
   OSZIPORTDDR |= (1<<PULSB);		//Pin 1 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSB);		//Pin   von   als Ausgang fuer OSZI
	
   /*
    TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang f�r Taste 0
    TASTENPORT |= (1<<TASTE0);	//Pull-up
    */
   
   
   
   // ---------------------------------------------------
   // Pin Change Interrupt enable on PCINT0 (PD7)
   // ---------------------------------------------------
   
   // PCIFR |= (1<<PCIF0);
   // PCICR |= (1<<PCIE0);
   //PCMSK0 |= (1<<PCINT7);
  
   // ---------------------------------------------------
   // USB_Attach
   // ---------------------------------------------------
   
   EICRA |= (1<<ISC01); // falling edge
   EIMSK=0;
   EIMSK |= (1<<INT0); // Interrupt en

   
   // ---------------------------------------------------
	//LCD
   // ---------------------------------------------------
	LCD_DDR |= (1<<LCD_RSDS_PIN);		// PIN als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD
   
   
}


void SPI_PORT_Init(void) // SPI-Pins aktivieren
{
   
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   
   
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO_PIN);
   SPI_PORT |= (1<<SPI_MISO_PIN); // HI
   SPI_DDR |= (1<<SPI_MOSI_PIN);
   SPI_DDR |= (1<<SPI_SCK_PIN);
//   SPI_PORT |= (1<<SPI_SCK_PIN); // HI
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
    
   
 
   
   /*
   // Slave init
   SPI_DDR |= (1<<SPI_MISO_PIN); // Output
   //SPI_PORT &= ~(1<<SPI_MISO_PIN); // HI
   SPI_DDR &= ~(1<<SPI_MOSI_PIN); // Input
   SPI_DDR &= ~(1<<SPI_SCK_PIN); // Input
   //SPI_PORT &= ~(1<<SPI_SCK_PIN); // LO
   SPI_DDR &= ~(1<<SPI_SS_PIN); // Input
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
    */
   
   
   
}

void SPI_ADC_init(void) // SS-Pin fuer EE aktivieren
{
   
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
}



void spi_start(void) // SPI-Pins aktivieren
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO_PIN);
   SPI_PORT &= ~(1<<SPI_MISO_PIN); // LO
   
   SPI_DDR |= (1<<SPI_MOSI_PIN);
   SPI_PORT &= ~(1<<SPI_MOSI_PIN); // LO
   
   SPI_DDR |= (1<<SPI_SCK_PIN);
   SPI_PORT &= ~(1<<SPI_SCK_PIN); // LO
   
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
  }

void spi_end(void) // SPI-Pins deaktivieren
{
   SPCR=0;
   
   SPI_DDR &= ~(1<<SPI_MOSI_PIN); // MOSI off
   SPI_DDR &= ~(1<<SPI_SCK_PIN); // SCK off
   SPI_DDR &= ~(1<<SPI_SS_PIN); // SS off
   
   //SPI_RAM_DDR &= ~(1<<SPI_RAM_CS_PIN); // RAM-CS-PIN off
   //SPI_EE_DDR &= ~(1<<SPI_EE_CS_PIN); // EE-CS-PIN off
}

/*
void spi_slave_init()
{
   SPCR=0;
   SPCR = (1<<SPE)|(1<<SPR1)|(0<<SPR0)|(1<<CPOL)|(1<<CPHA);
   
}
*/

void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

// http://www.co-pylit.org/courses/COSC2425/lectures/AVRNetworks/index.html

/* nicht verwendet
void timer1_init(void)
{
    // Quelle http://www.mikrocontroller.net/topic/103629
    
    OSZI_A_HI ; // Test: data fuer SR
    _delay_us(5);
    //#define FRAME_TIME 20 // msec
    KANAL_DDR |= (1<<KANAL_PIN); // Kanal Ausgang
    
    DDRD |= (1<<PORTD5); //  Ausgang
    PORTD |= (1<<PORTD5); //  Ausgang
    
    //TCCR1A = (1<<COM1A0) | (1<<COM1A1);// | (1<<WGM11);	// OC1B set on match, set on TOP
    //TCCR1B = (1<<WGM13) | (1<<WGM12) ;		// TOP = ICR1, clk = sysclk/8 (->1us)
    TCCR1B |= (1<<CS11);
    TCNT1  = 0;														// reset Timer
    
    // Impulsdauer
    OCR1B  = 0x80;				// Impulsdauer des Kanalimpulses
    
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt:
} // end timer1
*/
/*
void timer1_stop(void)
{
   // TCCR1A = 0;
   
}
 */
/*
 ISR(TIMER1_COMPA_vect)	 //Ende der Pulslaenge fuer einen Kanal
 {
 
 KANAL_HI;
 impulscounter++;
 
 if (impulscounter < ANZ_POT)
 {
 // Start Impuls
 
 TCNT1  = 0;
 //KANAL_HI;
 
 // Laenge des naechsten Impuls setzen
 
 //OCR1A  = POT_FAKTOR*Pot_Array[1]; // 18 us
 //OCR1A  = POT_FAKTOR*Pot_Array[impulscounter]; // 18 us
 
 }
 }
 */

/*
 ISR(TIMER1_COMPB_vect)	 //Ende des Kanalimpuls. ca 0.3 ms
 {
 //OSZI_A_LO ;
 //PORTB &= ~(1<<PORTB5); // OC1A Ausgang
 //OSZI_A_HI ;
 KANAL_LO;
 
 if (impulscounter < ANZ_POT)
 {
 }
 else
 {
 timer1_stop();
 
 }
 }
 */


void timer0 (void) // Grundtakt fuer Stoppuhren usw.
{
   // Timer fuer Exp
	//TCCR0 |= (1<<CS01);						// clock	/8
	//TCCR0 |= (1<<CS01)|(1<<CS02);			// clock	/64
	//TCCR0B |= (1<<CS02)| (1<<CS02);			// clock	/256
	//TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
	
   TCCR0B |= (1 << CS02);//
   //TCCR0B |= (1 << CS00);
   
   TCCR0B |= (1 << CS10); // Set up timer
	
   OCR0A = 0x02;
	
	//TIFR |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);							//Overflow Interrupt aktivieren
	TCNT0 = 0;					//R�cksetzen des Timers
}

/*
 void timer2 (uint8_t wert)
 {
 //timer2
 TCNT2   = 0;
 //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode
 TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8
 //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt
 TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt
 //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64
 TCCR2A = 0x00;
 
 
 OCR2 = wert;									//Setzen des Compare Registers auf Servoimpulsdauer
 }
 */

volatile uint16_t timer2Counter=0;
volatile uint16_t timer2BatterieCounter=0;

#pragma mark TIMER0_OVF

ISR (TIMER0_OVF_vect)
{
   mscounter++;
    
   if (mscounter%BLINK_DIV ==0)
   {
      blinkcounter++;
   }
   
}

#pragma mark timer1
void timer1(void)
{
   
   //SERVODDR |= (1<<SERVOPIN0);
   /*
    TCCR1A = (1<<WGM10)|(1<<COM1A1)   // Set up the two Control registers of Timer1.
    |(1<<COM1B1);             // Wave Form Generation is Fast PWM 8 Bit,
    TCCR1B = (1<<WGM12)|(1<<CS12)     // OC1A and OC1B are cleared on compare match
    |(1<<CS10);               // and set at BOTTOM. Clock Prescaler is 1024.
    
    OCR1A = 63;                       // Dutycycle of OC1A = 25%
    //OCR1B = 127;                      // Dutycycle of OC1B = 50%
    
    return;
    */
   // https://www.mikrocontroller.net/topic/83609
   
   
   OCR1A = 0x3E8;           // Pulsdauer 1ms
   OCR1A = 0x200;
   //OCR1A = Servoposition[2];
   //OCR1B = 0x0FFF;
   ICR1 = 0x6400;          // 0x6400: Pulsabstand 50 ms

   /*
   // TCCR1A |= (1<<COM1A0);
   TCCR1A |= (1<<COM1A1); // clear on compare match
   TCCR1A |= (1<<WGM11);
   
   TCCR1B |= (1<<WGM12);
   TCCR1B |= (1<<WGM13);
   
   TCCR1B |= (1<<CS11);
  */
   // http://www.ledstyles.de/index.php/Thread/18214-ATmega32U4-Schaltungen-PWM/
   DDRB |= (1<<DDB6)|(1<<DDB5);

   TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)|(1<<WGM11)|(1<<WGM12);
   
   TCCR1B |= (1<<WGM13)|(1<<CS11);
//   TCCR1A=0xAA;
//   TCCR1B=0x19;
   // TCCR1B |= (1<<CS10);
   
   
   
   //  TIMSK |= (1<<OCIE1A) | (1<<TICIE1); // OC1A Int enablad
}


#pragma mark INT0
ISR(INT0_vect) // Interrupt bei CS, falling edge
{
//   OSZI_A_LO;
   inindex=0;
   SPDR = 0;// Erstes Byte an Slave
//   OSZI_A_HI;
   
 //  spi_txbuffer[0]++;
 //  spi_txbuffer[2]--;

}





#pragma mark PIN_CHANGE
//https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328

/*
ISR (PCINT0_vect)
{
   
   if(INTERRUPT_PIN & (1<< MASTER_EN_PIN))// LOW to HIGH pin change, Sub ON
   {
      //OSZI_C_LO;
     
      masterstatus |= (1<<SUB_TASK_BIT); // Zeitfenster fuer Task offen
      adc_counter ++; // loest adc aus
      
   }
   else // HIGH to LOW pin change, Sub ON
   {
      displaystatus |= (1<<UHR_UPDATE);
      //masterstatus &= ~(1<<SUB_TASK_BIT);
   }
   
}
 */

uint8_t Tastenwahl(uint8_t Tastaturwert)
{
   /*
    // Atmega168
    
    #define TASTE1		19
    #define TASTE2		29
    #define TASTE3		44
    #define TASTE4		67
    #define TASTE5		94
    #define TASTE6		122
    #define TASTE7		155
    #define TASTE8		186
    #define TASTE9		212
    #define TASTE_L	234
    #define TASTE0		248
    #define TASTE_R	255
    
    
    // Atmega328
    #define TASTE1		17
    #define TASTE2		29
    #define TASTE3		44
    #define TASTE4		67
    #define TASTE5		94
    #define TASTE6		122
    #define TASTE7		155
    #define TASTE8		166
    #define TASTE9		214
    #define TASTE_L	234
    #define TASTE0		252
    #define TASTE_R	255
    */
   
   //lcd_gotoxy(0,0);
   //lcd_putint(Tastaturwert);
   /*
   if (Tastaturwert < TASTE1)
      return 1;
   if (Tastaturwert < TASTE2)
      return 2;
   if (Tastaturwert < TASTE3)
      return 3;
   if (Tastaturwert < TASTE4)
      return 4;
   if (Tastaturwert < TASTE5)
      return 5;
   if (Tastaturwert < TASTE6)
      return 6;
   if (Tastaturwert < TASTE7)
      return 7;
   if (Tastaturwert < TASTE8)
      return 8;
   if (Tastaturwert < TASTE9)
      return 9;
   
   if (Tastaturwert < TASTE_L)
      return 10;
   if (Tastaturwert < TASTE0)
      return 0;
   if (Tastaturwert <= TASTE_R)
      return 12;
   */
   
   
   
   // Tastatur2 // Reihenfolge anders
   /*
    #define WERT1    11    // 1 oben  Taste 2
    #define WERT3    34    // 2 links  Taste 4
    #define WERT4    64    // 3 unten  Taste 8
    #define WERT6    103   // 4 rechts  Taste 6
    #define WERT9    174   // 5 Mitte  Taste 5
    #define WERT2 	26    //  A links oben Taste  1
    #define WERT5    72       //    B links unten Taste 7
    #define WERT7    116      //   C rechts oben Taste 3
    #define WERT8    161      // D rechts unten Taste 9
    
    */

   if (Tastaturwert < WERT1)
      return 2;
   if (Tastaturwert < WERT2)
      return 1;
   if (Tastaturwert < WERT3)
      return 4;
   if (Tastaturwert < WERT4)
      return 8;
   if (Tastaturwert < WERT5)
      return 7;
   if (Tastaturwert < WERT6)
      return 6;
   if (Tastaturwert < WERT7)
      return 3;
   if (Tastaturwert < WERT8)
      return 9;
   if (Tastaturwert < WERT9)
      return 5;

   return -1;



}




/*
//#define OSZIA_LO() OSZIPORT &= ~(1<<4)
#define OSZI_A_HI() OSZIPORT |= (1<<4)
#define OSZI_A_TOGG() OSZIPORT ^= (1<<4)
*/

//#define OSZIA_LO( bit) PORTD &= ~(1 << (PULSA))

#define setbit(port, bit) (port) |= (1 << (bit))
#define clearbit(port, bit) (port) &= ~(1 << (bit))



// MARK:  - main
int main (void)
{
   
   uint16_t tempwert = 444;
   int8_t r;
   
   uint16_t spi_count=0;
   
	// set for 16 MHz clock
	CPU_PRESCALE(CPU_8MHz); // Strom sparen
   
	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
   
   //   sei();
	Master_Init();
   
   // ---------------------------------------------------
   // in attach verschobe, nur wenn USB eingesteckt
     usb_init();
   	while (!usb_configured()) /* wait */ ;
   
	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(100);
   
   volatile    uint8_t outcounter=0;
   volatile    uint8_t testdata =0x00;
//   volatile    uint8_t testaddress =0x00;
   volatile    uint8_t errcount =0x00;
//   volatile    uint8_t ram_indata=0;
   
 //  volatile    uint8_t eeprom_indata=0;
 //  volatile    uint8_t eeprom_testdata =0x00;
 //  volatile    uint8_t eeprom_testaddress =0x00;
   volatile    uint8_t usb_readcount =0x00;
   
   // ---------------------------------------------------
 	// initialize the LCD
   // ---------------------------------------------------
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   
	lcd_puts("Guten Tag\0");
	delay_ms(100);
	lcd_cls();
	//lcd_puts("READY\0");
	lcd_puts("V: \0");
	lcd_puts(VERSION);
   lcd_clr_line(1);
	
   lcd_gotoxy(0,0);
   lcd_puts("Data_Logger\0");
   delay_ms(1000);
   //lcd_cls();
   lcd_clr_line(0);
   
	uint8_t TastaturCount=0;
		
	//initADC(1);
	
	uint16_t loopcount0=0;
	uint16_t loopcount1=0;
	/*
    Bit 0: 1 wenn wdt ausgel�st wurde
    */
   
   PWM = 0;
   
   char* versionstring[4] = {};
   strncpy((char*)versionstring, (VERSION+9), 3);
   versionstring[3]='\0';
   volatile uint16_t versionint = atoi((char*)versionstring);
   volatile uint8_t versionintl = versionint & 0x00FF;
   //versionint >>=8;
   volatile uint8_t versioninth = (versionint & 0xFF00)>>8;
   
   uint8_t anzeigecounter=0;
   
 //  masterstatus |= (1<<SUB_READ_EEPROM_BIT); // sub soll EE lesen
   
#pragma mark DS1820 init
   
   // DS1820 init-stuff begin
  // OW_OUT |= (1<<OW_PIN);
   uint8_t i=0;
   uint8_t nSensors=0;
   uint8_t err = ow_reset();
   lcd_gotoxy(18,0);
   lcd_puthex(err);
   gNsensors = search_sensors();
   
   delay_ms(100);
   //lcd_gotoxy(0,0);
   //lcd_puts("Sn:\0");
   //lcd_puthex(gNsensors);
   if (gNsensors>0)
   {
      lcd_clr_line(1);
      start_temp_meas();
   }
   i=0;
   while(i<MAXSENSORS)
   {
      gTempdata[i]=0;
      i++;
   }
   delay_ms(100);
   if (gNsensors>0)
   {
      lcd_gotoxy(0,0);
      lcd_puts("Sns:\0");
      lcd_puthex(gNsensors);

      lcd_clr_line(1);
      start_temp_meas();
   }
   i=0;
   while(i<MAXSENSORS)
   {
      gTempdata[i]=0;
      i++;
   }

   // DS1820 init-stuff end

   SPI_PORT_Init();

   // ---------------------------------------------------
   // Vorgaben fuer Homescreen
   // ---------------------------------------------------
   //substatus |= (1<<SETTINGS_READ);;
   // ---------------------------------------------------
   // Settings beim Start lesen
   // ---------------------------------------------------
   
  // timer1(); PORTB5,6, Servo
   
   sei();
   

   
   volatile   uint8_t old_H=0;
   volatile   uint8_t old_L=0;
   uint8_t teensy_err =0;
   uint8_t testfix=0;


   // MMC
   OSZIPORTDDR |= (1<<PULSA);	//Pin 0 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSA);		// HI
   
#pragma mark MMC
#define DRV_MMC 0
   long p1, p2, p3;
   
   BYTE Buff[4096];	/* Working buffer */
   
   FATFS FatFs[2];		/* File system object for each logical drive */
   FIL File[2];		/* File object */
   DIR Dir;			/* Directory object */
   FILINFO Finfo;
   DWORD AccSize;				/* Work register for fs command */
   WORD AccFiles, AccDirs;
   
   BYTE RtcOk;				/* RTC is available */
   volatile UINT Timer;	/* Performance timer (100Hz increment) */
   
   //uint8_t mmcerr = disk_initialize(DRV_MMC);
//   lcd_gotoxy(0,0);
//   lcd_puthex(mmcerr);
   
  // SourceCodeV3
   lcd_gotoxy(0,1);
   uint8_t mmcerr = mmc_init();
   lcd_gotoxy(10,0);
   lcd_puthex(mmcerr);
   
   
//   while ( mmc_init() !=0) //ist der R�ckgabewert ungleich NULL ist ein Fehler aufgetreten
   {
//      lcd_puts("NO");
   }
   //lcd_puts("YES");
   
   
    // SourceCodeV3  end
   // MARK:  while
   while (1)
	{
      //OSZI_B_LO;
		//Blinkanzeige
		loopcount0+=1;
      
      
      /* **** spi_buffer abfragen **************** */
// MARK:  spi_rxdata
      
      /* **** end spi_buffer abfragen **************** */
      
      //OSZI_A_TOGG;
      
		if (loopcount0==0xDFFF)
		{
         /*
          uint8_t err = ow_reset();
         lcd_gotoxy(12,0);
         lcd_puthex(err);
         //PORTD &= ~(1<<4);
         OSZIA_LO;
         //clearbit(PORTD, PD4);
         ow_delay_us(250);
         ow_delay_us(250);
         ow_delay_us(50);
         OSZIA_HI;
         //setbit(PORTD, PD4);
         //PORTD |=  (1<<4);
          */
			loopcount0=0;
			loopcount1+=1;
			LOOPLEDPORT ^=(1<<LOOPLED);
         
         //SPI_PORT ^= (1<<SPI_SCK_PIN);
         
         //mmc_write_byte(loopcount1 & 0xFF);
         
         //OSZI_A_LO;
         //lcd_gotoxy(18,0);
         //lcd_puthex(loopcount1);
         
         sendbuffer[0] = ((usb_readcount));
         sendbuffer[1] = (loopcount1 & 0xFF);
         sendbuffer[2] = 16;
         sendbuffer[3] = 75;
        // sendbuffer[5] = spi_rxbuffer[0];
         
         for (int i=0;i<SPI_BUFSIZE;i++)
         {
            //lcd_puthex(spi_rxbuffer[i]);
//            sendbuffer[i+CODE_OFFSET] = spi_rxbuffer[i];
         }

         lcd_gotoxy(0,1);
         lcd_putc('S');
         //lcd_putc(' ');
          for (int i=0;i<4;i++)
          {
             lcd_puthex(sendbuffer[i]);
          }

         /*
         lcd_gotoxy(0,1);
         lcd_putc('R');
         lcd_puthex(spi_rxbuffer[0]);
         lcd_putc('*');
         lcd_puthex(spi_rxbuffer[1]);
         
         lcd_puthex(spi_rxbuffer[2]);
         lcd_puthex(spi_rxbuffer[3]);
         lcd_putc('-');
         
         lcd_puthex(spi_rxbuffer[4]);
         lcd_puthex(spi_rxbuffer[5]);
         lcd_puthex(spi_rxbuffer[6]);
         lcd_puthex(spi_rxbuffer[7]);
         */
         /*
         if (OHNE_INTERFACE)
         {
            spi_txbuffer[0] = 0x81;
            tempwert++;
            spi_txbuffer[1] = tempwert & 0x00FF;
            spi_txbuffer[2] = (tempwert & 0xFF00)>>8;
            spi_txbuffer[3] = 7;
         }
          */
         /*
          // Ausgabe auf soft-SPI
         lcd_gotoxy(0,0);
         lcd_putc('T');
         lcd_puthex(spi_txbuffer[0]);
         lcd_putc('+');
         lcd_puthex(spi_txbuffer[1]);
         
         lcd_puthex(spi_txbuffer[2]);
         lcd_puthex(spi_txbuffer[3]);
          */
        // lcd_putc('i');

        // lcd_puthex(isrcontrol);
         uint8_t testbit=1;
         if (!((old_H == spi_rxbuffer[testbit])) )
         {
            teensy_err++;
            old_H = spi_rxbuffer[testbit];
            testfix = loopcount1;
         }

         
         /*
         if (!((old_H == spi_rxbuffer[3]) && (old_L == spi_rxbuffer[2])) )
         
         {
            teensy_err++;
            old_L = spi_rxbuffer[2];
            old_H = spi_rxbuffer[3];
         }
          */
         
         //lcd_gotoxy(14,1);
         //lcd_puthex(spi_count);

         //lcd_gotoxy(16,1);
         //lcd_puthex(spi_count);
        
//         lcd_gotoxy(18,1);
//         lcd_puthex(isrcontrol);
/*
         lcd_puthex(sendbuffer[CODE_OFFSET]);
         lcd_puthex(sendbuffer[CODE_OFFSET+1]);
         lcd_puthex(sendbuffer[ROTARY_OFFSET]);
         lcd_puthex(sendbuffer[ROTARY_OFFSET+1]);
         lcd_putc(' ');
         uint16_t rot = (sendbuffer[ROTARY_OFFSET+1]<<8) | sendbuffer[ROTARY_OFFSET];
         lcd_putint16(rot);
 */
         //lcd_puthex(sendbuffer[9]);
         //lcd_putc(' ');
         //lcd_puthex(sendbuffer[8]);
         //lcd_putc(' ');
         //lcd_puthex(SPI_Data_counter);
      //   sendbuffer[5] = isrcontrol;
         sendbuffer[4] = 28;
//         lcd_gotoxy(11,0);
 //        lcd_putc('x');
//         lcd_puthex(usb_readcount);

         uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
         //lcd_puthex(usberfolg);
         
         
         if(loopcount1%16 == 0)
         {
        

#pragma mark Sensors
         // Temperatur messen mit DS18S20
         
         if (gNsensors) // Sensor eingesteckt
         {
            start_temp_meas();
            delay_ms(800);
            read_temp_meas();
            uint8_t line=1;
            //Sensor 1
            lcd_gotoxy(10,line);
            lcd_puts("T:     \0");
            if (gTempdata[0]/10>=100)
            {
               lcd_gotoxy(13,line);
               lcd_putint((gTempdata[0]/10));
            }
            else
            {
               lcd_gotoxy(12,line);
               lcd_putint2((gTempdata[0]/10));
            }
            
            lcd_putc('.');
            lcd_putint1(gTempdata[0]%10);
         }
          
            sendbuffer[DSLO]=((gTempdata[0])& 0x00FF);// T kommt mit Faktor 10 vom DS. Auf TWI ist T verdoppelt
            sendbuffer[DSHI]=((gTempdata[0]& 0xFF00)>>8);// T kommt mit Faktor 10 vom DS. Auf TWI ist T verdoppelt
            //lcd_gotoxy(10,0);
            //lcd_putint(sendbuffer[DSLO]);
            //lcd_putint(sendbuffer[DSHI]);
            // Halbgrad addieren
         if (gTempdata[0]%10 >=5) // Dezimalstelle ist >=05: Wert  aufrunden, 1 addieren
         {
           // txbuffer[INNEN] +=1;
         }
       } //
         
			

// MARK:  USB send
         // neue Daten abschicken
//         if ((usbtask & (1<<EEPROM_WRITE_PAGE_TASK) )) //|| usbtask & (1<<EEPROM_WRITE_BYTE_TASK))
         //OSZI_C_LO;
         
      //   uint8_t anz = usb_rawhid_send((void*)sendbuffer, 50); // 20 us
         //OSZI_C_HI;
         //OSZI_A_HI;

         // OSZI_B_HI;
      } // if loopcount0
      
      /**	ADC	***********************/
      
      /**	END ADC	***********************/
      
      /**	Begin USB-routinen	***********************/
// MARK USB read
      // Start USB
      //OSZI_D_LO;
      r=0;
      
      
      
  //    SPI_PORT |= (1<<SPI); // PIN setzen, um Master die Praesenz zu melden

      
      
      
      r = usb_rawhid_recv((void*)recvbuffer, 0); // 5us
      //OSZI_D_HI;
      // MARK: USB_READ
      
      if (r > 0)
      {
         //OSZI_A_LO ;
         
         //OSZI_D_LO;
         cli();
         usb_readcount++;
         uint8_t code = 0x00;
         
         
         lcd_gotoxy(0,0);
         //lcd_putc('*');
         //lcd_puthex(r);
         //lcd_putc('*');
         //lcd_putint2(usb_readcount);
         lcd_putc('R');
         
         lcd_puthex(recvbuffer[8]);
         lcd_puthex(recvbuffer[9]);
         
         lcd_gotoxy(10,0);

         //lcd_puthex(recvbuffer[10]);
         //lcd_puthex(recvbuffer[11]);
         //lcd_putint12(recvbuffer[10] | (recvbuffer[11]<<8));
         OCR1A = (recvbuffer[10] | (recvbuffer[11]<<8));
         lcd_putc('*');
         

      //   for (i=0;i<SPI_BUFFERSIZE;i++)
         {
            //lcd_puthex(spi_rxbuffer[i]);
            //sendbuffer[i+ROTARY_OFFSET] = spi_rxbuffer[i];
         }

                  //lcd_putc('*');
         //lcd_puthex(r);
         //lcd_putc('*');
         //lcd_puthex(sendbuffer[ROTARY_OFFSET]);
         //lcd_putc('*');
         //lcd_puthex(spi_rxbuffer[0]);
         
        // lcd_gotoxy(10,1);
         //lcd_putc('b');
         //lcd_puthex(spi_rxbuffer[1]);
         /*
         //lcd_putc('d');
         lcd_puthex(buffer[1]);
         lcd_puthex(buffer[2]);
         */
         /*
         lcd_gotoxy(11,0);
         lcd_puthex(buffer[15]);

         lcd_gotoxy(15,0);
         lcd_puthex(spi_txbuffer[0]);

         spi_txbuffer[0] = buffer[0];
         
         
         lcd_gotoxy(17,0);
         lcd_puthex(spi_txbuffer[0]);
        */
         if (recvbuffer[15] & (1<<7)) // SPI-CONTROL setzen. MASTER soll SPI abfragen
         {
            spi_txbuffer[0] |= (1<<7);
 //           SPI_PORT &= ~(1<<SPI); // PIN setzen, um Master die Praesenz zu melden. Aktiv LO
            spistatus |= (1<< SPI_RUN_BIT); // MASTER soll SPI abfragen
            usbstatus |= (1<<USB_NEW);
            spi_count=0;
            lcd_gotoxy(19,0);
            lcd_putc('+');

         }
         else
         {
            spi_txbuffer[0] &= ~(1<<7);
 //           SPI_PORT |= (1<<SPI); // HI ist : Teensy deaktiviert
            spistatus &= ~(1<< SPI_RUN_BIT);
            lcd_gotoxy(19,0);
            lcd_putc('-');

         }
         
         spi_txbuffer[1] = recvbuffer[1];
         spi_txbuffer[2] = recvbuffer[2];
         spi_txbuffer[3] = recvbuffer[3];
         
         /*
         spi_txbuffer[0] = 0x81;
         uint16_t tempwert = 444;
         spi_txbuffer[1] = tempwert & 0x00FF;
         spi_txbuffer[2] = (tempwert & 0xFF00)>>8;
         //spi_txbuffer[3] = 7;
*/
         
         //lcd_puthex(buffer[0]);

         /*
         sendbuffer[0] = usb_readcount;
         sendbuffer[1] = 13;
         sendbuffer[2] = 17;
         sendbuffer[3] = 75;
         sendbuffer[5] = spi_rxbuffer[0];
         lcd_puthex(sendbuffer[5]);
          */
       //  lcd_putc('*');
        
         
      //   uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
         
        // lcd_gotoxy(0,1);
        // lcd_putc('*');
       //  lcd_putint(usberfolg);
       //  lcd_putc('*');
         
         

         usbstatus |= (1<<USB_RECV);
         {
            code = 0;//buffer[0];
            /*
            switch (code)
            {
               case 0xC0: // Write EEPROM Page start
               {
                  
                  usb_readcount=0;
                  abschnittnummer++;
                  eepromstartadresse = buffer[1] | (buffer[2]<<8);
                  anzahlpakete = buffer[3];
                  //eepromstatus |= (1<<EE_WRITE);
                  lcd_gotoxy(8,0);
                  lcd_putc('E');
                  lcd_puthex(code);
                  lcd_putc('*');
                  lcd_putint(anzahlpakete);
                  lcd_putc('*');
                  lcd_putint(abschnittnummer);
                  sendbuffer[0] = 0xC1;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  lcd_putc('*');
                  lcd_gotoxy(18,1);
                  lcd_putc('W');
                  lcd_putc('P');
                  
                  usbtask |= (1<<EEPROM_WRITE_PAGE_TASK);
                  masterstatus |= (1<<SUB_TASK_BIT);
               }break;
                  
                  
                  
                  
                  
                  // ---------------------------------------------------
                  // MARK: C4 Write EEPROM Byte
                  // ---------------------------------------------------
                  
               case 0xC4: // Write EEPROM Byte  // 10 ms
               {
                  //OSZI_A_TOGG;
                  usb_readcount=0;
                  //abschnittnummer++;
                  eepromstartadresse = buffer[1] | (buffer[2]<<8);
                  eeprom_databyte = buffer[3];
                  
                  //anzahlpakete = buffer[3];
                  //eepromstatus |= (1<<EE_WRITE);
                  lcd_gotoxy(18,1);
                  lcd_putc('W');
                  lcd_putc('1');
                  //lcd_putc('*');
                  
                  
                  
                  sendbuffer[0] = 0xC5;
                  
                  sendbuffer[1] = eepromstartadresse & 0xFF;
                  sendbuffer[2] = (eepromstartadresse & 0xFF00)>>8;
                  
                  sendbuffer[3] = buffer[3];
                  sendbuffer[4] = 0; //check;// ist bytechecksumme
                  sendbuffer[5] = usb_readcount;
                  
                  sendbuffer[6] = 0xFF;
                  sendbuffer[7] = 0xFF;
                  
                  sendbuffer[8] = 0xF8;
                  sendbuffer[9] = 0xFB;
                  
                  usb_rawhid_send((void*)sendbuffer, 50);
                  
                  masterstatus |= (1<<SUB_TASK_BIT);
                  //usbtask |= (1<<EEPROM_WRITE_BYTE_TASK);
                  
               }break;
                  // ---------------------------------------------------
             
                  
            } // switch code
             
             */
         }
         //OSZI_A_HI;
         //lcd_putc('$');
         code=0;
         sei();
         
         //OSZI_D_HI;
         //OSZI_A_HI ;
		} // r>0, neue Daten
      else
      {
         //OSZI_B_LO;
      }
      
      /**	End USB-routinen	***********************/
 		
	    
      
		//OSZI_B_HI;
      
	}//while
   //free (sendbuffer);
   
   // return 0;
}
