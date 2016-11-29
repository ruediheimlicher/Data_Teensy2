#define CPU_16MHz       0x00
#define CPU_8MHz        0x01

#define OHNE_INTERFACE 0

#define USB_PACKETSIZE 64

#define TEST 1
//Oszi
#define OSZIPORT           PORTD
#define OSZIPORTDDR        DDRD
#define OSZIPORTPIN        PIND
#define OSZI_PULS_A        0
#define PULSA              4
#define PULSB              5
#define OSZI_PULS_B        5


#define OSZIA_LO OSZIPORT &= ~(1<<PULSA)
#define OSZIA_HI OSZIPORT |= (1<<PULSA)
#define OSZIA_TOGG OSZIPORT ^= (1<<PULSA)


#define OSZI_B_LO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG OSZIPORT ^= (1<<OSZI_PULS_B)




#define LOOPLEDDDR          DDRD    
#define LOOPLEDPORT         PORTD   
#define LOOPLED             6       // fix verdrahtet

#if(MCU ==atmega32u4)//        # Teensy 2.0
#define SPI_PORT           PORTB
#define SPI_DDR            DDRB
#define SPI_MISO           PB3
#define SPI_MOSI           PB2
#define SPI_CLK            PB1
#define SPI_SS             PB0
#endif

// bits von usbstatus

#define WRITEAUTO          0

#define WRITETEMPERATUR    2

#define READAUTO           1

// bits von spistatus

#define WRITE_SPANNUNG        1
#define WRITE_STROM           2
#define SPI_RUN_BIT           7

#define SOFT_SPI_PORT           PORTB
#define SOFT_SPI_DDR            DDRB
#define SOFT_SPI_MISO           PB3
#define SOFT_SPI_MOSI           PB2
#define SOFT_SPI_SCLK           PB1
#define SOFT_SPI_SS_0            PB0
#define SOFT_SPI_SS_1            PB0

// USB Eingang
// Temperatur
#define DSLO               8
#define DSHI               9

// ADC
#define  ADCLO             10
#define  ADCHI             11

// USB Ausgang
#define SERVOALO           10
#define SERVOAHI           11

// write SD
#define WRITE_MMC_TEST     0xF1


// default
#define DEFAULT            0xFE

// logger
#define LOGGER_START       0xA0
#define LOGGER_CONT        0xA1

#define LOGGER_STOP        0xAF


// EEPROM Speicherorte

#pragma mark               Transfer

#define TRANSFERBLOCK      0xA0  //Transfer an Interface. Byte 1: Abschnitt Byte 2,3: Blockoffset Byte 4,5: Anzahl Blocks
#define TASK_OFFSET        0x2000 // Ort fuer Einstellungen

#define PACKET_SIZE     0x30 // 48 bytes fuer USB-Transfer
//#define PACKET_SIZE     0x18 // 24 bytes fuer USB-Transfer
#define PACKET_START    8    // erstes byte

#define SETTINGBREITE      0x100; // 256 Bytes, Breite des Settingblocks fuer ein model

#define  MITTE_OFFSET      0x10 // 16
#define  LEVEL_OFFSET      0x20 // 32
#define  EXPO_OFFSET       0x30 // 48
#define  MIX_OFFSET        0x40 // 64

#define  TRIMM_OFFSET      0x50 // 80

#define FUNKTION_OFFSET    0x60 // 96
#define DEVICE_OFFSET      0x70 // 122
#define AUSGANG_OFFSET     0x80 // 128



#define SAVE_LEVEL         0
#define SAVE_TRIMM         1
#define SAVE_MIX           2
#define SAVE_EXPO          3
#define SAVE_FUNKTION      4
#define SAVE_DEVICE        5
#define SAVE_AUSGANG       6


// Tastatur
// Atmega168
/*
 #define TASTE1		19
 #define TASTE2		29
 #define TASTE3		44
 #define TASTE4		67
 #define TASTE5		94
 #define TASTE6		122
 #define TASTE7		155
 #define TASTE8		186
 #define TASTE9		212
 #define TASTE_L     234
 #define TASTE0		248
 #define TASTE_R     255
 */
/*
// Atmega328
#define TASTE1		17
#define TASTE2		29
#define TASTE3		44
#define TASTE4		67
#define TASTE5		94
#define TASTE6		122
#define TASTE7		155
#define TASTE8		190
#define TASTE9		214
#define TASTE_L	234
#define TASTE0		252
#define TASTE_R	255
*/
/*
// Teensy2 int ref/TL431
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
/*
// Teensy2 int ref/TL431
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
// Teensy2 int Vcc Tastatur1
/*
#define TASTE1		16
#define TASTE2		26
#define TASTE3		40
#define TASTE4		62
#define TASTE5		88
#define TASTE6		114
#define TASTE7		146
#define TASTE8		177
#define TASTE9		222


#define TASTE_L	168
#define TASTE0		178
#define TASTE_R	194
*/

// Teensy2 int VCC Tastatur2

#define WERT1    19    // 1 oben  Taste 2
#define WERT3    49    // 2 links  Taste 4
#define WERT4    68    // 3 unten  Taste 8
#define WERT6    110   // 4 rechts  Taste 6
#define WERT9    215   // 5 Mitte  Taste 5
#define WERT2     30    //  A links oben Taste  1
#define WERT5    88       //    B links unten Taste 7
#define WERT7    139      //   C rechts oben Taste 3
#define WERT8    168      // D rechts unten Taste 9
#define WERT9    225      // Mitte rechts unten Taste 9

/*
#define TASTE_L_O    15
#define TASTE_L_L		23
#define TASTE_L_U		34
#define TASTE_L_R    51
#define TASTE_L_M		72
#define TASTE_R_O		94
#define TASTE_R_L		120
#define TASTE_R_U		141
#define TASTE_R_R    155
#define TASTE_R_M    168
*/

/*
 // Tastatur 3x3
#define TASTE_L_O    16
#define TASTE_L_L		26
#define TASTE_L_U		40
#define TASTE_L_R    62
#define TASTE_L_M		88
#define TASTE_R_O		114
#define TASTE_R_L		146
#define TASTE_R_U		177
#define TASTE_R_R    222
#define TASTE_R_M    245
*/

// Tastatur 2xPitch
#define TASTE_L_O    22
#define TASTE_L_L		61
#define TASTE_L_U		102
#define TASTE_L_R    135
#define TASTE_L_M		205
#define TASTE_R_O		38
#define TASTE_R_L		80
#define TASTE_R_U		123
#define TASTE_R_R    178
#define TASTE_R_M    235




#define MANUELLTIMEOUT	100 // Loopled-counts bis Manuell zurueckgesetzt wird. 50: ca. 30s

#define TRIMMTIMEOUT	4 // Loopled-counts bis Manuell zurueckgesetzt wird. 50: ca. 30s


//#define MITTE_TASK         0x01 // Mitte lesen
//#define KANAL_TASK         0x02 // Level und Expo lesen
//#define MIX_TASK           0x03 // Mix lesen


#define MS_DIV          4	// Bit 4 von Status. Gesetzt wenn 1s abgelaufen
#define UPDATESCREEN    5 // Bit in status wird gesetzt wenn eine Taste gedrueckt ist, reset wenn update ausgefuehrt

#define SETTINGWAIT     6  // Bit in status wird gesetzt bis Taste 5 3* gedrueckt ist

//#define MANUELL			7	// Bit 7 von Status

#define MINWAIT         3 // Anzahl loops von loopcount1 bis einschalten




// end Screen




// SPI



#define SPI_BUFSIZE 8

#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define WDTBIT              7

#define LOOPLEDLONG           0xDFFF
#define LOOPLEDSHORT          0x4FFF





// Bit

// bits von programmstatus

#define MOTOR_ON        1
#define STOP_ON         2
#define EEPROM_TASK     3  // Daten in EEPROM sichern
#define USB_ATTACH_TASK  4  // USB initiieren



// Bits von masterstatus
#define  SUB_TASK_BIT             4 // Sub hat Aufgaben
#define  SUB_READ_EEPROM_BIT      5 // Sub soll EEPROM lesen
#define  DOGM_BIT                6 // Master soll EE lesen nach Aenderungen im DOGM
#define  HALT_BIT                7 //Bit 7

// Bits von eepromstatus
#define READ_EEPROM_START        0  // Beim Start gesetzt. Soll einmaliges Lesen der Settings beim Update des Masters ausloesen


// Bits von displaystatus
#define UHR_UPDATE         0
#define BATTERIE_UPDATE    1




// Bits fuer usbstatus
#define USB_RECV  0

#define USB_NEW   1

#define USB_ATTACH            1 // USB_Spannung detektiert



#define ANZ_POT               6 // Anzahl zu lesender Potis

#define POT_FAKTOR            1.20 // Korrekturfaktor fuer Potentiometerstellung


#define MASTER_PORT            PORTD   //    
#define MASTER_DDR             DDRD    //    
#define MASTER_PIN             PIND    //
// PIN's
#define SUB_BUSY_PIN             5 // Ausgang fuer busy-Meldung des Sub an Master

#define INTERRUPT_PORT            PORTB   //
#define INTERRUPT_DDR             DDRB    //
#define INTERRUPT_PIN             PINB    //
// PIN's
#define MASTER_EN_PIN            7 // Eingang fur PinChange-Interrupt vom Master

#define SUB_EN_PORT              PORTE // Gate-Zugang zu EE und RAM fuer Memory-Zugriffe  des Sub
#define SUB_EN_DDR               DDRE  // mit RAM_CS_HI, EE_CS_HI
// PIN's
#define SUB_EN_PIN               0

#define USB_PORT            PORTD   //
#define USB_DDR             DDRD    //
#define USB_PIN             PIND    //
// PIN's
#define USB_DETECT_PIN      3



#define EEPROM_WRITE_BYTE_TASK     1
#define EEPROM_WRITE_PAGE_TASK     2
#define EEPROM_READ_BYTE_TASK      3
#define EEPROM_READ_PAGE_TASK      4
#define EEPROM_AUSGABE_TASK        5

#define EEPROM_WRITE_START_OK    0xB0


// RAM-Tasks

#define READ_TASKADRESSE         0x1FA     // RAM_Adresse fuer  Task-Auftrag von PPM an RC_LCD
#define READ_TASKDATA            0x1FB  

#define WRITE_TASKADRESSE        0x1F0     // RAM_Adresse fuer Task-Auftrag von RC_LCD an PPM
#define WRITE_TASKDATA           0x1F1

#define RAM_SEND_PPM_TASK         2 // PPM soll Status lesen (Auftrag AN PPM) In PPM: RAM_RECV_LCD_TASK
#define RAM_RECV_PPM_TASK         1 // LCD soll Status lesen (Auftrag VON PPM) In PPM: RAM_SEND_LCD_TASK

#define RAM_SEND_DOGM_TASK         3
#define RAM_SEND_TRIMM_TASK      4

#define RAM_TASK_OK           7 // PPM hat Task gelesen


// TRIMM_Tasks

#define RAM_TRIMM_OFFSET      0x2F0 // Startadresse fuer Trimmdaten

#define SD_DATA_SIZE 512
/* commands available in SPI mode */

/* CMD0: response R1 */
#define CMD_GO_IDLE_STATE 0x00
/* CMD1: response R1 */
#define CMD_SEND_OP_COND 0x01
/* CMD8: response R7 */
#define CMD_SEND_IF_COND 0x08
/* CMD9: response R1 */
#define CMD_SEND_CSD 0x09
/* CMD10: response R1 */
#define CMD_SEND_CID 0x0a
/* CMD12: response R1 */
#define CMD_STOP_TRANSMISSION 0x0c
/* CMD13: response R2 */
#define CMD_SEND_STATUS 0x0d
/* CMD16: arg0[31:0]: block length, response R1 */
#define CMD_SET_BLOCKLEN 0x10
/* CMD17: arg0[31:0]: data address, response R1 */
#define CMD_READ_SINGLE_BLOCK 0x11
/* CMD18: arg0[31:0]: data address, response R1 */
#define CMD_READ_MULTIPLE_BLOCK 0x12
/* CMD24: arg0[31:0]: data address, response R1 */
#define CMD_WRITE_SINGLE_BLOCK 0x18
/* CMD25: arg0[31:0]: data address, response R1 */
#define CMD_WRITE_MULTIPLE_BLOCK 0x19
/* CMD27: response R1 */
#define CMD_PROGRAM_CSD 0x1b
/* CMD28: arg0[31:0]: data address, response R1b */
#define CMD_SET_WRITE_PROT 0x1c
/* CMD29: arg0[31:0]: data address, response R1b */
#define CMD_CLR_WRITE_PROT 0x1d
/* CMD30: arg0[31:0]: write protect data address, response R1 */
#define CMD_SEND_WRITE_PROT 0x1e
/* CMD32: arg0[31:0]: data address, response R1 */
#define CMD_TAG_SECTOR_START 0x20
/* CMD33: arg0[31:0]: data address, response R1 */
#define CMD_TAG_SECTOR_END 0x21
/* CMD34: arg0[31:0]: data address, response R1 */
#define CMD_UNTAG_SECTOR 0x22
/* CMD35: arg0[31:0]: data address, response R1 */
#define CMD_TAG_ERASE_GROUP_START 0x23
/* CMD36: arg0[31:0]: data address, response R1 */
#define CMD_TAG_ERASE_GROUP_END 0x24
/* CMD37: arg0[31:0]: data address, response R1 */
#define CMD_UNTAG_ERASE_GROUP 0x25
/* CMD38: arg0[31:0]: stuff bits, response R1b */
#define CMD_ERASE 0x26
/* ACMD41: arg0[31:0]: OCR contents, response R1 */
#define CMD_SD_SEND_OP_COND 0x29
/* CMD42: arg0[31:0]: stuff bits, response R1b */
#define CMD_LOCK_UNLOCK 0x2a
/* CMD55: arg0[31:0]: stuff bits, response R1 */
#define CMD_APP 0x37
/* CMD58: arg0[31:0]: stuff bits, response R3 */
#define CMD_READ_OCR 0x3a
/* CMD59: arg0[31:1]: stuff bits, arg0[0:0]: crc option, response R1 */
#define CMD_CRC_ON_OFF 0x3b

/* command responses */
/* R1: size 1 byte */
#define R1_IDLE_STATE 0
#define R1_ERASE_RESET 1
#define R1_ILL_COMMAND 2
#define R1_COM_CRC_ERR 3
#define R1_ERASE_SEQ_ERR 4
#define R1_ADDR_ERR 5
#define R1_PARAM_ERR 6
/* R1b: equals R1, additional busy bytes */
/* R2: size 2 bytes */
#define R2_CARD_LOCKED 0
#define R2_WP_ERASE_SKIP 1
#define R2_ERR 2
#define R2_CARD_ERR 3
#define R2_CARD_ECC_FAIL 4
#define R2_WP_VIOLATION 5
#define R2_INVAL_ERASE 6
#define R2_OUT_OF_RANGE 7
#define R2_CSD_OVERWRITE 7
#define R2_IDLE_STATE (R1_IDLE_STATE + 8)
#define R2_ERASE_RESET (R1_ERASE_RESET + 8)
#define R2_ILL_COMMAND (R1_ILL_COMMAND + 8)
#define R2_COM_CRC_ERR (R1_COM_CRC_ERR + 8)
#define R2_ERASE_SEQ_ERR (R1_ERASE_SEQ_ERR + 8)
#define R2_ADDR_ERR (R1_ADDR_ERR + 8)
#define R2_PARAM_ERR (R1_PARAM_ERR + 8)
/* R3: size 5 bytes */
#define R3_OCR_MASK (0xffffffffUL)
#define R3_IDLE_STATE (R1_IDLE_STATE + 32)
#define R3_ERASE_RESET (R1_ERASE_RESET + 32)
#define R3_ILL_COMMAND (R1_ILL_COMMAND + 32)
#define R3_COM_CRC_ERR (R1_COM_CRC_ERR + 32)
#define R3_ERASE_SEQ_ERR (R1_ERASE_SEQ_ERR + 32)
#define R3_ADDR_ERR (R1_ADDR_ERR + 32)
#define R3_PARAM_ERR (R1_PARAM_ERR + 32)
/* Data Response: size 1 byte */
#define DR_STATUS_MASK 0x0e
#define DR_STATUS_ACCEPTED 0x05
#define DR_STATUS_CRC_ERR 0x0a
#define DR_STATUS_WRITE_ERR 0x0c


