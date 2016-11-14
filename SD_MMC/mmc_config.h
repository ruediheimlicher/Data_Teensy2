/*
 * 	Doku, siehe http://www.mikrocontroller.net/articles/AVR_FAT32
 * 	Neuste Version: http://www.mikrocontroller.net/svnbrowser/avr-fat32/
 * 	Autor: Daniel R.
 */


#ifndef CONFIG_H_
	#define CONFIG_H_

	#include <ctype.h>
	#include <stdint.h>
	//#include "stm32f10x.h"	 // wegen den hardware pins SMT32...
	#include <avr/io.h>		 // wegen den hardware pins AVR

	//#####################################################################################
	// BOOL DEFINITIONEN
	
	#define TRUE 	0x01
	#define FALSE 	0x00

	//#####################################################################################
	// FLAGS

	// flags fuer ffopen (sind die rueckgabewerte von ffopen, auf diese wird geprueft..)
	#define MMC_FILE_OPENED 	0	// datei existierte und wurde geoeffnet.
	#define MMC_FILE_CREATED 	1	// datei angelegt.
	#define MMC_FILE_ERROR		2	// datei existiert nicht/nicht angelegt.


	//#####################################################################################
	// WICHTIGE SCHALTER

	// schalter die den funktionsumfang aendern bzw, den funktionsumfang einiger funktionen :)
	#define MMC_WRITE 			TRUE	// TRUE, dann mit write unterstuetzung, wenn FALSE dann read only !
	#define MMC_OVER_WRITE 		FALSE	// TRUE und MMC_WRITE TRUE, dann kann ffwrite dateien ueberschreiben, wenn FALSE dann nur normales schreiben. um an eine datei anzuhaengen ist MMC_OVER_WRITE TRUE nicht noetig!
	#define MMC_MULTI_BLOCK 	FALSE	// TRUE und MMC_OVER_WRITE FALSE, dann werden multiblock schreib/lese funktionen benutzt. ist schneller, wird aber moeglicherweise nicht von allen karten unterstützt. wenn FALSE ist normale operation
	#define MMC_ENDIANNESS_LITTLE 	TRUE	// TRUE, dann ist der code auf littleendian ausgelegt. AVR ist littleendian. code ist auf littleendian optimiert!! siehe: http://de.wikipedia.org/wiki/Endianness
	#define MMC_LFN_SUPPORT		FALSE	// TRUE, dann mit unterstuetzung fuer lange dateinamen. kostet wenn read und write benutzt wird um die 800 bytes flash...
	#define MMC_RM_FILES_ONLY	FALSE	// TRUE ,MMC_WRITE TRUE und MMC_RM TRUE, dann wird die funktion ffrm so mit kompiliert, dass sie nur dateien loeschen kann. wenn FALSE und MMC_WRITE TRUE, dann kann die funktion dateien und ordner rekursiv loeschen !

	// schalter die explizit funktionen mit kompilieren oder nicht!
	#define MMC_TIME_STAMP 		FALSE 	// TRUE, dann werden die funktionen fat_getTime und fat_getFreeBytes mit kompiliert. siehe auch abschnitt: ZEIT FUNKTIONEN, weiter unten
	#define MMC_RM 				TRUE	// TRUE und MMC_WRITE TRUE, dann wird die funktion ffrm mit kompiliert.
	#define MMC_SEEK			TRUE	// TRUE,dann wird die funktion ffseek mit kompiliert. mit dieser funktion kann man in einer geoeffneten datei vor und zurueck spulen. nur in kombination mit MMC_OVER_WRITE TRUE kann in einer datei ueberschrieben werden.
	#define MMC_MKDIR			TRUE	// TRUE und MMC_WRITE TRUE, dann wird die funktion ffmkdir mit kompiliert. mit dieser funktion kann man ordner anlegen.
	#define MMC_GET_FREE_BYTES	FALSE	// TRUE, dann wird die funkton fat_getFreeBytes mit kompiliert. mit dieser funktion kann der freie platzt auf der karte ermittelt werden
	#define MMC_LS				TRUE	// TRUE, dann wird die funktion ffls mit kompiliert. mit dieser funkion kann man die dateien auf der karte anzeigen lassen
	#define MMC_CD				TRUE	// TRUE, dann werden die funktionen ffcd und ffcdLower mit kompiliert. mit diesen funktionen kann man in ein verzeichnis wechseln oder aus einem verzeichnis ein verzeichnis hoeher wechseln.
	#define MMC_FILE_EXSISTS	FALSE	// TRUE, dann wird die funktion ffileExsists mit kompiliert. mit dieser funktion kann geprueft werden, ob es die datei im aktuellen verzeinis gibt !
	#define MMC_WRITE_STRING	TRUE	// TRUE und MMC_WRITE TRUE, dann wird die funktion ffwrites mit kompiliert. mit dieser funktion koennen strings auf die karte geschrieben werden.
	#define MMC_WRITEN			FALSE	// TRUE	und MMC_WRITE TRUE, dann wird die funktion ffwriten mit kompiliert.	mit dieser funktion koennen mehrere zeichen auf die karte geschrieben werden!

	// vorsicht, da die variable die die sektoren zaehlt ein short ist (MMC_MAX_CLUSTERS_IN_ROW*fat.secPerClust) !!
	#define MMC_MAX_CLUSTERS_IN_ROW 256 	// gibt an wie viele cluster am stueck ohne fat-lookup geschrieben bzw gelesen werden können, wenn die fat nicht fragmentiert ist !


	//#####################################################################################
	// SPI EINSTELLUNGEN

	// wenn MAX_SPEED FALSE dann wird die SPI bus geschwindigkeit nicht hoch gesetzt. ist zum testen von hardware die nicht richtig läuft
	#define MMC_MAX_SPEED 		TRUE
	#define MMC_STATUS_INFO 	FALSE

	// software spi? wenn TRUE muessen in der mmc.h noch die pins eingestellt werden ueber die die spi kommunikation laufen soll
	// es werden nur pins an einem port unterstuetzt !
	#define MMC_SOFT_SPI 	FALSE


	//#####################################################################################
	// TYPDEFINITIONEN

	// pointer typ: fptr, auf eine funktion mit einem arugment: arg und ohne rueckgabewert. arg ist vom typ: "unsigned char *" !
	// wird fuer die ausgabe benutzt, so kann in der main auch einfach eine eigene
	// uart routine verwendet werden! beispiel aufruf von ffls: "ffls(uputs);" wobei uputs eine funktion
	// der form "uputs(unsigned char *s)" ist
	typedef void(*fptr_t)(char* arg);


	// in main.c definiert. konvertiert einen long zu string, wird nur von ls benoetigt
	#if (MMC_LS == TRUE)
		extern char *ltostr(long num, char *string, short max_chars, unsigned char base);
	#endif

	//#####################################################################################
	// ZEIT FUNKTIONEN

	// siehe fat.c/fat.h


	//#########################################################################################################################################
	// DIE DATENSTRUKT DAKLARATIONEN (urspruenglich waren die in fat.c)

	extern struct Fat_t{				// fat daten (1.cluster, root-dir, dir usw.)
		unsigned long 	lastSector;		// nicht der aktuell geladene sektor, sondern der davor
		unsigned char 	bufferDirty;	// puffer wurde beschrieben, sector muss geschrieben werden bevor er neu geladen wird
		unsigned long 	dir; 		  	// Direktory zeiger rootDir=='0' sonst(1.Cluster des dir; start auf root)
		unsigned long 	rootDir;		// Sektor(f16)/Cluster(f32) nr root directory
		unsigned long 	dataDirSec;		// Sektor nr data area
		unsigned long 	fatSec;	 		// Sektor nr fat area
		unsigned char 	secPerClust;	// anzahl der sektoren pro cluster
		unsigned char 	fatType;		// fat16 oder fat32 (16 oder 32)
		unsigned char 	card_type;		// der SD/MMC karten typ. siehe auch mmc.c
		unsigned char 	sector[512];	// der puffer fuer sektoren !
	}fat;

	extern struct File_t{					// datei infos usw.
		unsigned short 	cntOfBytes;		// seek + cntOfBytes ist position in datei
		unsigned long 	seek;			// seek + cntOfBytes ist position in datei
		unsigned long 	currentSectorNr;// aktuell zu bearbeitender sektor bei einer geoeffneten datei
		unsigned long 	length;			// 28,4			datei Laenge (4-byte)
		char 			*name;			// zeiger auf langen dateinamen (lfn)...ueber den wird auch im ordner nach der datei gesucht. der kurze dateiname (sfn) leitet sich aus dem lfn ab!
		unsigned char 	row;			// reihe im sektor in der die datei infos stehen (reihe 0-15)
		unsigned long 	firstCluster;	// 20,2 /26,2	datei 1.cluster hi,low(moeglicherweise der einzige)	(4-byte)
		unsigned long 	entrySector;	// der sektor in dem der sfn dateieintrag ist.
	}file;

	extern struct Chain_t{
		unsigned long 	startSectors;	// erster freier/verketteter sektor
		unsigned short	 	cntSecs;		// anzahl der freien/verketteten in einer reihe
		unsigned long 	lastCluster;	// letzter bekannter cluster der teilkette davor !
	}chain;

	////#define loop_until_bit_is_set(x,y)	while( !(x&(1<<y)) ){}

#endif
