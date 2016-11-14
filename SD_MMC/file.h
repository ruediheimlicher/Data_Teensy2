/*
 * 	Doku, siehe http://www.mikrocontroller.net/articles/AVR_FAT32
 *  Neuste Version: http://www.mikrocontroller.net/svnbrowser/avr-fat32/
 *	Autor: Daniel R.
 */


#ifndef _FILE_H

  #define _FILE_H

  //#######################################################################################################################
  // funktionen

  extern char           ffread(void);											// liest byte-weise aus der datei (puffert immer 512 bytes zwischen)
  extern void           ffwrite( unsigned char c);								// schreibt ein byte in die geoeffnete datei
  extern void           ffwrites(char *s );							// schreibt string auf karte
  extern void           ffwriten( unsigned char *s, unsigned short n );			// schreibt n bytes aus s auf die karte. maximal 2^16 stueck wegen datentyp von n !
  extern unsigned char 	ffopen(char name[], unsigned char rw_flag);	// kann immer nur 1 datei bearbeiten.
  extern unsigned char 	ffclose(void);											// muss aufgerufen werden bevor neue datei bearbeitet wird.
  extern void           ffseek(unsigned long offset);							// setzt zeiger:bytesOfSec auf position in der geöffneten datei.
  extern unsigned char 	ffcd(char name[]);							// wechselt direktory
  extern void           ffls(fptr_t uputs_ptr);									// zeigt direktory inhalt an, muss zeiger auf eine ausgabe funktion uebergeben bekommen
  extern unsigned char 	ffcdLower(void);										// geht ein direktory zurueck, also cd.. (parent direktory)
  extern unsigned char 	ffrm(char name[]);							// loescht datei aus aktuellem verzeichniss.
  extern void           ffmkdir(char *name); 						// legt ordner in aktuellem verzeichniss an.
  extern void           fflushFileData(void);									// updatet datei informationen. sichert alle noetigen informationen!
  extern unsigned char 	ffileExsists ( unsigned char name[]);					// prueft ob es die datei im aktuellen verzeichnis gibt. ffopen wuerde die datei direkt anlegen falls es sie noch nicht gibt!
  
  //#######################################################################################################################
  



#endif




