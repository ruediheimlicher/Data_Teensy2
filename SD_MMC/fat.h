/*
 * 	Doku, siehe http://www.mikrocontroller.net/articles/AVR_FAT32
 *  Neuste Version: http://www.mikrocontroller.net/svnbrowser/avr-fat32/
 *	Autor: Daniel R.
 */



#ifndef _FAT_H
  #define _FAT_H

	// #######################################################################################################################
	// "daten" ketten siehe doku...
	// 1. fat_getFreeRowOfCluster -> fat_getFreeRowOfDir -> fat_makeRowDataEntry -> fat_makeFileEntry -> fat_writeSector  "eintrag gemacht !!"
	// 2. fat_loadSector -> fat_loadRowOfSector -> fat_loadFileDataFromCluster -> fat_loadFileDataFromDir (-> fat_cd)   "daten chain"

	// #######################################################################################################################
	// funktionen

  extern unsigned long fat_clustToSec(unsigned long);										// rechnet cluster zu 1. sektor des clusters um
  extern unsigned long fat_secToClust(unsigned long sec);									// rechnet sektor zu cluster um!
  extern unsigned long fat_getNextCluster(unsigned long oneCluster);						// fat auf naechsten, verketteten cluster durchsuchen
  extern long long fat_getFreeBytes(void);													// berechnet den freien platz der karte in bytes!
  extern unsigned char fat_writeSector(unsigned long sec);									// schreibt sektor auf karte
  extern unsigned char fat_loadSector(unsigned long sec);									// laed Uebergebenen absoluten sektor
  extern unsigned char fat_loadFileDataFromDir(char name []);								// durchsucht das aktuelle directory
  extern unsigned char fat_loadFatData(void);												// laed fat daten
  extern void 	fat_loadRowOfSector(unsigned short row);									// laed reihe des geladen sektors auf struct:file
  extern void 	fat_setCluster( unsigned long cluster, unsigned long content);				// setzt cluster inhalt in der fat
  extern void 	fat_delClusterChain(unsigned long startCluster);							// loescht cluster-chain in der fat
  extern void 	fat_makeFileEntry(char name [],unsigned char attrib);						// macht einen datei/ordner eintrag
  extern void 	fat_getFreeClustersInRow(unsigned long offsetCluster);						// sucht zusammenhaengende freie cluster aus der fat
  extern void 	fat_getFatChainClustersInRow( unsigned long offsetCluster);					// sucht fat-chain cluster die zusammenhaengen
  extern void 	fat_setClusterChain(unsigned long startCluster, unsigned long endCluster);	// verkettet cluster zu einer cluster-chain
  void 			fat_makeSfnDataEntry(char name [],unsigned char attrib,unsigned long cluster,unsigned long length);

  // #######################################################################################################################
  // variablen

#endif


 



