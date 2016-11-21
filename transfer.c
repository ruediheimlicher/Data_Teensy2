//
//  transfer.c
//  Data_Teensy
//
//  Created by Ruedi Heimlicher on 19.11.2016.
//
//

#include <stdio.h>
//#include "defines.h"

#include "chan_n/mmc_avr.h"

uint8_t transferBlock(uint8_t start)
{
   uint8_t transferbuffer[512];
   uint8_t i=0;
   
   {
      uint8_t  writeerr = mmc_disk_read ((void*)transferbuffer,1,	1);
   
      
   }
   
   return 0;
}