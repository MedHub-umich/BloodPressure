#include "crc.h"

unsigned short crc16(unsigned char *data, size_t length) { 
   size_t count;
   unsigned int crc = SEED;
   unsigned int temp;

   for (count = 0; count < length; ++count)
   {      
     temp = (*data++ ^ (crc >> 8)) & 0xff;
     crc = crc_table[temp] ^ (crc << 8);
   }

   return (unsigned short)(crc ^ FINAL);

} 
