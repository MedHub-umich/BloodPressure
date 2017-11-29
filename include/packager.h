#ifndef PACAKGER_H
#define PACAKGER_H

#include "crc.h"
#include "pendingMessages.h"
#include <string.h>

#define START_BYTE 0xF0
#define RES_BYTES 0x0000
#define MAX_DATA_LENGTH 247 //1 byte
#define SOF_LOC 0
#define SIZE_LOC 1
#define SEQ_LOC 2
#define RESERVED_LOC 3
#define TYPE_LOC 5
#define DATA_LOC 6

//Type defines
#define BLOOD_PRESSURE_DATA_TYPE 1
#define ECG_DATA_TYPE 2
#define HEART_RATE_DATA_TYPE 3
#define BREATHING_RATE_DATA_TYPE 4
#define TEMPERATURE_DATA_TYPE 5


typedef struct Packager
{
	uint8_t type;
	uint8_t seq;
	uint8_t dataLength;
	char data[MAX_DATA_LENGTH];
	uint8_t currAmount;
	uint16_t firstTimeStamp;
} Packager;

int createPackage(Packager* currPackager);

int addToPackage(char* dataIn, size_t length, Packager* currPackager);

int packagerInit(uint8_t dataType, uint8_t desiredLength, Packager* currPackager);

//The packager creates a new package with the following ordering: 
// SOF Byte, Size Byte, Sequence Byte, Reserved 2 Bytes, Type Byte, Data Bytes, 2 bytes of CRC 

#endif //PACAKGER_H