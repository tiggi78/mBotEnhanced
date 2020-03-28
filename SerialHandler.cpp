/*
 * License            : CC-BY-SA 3.0
 */

#include "SerialHandler.h"

/* action values of parseData()*/
#define GET 1
#define RUN 2
#define RESET 4
#define START 5

char buffer[BUFFER_LEN];


void writeSerial( unsigned char c )
{
    Serial.write( c );
}

char readSerial( boolean& isAvailable )
{
    char serialRead = 0;
    isAvailable = false;
    if( Serial.available() > 0 )
    {
        isAvailable = true;
        serialRead = Serial.read();
    }
    return serialRead;
}

unsigned char readBuffer( int16_t index )
{
    return buffer[index];
}

void writeBuffer( int16_t index, unsigned char c )
{
    buffer[index] = c;
}

void writeHead()
{
    writeSerial( 0xff );
    writeSerial( 0x55 );
}

void writeEnd()
{
    Serial.println();
}

