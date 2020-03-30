/*
 * License            : CC-BY-SA 3.0
 */
#include "SerialHandler.h"

union
{
    byte byteVal[4];
    float floatVal;
    long longVal;
} val;

union
{
    byte byteVal[8];
    double doubleVal;
} valDouble;

union
{
    byte byteVal[2];
    short shortVal;
} valShort;

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

void callOK()
{
    writeSerial( 0xff );
    writeSerial( 0x55 );
    writeEnd();
}

void sendByte( char c )
{
    writeSerial( 1 );
    writeSerial( c );
}

void sendString( String s )
{
    int l = s.length();
    writeSerial( 4 );
    writeSerial( l );
    for( int i = 0; i < l; i++ )
    {
        writeSerial( s.charAt( i ) );
    }
}

//1 byte 2 float 3 short 4 len+string 5 double
void sendFloat( float value )
{
    writeSerial( 2 );
    val.floatVal = value;
    writeSerial( val.byteVal[0] );
    writeSerial( val.byteVal[1] );
    writeSerial( val.byteVal[2] );
    writeSerial( val.byteVal[3] );
}

void sendShort( double value )
{
    writeSerial( 3 );
    valShort.shortVal = value;
    writeSerial( valShort.byteVal[0] );
    writeSerial( valShort.byteVal[1] );
    writeSerial( valShort.byteVal[2] );
    writeSerial( valShort.byteVal[3] );
}

void sendDouble( double value )
{
    writeSerial( 5 );
    valDouble.doubleVal = value;
    writeSerial( valDouble.byteVal[0] );
    writeSerial( valDouble.byteVal[1] );
    writeSerial( valDouble.byteVal[2] );
    writeSerial( valDouble.byteVal[3] );
    writeSerial( valDouble.byteVal[4] );
    writeSerial( valDouble.byteVal[5] );
    writeSerial( valDouble.byteVal[6] );
    writeSerial( valDouble.byteVal[7] );
}

short readShort( int idx )
{
    valShort.byteVal[0] = readBuffer( idx );
    valShort.byteVal[1] = readBuffer( idx + 1 );
    return valShort.shortVal;
}

float readFloat( int idx )
{
    val.byteVal[0] = readBuffer( idx );
    val.byteVal[1] = readBuffer( idx + 1 );
    val.byteVal[2] = readBuffer( idx + 2 );
    val.byteVal[3] = readBuffer( idx + 3 );
    return val.floatVal;
}

char* readString( int idx, int len )
{
#define RECEIVE_STR_LEN 20
    /* room for '\0' */
    static char _receiveStr[RECEIVE_STR_LEN + 1] = {};
    if( len >= RECEIVE_STR_LEN )
    {
        len = RECEIVE_STR_LEN;
    }

    for( int i = 0; i < len; i++ )
    {
        _receiveStr[i] = readBuffer( idx + i );
    }
    _receiveStr[len] = '\0';
    return _receiveStr;
}

uint8_t* readUint8( int idx, int len )
{
#define RECEIVE_U8_LEN 16
    static uint8_t _receiveUint8[RECEIVE_U8_LEN] = {};
    if( len > RECEIVE_U8_LEN )
    {
        len = RECEIVE_U8_LEN;
    }
    for( int i = 0; i < len; i++ )
    {
        _receiveUint8[i] = readBuffer( idx + i );
    }
    return _receiveUint8;
}
