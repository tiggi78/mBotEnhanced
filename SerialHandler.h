/*
 * License            : CC-BY-SA 3.0
 */

/*
 * File:   SerialHandler.h
 * Author: giovanni
 *
 * Created on 26 marzo 2020, 23:23
 */

#ifndef SERIALHANDLER_H
#define SERIALHANDLER_H

#include <stdint.h>
#include <Arduino.h>

extern HardwareSerial Serial;
#define BUFFER_LEN 52

void writeSerial( unsigned char c );
char readSerial( boolean& isAvailable );

unsigned char readBuffer( int16_t index );
void writeBuffer( int16_t index, unsigned char c );

void writeHead();
void writeEnd();
void callOK();

void sendByte( char c );
void sendString( String s );
char* readString( int idx, int len );

void sendFloat( float value );
float readFloat( int idx );

void sendDouble( double value );

void sendShort( double value );
short readShort( int idx );

uint8_t* readUint8( int idx, int len );

#endif /* SERIALHANDLER_H */
