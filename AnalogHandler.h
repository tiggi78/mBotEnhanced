/*
 * License            : CC-BY-SA 3.0
 */

/*
 * File:   analogHandler.h
 * Author: giovanni
 *
 * Created on 2 maggio 2020, 01:31
 */

#ifndef ANALOGHANDLER_H
#define ANALOGHANDLER_H
#include <stdint.h>
#include <Arduino.h>
#define MAX_ANALOGS 9

/* http://www.cyberrailguru.com/software-development/interrupts-in-c-and-avr*/
extern "C" void ADC_vect( void )  __attribute__( ( signal ) );

class AnalogHandler
{
public:
    static void begin();
    static void task( void* arg );
    static uint16_t getValue( uint8_t index );
    friend void ADC_vect();
private:
    /*
     * This vector contains analogs that must be read. If an input must not be read
     * (i.e used by another function) set to DISABLED_ANALOG so it is ignored
     */
    static uint8_t m_analogs[MAX_ANALOGS];
    volatile static uint16_t m_analogValues[MAX_ANALOGS];
    volatile static uint8_t m_currentIndex;

    AnalogHandler() {}
    AnalogHandler( const AnalogHandler& orig ) {}
    virtual ~AnalogHandler() {}

};

#endif /* ANALOGHANDLER_H */

