/*
 * License            : CC-BY-SA 3.0
 */

/*
 * File:   analogHandler.cpp
 * Author: giovanni
 *
 * Created on 2 maggio 2020, 01:31
 */

#include "AnalogHandler.h"
#include <util/atomic.h>

#define DISABLED_ANALOG 255
uint8_t AnalogHandler::m_analogs[MAX_ANALOGS] = {0, 1, 255, 255, 4, 5, 6, 7, 8, 14, 15};
volatile uint16_t AnalogHandler::m_analogValues[MAX_ANALOGS] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
volatile uint8_t AnalogHandler::m_currentIndex = 0;

#pragma GCC push_options
#pragma GCC optimize ("-O3")
ISR( ADC_vect )
{
    /* ISR TIME*/
    //AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex] = micros();
    uint8_t low  = ADCL;
    uint8_t high = ADCH;

    /* Store curret value*/
    AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex] = ( high << 8 ) | low;

    /* conversion time */
    //AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex] = micros() - AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex];

    /* Increment index ignoring disabled analogs */
    do
    {
        AnalogHandler::m_currentIndex++;
    }
    while( ( AnalogHandler::m_currentIndex < MAX_ANALOGS ) &&
            ( AnalogHandler::m_analogs[AnalogHandler::m_currentIndex] == DISABLED_ANALOG ) );

    /* if all analogs was read, disable interrupts */
    if( AnalogHandler::m_currentIndex == MAX_ANALOGS )
    {
        ADCSRA &= ~( 1 << ADIE );
    }
    else
    {
        /* trigger new conversion on new channel */
        ADMUX = ( ( ADMUX & 0xF0 ) | AnalogHandler::m_analogs[AnalogHandler::m_currentIndex] );
        //AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex] = micros();
        /* start the conversion */
        ADCSRA |= ( 1 << ADSC );
    }
    /* ISR TIME*/
    //AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex - 1] = micros() - AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex - 1];

}
#pragma GCC pop_options

void AnalogHandler::begin()
{
    // Vcc Reference
    ADMUX = ( 1 << REFS0 );
    /* clear and enable  interrupts */
    ADCSRA &= ~( 1 << ADIF );
    ADCSRA |= ( 1 << ADIE );
}

void AnalogHandler::task( void* arg )
{
    AnalogHandler::m_currentIndex = 0;
    /* TODO: ignore if input 0 is DIABLED */
    ADMUX = ( ( ADMUX & 0xF0 ) | AnalogHandler::m_analogs[AnalogHandler::m_currentIndex] );
    //start the conversion
    ADCSRA |= ( 1 << ADSC );
    /* clear and enable  interrupts */
    ADCSRA &= ~( 1 << ADIF );
    ADCSRA |= ( 1 << ADIE );
}
uint16_t AnalogHandler::getValue( uint8_t index )
{
    uint16_t value = 0;

    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
        value = m_analogValues[index];
    }
    return value;
}
