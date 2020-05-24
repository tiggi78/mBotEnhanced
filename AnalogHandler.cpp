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
volatile uint16_t AnalogHandler::m_analogValues[MAX_ANALOGS] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };
volatile uint8_t AnalogHandler::m_currentIndex = 0;
volatile uint8_t AnalogHandler::ADMUXRef = 0;


#pragma GCC push_options
#pragma GCC optimize ("-O3")
ISR( ADC_vect )
{
    static uint8_t firstAfterChange = 1;
    /* ISR TIME*/
    //AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex] = micros();
    uint8_t low  = ADCL;
    uint8_t high = ADCH;

    /* conversion time */
    //AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex] = micros() - AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex];
    if( ( --firstAfterChange ) > 0 )
    {
        /* Discard this reading and re-read this input after a reference change*/
        //AnalogHandler::m_currentIndex--;
        //firstAfterChange = false;
        /* start the conversion */
        ADCSRA |= ( 1 << ADSC );
    }
    else
    {
        /* Store current value*/
        AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex] = ( high << 8 ) | low;
        firstAfterChange = 1;


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
            uint8_t oldADMUXRef = AnalogHandler::ADMUXRef;

            if( ( AnalogHandler::m_analogs[AnalogHandler::m_currentIndex] == 8 )
                    //||
                    //( AnalogHandler::m_analogs[AnalogHandler::m_currentIndex] == 14 )
              )
            {
                // 1.1 Vref for Temperature sensor
                AnalogHandler::ADMUXRef |= ( uint8_t )( 1 << REFS1 );
            }
            else
            {
                AnalogHandler::ADMUXRef &= ~( ( uint8_t )( 1 << REFS1 ) );
            }
            /* trigger new conversion on new channel */
            ADMUX = ( AnalogHandler::ADMUXRef | AnalogHandler::m_analogs[AnalogHandler::m_currentIndex] );

            /* if VREF for ADMux has changed, next value will be wrong. Read it more time to wait setting time*/
            if( ( oldADMUXRef != AnalogHandler::ADMUXRef ) ||
                    /* also reading internal bandgap need some settling time*/
                    ( AnalogHandler::m_analogs[AnalogHandler::m_currentIndex] == 14 ) )
            {
                /* bandgap reading need only 10. Temperatire sensor needs more*/
                firstAfterChange = 50;
            }
            //AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex] = micros();
            /* start the conversion */
            ADCSRA |= ( 1 << ADSC );
        }
    }
    /* ISR TIME*/
//AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex - 1] = micros() - AnalogHandler::m_analogValues[AnalogHandler::m_currentIndex - 1];

}
#pragma GCC pop_options

void AnalogHandler::begin()
{
    ADMUXRef = ( 1 << REFS0 );// | ( 1 << REFS1 );
    // Vcc Reference
    ADMUX = ADMUXRef;
    //ADMUX |= ( 1 << REFS1 );
    ADCSRA &= ~( 1 << ADIF );
    ADCSRA |= ( 1 << ADIE );
}

void AnalogHandler::task( void* arg )
{
    m_currentIndex = 0;
    /* TODO: ignore if input 0 is DISABLED */
    ADMUX = ( ADMUXRef | AnalogHandler::m_analogs[AnalogHandler::m_currentIndex] );
    /* clear and enable  interrupts */
    ADCSRA &= ~( 1 << ADIF );
    ADCSRA |= ( 1 << ADIE );
    //start the conversion
    ADCSRA |= ( 1 << ADSC );

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
