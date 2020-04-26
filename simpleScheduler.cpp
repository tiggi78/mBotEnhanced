/*
 * License            : CC-BY-SA 3.0
 */

/*
 * File:   simpleScheduler.cpp
 * Author: giovanni
 *
 * Created on 19 aprile 2020, 01:50
 */

#include "simpleScheduler.h"
#include <Arduino.h>
#include <util/atomic.h>

uint8_t simpleScheduler::m_numberOfTasks = 0;

struct simpleScheduler::task_s simpleScheduler::m_tasks[MAX_TASKS];

#pragma GCC push_options
#pragma GCC optimize ("-O3")
void simpleScheduler::addTask( uint16_t taskTime, void ( *function )( void* arg ), void* argument )
{
    if( m_numberOfTasks < MAX_TASKS )
    {
        m_tasks[m_numberOfTasks].function = function;
        m_tasks[m_numberOfTasks].argument = argument;
        m_tasks[m_numberOfTasks].taskTime = taskTime;
        m_tasks[m_numberOfTasks].taskDuration = 0;
        m_tasks[m_numberOfTasks].counter = 0;
        ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
        {
            m_numberOfTasks++;
            // enable TIMER0 OCA interrupt
            TIMSK0 |= _BV( OCIE0A );
            TIMSK0 &= ~( _BV( OCIE0B ) );
        }
    }
}

static uint8_t simpleScheduler::getTaskDuration( uint8_t taskid )
{
    if( taskid < MAX_TASKS )
        return m_tasks[taskid].taskDuration;
    else
        return 0;
}

static void simpleScheduler::updateTasks()
{
    for( uint8_t i = 0; i < m_numberOfTasks; ++i )
    {
        ( m_tasks[i].counter )++;
        if( m_tasks[i].counter == m_tasks[i].taskTime )
        {
            if( m_tasks[i].function != NULL )
            {
                m_tasks[i].taskDuration = micros();
                m_tasks[i].function( m_tasks[i].argument );
                m_tasks[i].taskDuration = micros() - m_tasks[i].taskDuration;
            }
            m_tasks[i].counter = 0;
        }
    }
}

#define MAX_ERROR     250
#define ERROR_PER_INT 6

ISR( TIMER0_COMPA_vect )
{
    /* When Timer0 is in Fast PWM mode, seems that COMPA interrupt is called
     * with TIMER0_OVF.
     * Arduino Timer0 is set with prescaler=64 and in fast PWM mode it overflows
     * at 0xFF. With 16 Mhz clock frequency, to have 1 ms interrupt, timer0 should
     * count up to 250, so for every interrupt there is an error of 6 ticks.
     * When the cumulated error reaches (or overcome) 250, another updateTask
     * is called to increment task counter. The error is resetted subtracting
     * MAX_ERROR value
     */

    static uint8_t error = 0;
    error += ERROR_PER_INT;
    simpleScheduler::updateTasks();
    if( error >= MAX_ERROR )
    {
        error -= MAX_ERROR;
        simpleScheduler::updateTasks();
    }

}
#pragma GCC pop_options
