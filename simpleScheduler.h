/*
 * License            : CC-BY-SA 3.0
 */

/*
 * File:   simpleScheduler.h
 * Author: giovanni
 *
 * Created on 19 aprile 2020, 01:50
 */

#include <stdint.h>

#ifndef SIMPLESCHEDULER_H
#define SIMPLESCHEDULER_H
#define MAX_TASKS 4
class simpleScheduler
{
public:
    static void addTask( uint16_t taskTime, void ( *function )( void* arg ), void* argument );
    static void updateTasks();
    static uint8_t getTaskDuration( uint8_t taskid );
private:
    static uint8_t m_numberOfTasks;
    static struct task_s
    {
        void ( * function )( void* arg );
        void* argument;
        /* Tick counter for a single task. When m_counter[i] == m_taskTime[i],
         * i task is ececuted*/
        uint16_t counter;
        /* Task tick time in ms */
        uint16_t taskTime;
        /* Task last duration (us)*/
        uint16_t taskDuration;
    } m_tasks[MAX_TASKS];

    /* Avoid calling these functions */
    simpleScheduler() {};
    simpleScheduler( const simpleScheduler& orig ) {};
    virtual ~simpleScheduler() {};

};

#endif /* SIMPLESCHEDULER_H */

