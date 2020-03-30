/*
 * License            : CC-BY-SA 3.0
 */

/*
 * File:   MotorHandler.h
 * Author: giovanni
 *
 * Created on 29 marzo 2020, 22:53
 */

#ifndef MOTORHANDLER_H
#define MOTORHANDLER_H
#include <stdint.h>

enum motorType
{
    M_LEFT,
    M_RIGHT
};

void Forward();
void Backward();
void TurnLeft();
void TurnRight();
void TurnLeft2();
void TurnRight2();
void BackwardAndTurnLeft();
void BackwardAndTurnRight();
void ChangeSpeed( int16_t spd );
int16_t getSpeed();
void setMotors( int16_t leftSpeed, int16_t rightSpeed );

void setMotor( enum motorType type, int16_t Speed );

void Stop();


#endif /* MOTORHANDLER_H */

