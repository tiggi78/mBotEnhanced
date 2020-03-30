/*
 * License            : CC-BY-SA 3.0
 */

#include "MotorHandler.h"
#include "MeDCMotor.h"

/* Global speed */
int moveSpeed = 200;

MeDCMotor MotorL( M1 );
MeDCMotor MotorR( M2 );

void Forward()
{
    MotorL.run( -moveSpeed );
    MotorR.run( moveSpeed );
}

void Backward()
{
    MotorL.run( moveSpeed );
    MotorR.run( -moveSpeed );
}

void TurnLeft()
{
    MotorL.run( moveSpeed * 0.8 );
    MotorR.run( moveSpeed * 0.8 );
}

void TurnRight()
{
    MotorL.run( -moveSpeed * 0.8 );
    MotorR.run( -moveSpeed * 0.8 );
}

void TurnLeft2()
{
    MotorL.run( -moveSpeed / 5 );
    MotorR.run( moveSpeed );
}

void TurnRight2()
{
    MotorL.run( -moveSpeed );
    MotorR.run( moveSpeed / 5 );
}

void BackwardAndTurnLeft()
{
    MotorL.run( moveSpeed / 3 );
    MotorR.run( -moveSpeed );
}

void BackwardAndTurnRight()
{
    MotorL.run( moveSpeed );
    MotorR.run( -moveSpeed / 3 );
}

void ChangeSpeed( int spd )
{
    moveSpeed = spd;
}

int getSpeed()
{
    return moveSpeed;
}
void Stop()
{
    MotorL.run( 0 );
    MotorR.run( 0 );
}

void setMotors( int16_t leftSpeed, int16_t rightSpeed )
{
    setMotor( M_LEFT, leftSpeed );
    setMotor( M_RIGHT, rightSpeed );
}

void setMotor( enum motorType type, int16_t Speed )
{
    if( M_LEFT == type )
        MotorL.run( Speed );
    if( M_RIGHT == type )
        MotorR.run( Speed );
}
