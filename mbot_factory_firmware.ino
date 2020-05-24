/*************************************************************************
* File Name          : mbot_factory_firmware.ino
* Author             : Ander, Mark Yan
* Updated            : Ander, Mark Yan
* Version            : V06.01.009
* Date               : 21/06/2017
* Description        : Firmware for Makeblock Electronic modules with Scratch.
* License            : CC-BY-SA 3.0
* Copyright (C) 2013 - 2017 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
**************************************************************************/
#include <Wire.h>
#include <MeMCore.h>
#include <Arduino.h>

#include "SerialHandler.h"
#include "MotorHandler.h"
#include "simpleScheduler.h"
#include "AnalogHandler.h"

/*
 * MCU peripherals usage
 * Timer0 (8 bit)
 *   Arduino millis() handling. Fast PWM mode, /64 prescaler, 0xFF overflow
 *
 * Timer1 (16 bit)
 *   servo lib
 *
 * Timer2 (8 bit)
 *   tone()
 *   MeIR Infrared sender (receiver)
 *
 * UART
 *  Arduino buffered serial communcation (UART/USB & Bluetooth moudule) with interrupt
 *
 */

/* Device of readSensor() function */
enum device_e
{
    VERSION               = 0,
    ULTRASONIC_SENSOR     = 1,
    TEMPERATURE_SENSOR    = 2,
    LIGHT_SENSOR          = 3,
    POTENTIONMETER        = 4,
    JOYSTICK              = 5,
    GYRO                  = 6,
    SOUND_SENSOR          = 7,
    RGBLED                = 8,
    SEVSEG                = 9,
    MOTOR                 = 10,
    SERVO                 = 11,
    ENCODER               = 12,
    IR                    = 13,
    IRREMOTE              = 14,
    PIRMOTION             = 15,
    INFRARED              = 16,
    LINEFOLLOWER          = 17,
    IRREMOTECODE          = 18,
    SHUTTER               = 20,
    LIMITSWITCH           = 21,
    BUTTON                = 22,
    HUMITURE              = 23,
    FLAMESENSOR           = 24,
    GASSENSOR             = 25,
    COMPASS               = 26,
    TEMPERATURE_SENSOR_1  = 27,
    DIGITAL               = 30,
    ANALOG                = 31,
    PWM                   = 32,
    SERVO_PIN             = 33,
    TONE                  = 34,
    BUTTON_INNER          = 35,
    ULTRASONIC_ARDUINO    = 36,
    PULSEIN               = 37,
    STEPPER               = 40,
    LEDMATRIX             = 41,
    TIMER                 = 50,
    TOUCH_SENSOR          = 51,
    JOYSTICK_MOVE         = 52,
    COMMON_COMMONCMD      = 60,
    ENCODER_BOARD         = 61,
    ENCODER_PID_MOTION    = 62,
    PM25SENSOR            = 63
};

//Secondary command (COMMON_COMMONCMD)
#define SET_STARTER_MODE                 0x10
#define SET_AURIGA_MODE                  0x11
#define SET_MEGAPI_MODE                  0x12
#define GET_BATTERY_POWER                0x70
#define GET_AURIGA_MODE                  0x71
#define GET_MEGAPI_MODE                  0x72
//Read type (ENCODER_BOARD)
#define ENCODER_BOARD_POS                0x01
#define ENCODER_BOARD_SPEED              0x02
//Secondary command (ENCODER_PIN_MOTOON)
#define ENCODER_BOARD_POS_MOTION         0x01
#define ENCODER_BOARD_SPEED_MOTION       0x02
#define ENCODER_BOARD_PWM_MOTION         0x03
#define ENCODER_BOARD_SET_CUR_POS_ZERO   0x04
#define ENCODER_BOARD_CAR_POS_MOTION     0x05
//Secondary command (PM25SENSOR)
#define GET_PM1_0                        0x01
#define GET_PM2_5                        0x02
#define GET_PM10                         0x03

/* Local functions */
void parseData();
void LedsOff();

void readSensor( enum device_e device );
void runModule( enum device_e device );
void getIRCommand();

/* GLOBALS (peripehrals) */
MeRGBLed rgb( 0, 16 );

MeRGBLed internalRGB( PORT_7, 2 );

MeUltrasonicSensor ultr( PORT_3 );
MeLineFollower line( PORT_2 );

MeLEDMatrix ledMx;
MeIR ir;
MeJoystick joystick;
MeBuzzer buzzer;
MeTemperature ts;
Me7SegmentDisplay seg;

MePort generalDevice;
Servo servo;

/* TONE NOTES */

#define NTDL1    147
#define NTDL2    165
#define NTDL3    175
#define NTDL4    196
#define NTDL5    221
#define NTDL6    248
#define NTDL7    278

#define NTD1     294
#define NTD2     330
#define NTD3     350
#define NTD4     393
#define NTD5     441
#define NTD6     495
#define NTD7     556

#define NTDH1    589
#define NTDH2    661
#define NTDH3    700
#define NTDH4    786
#define NTDH5    882
#define NTDH6    990
#define NTDH7    112

/* motor_sta values */
enum motorStatus_e
{
    STOP  = 0,
    RUN_F = 0x01,
    RUN_B = 0x01 << 1,
    RUN_L = 0x01 << 2,
    RUN_R = 0x01 << 3,
} motorStatus = STOP;
enum motorStatus_e prevMotorStatus = STOP;

/* Offset for motor speeed (when pushing on remote control) */
int minSpeed = 48;
/* Multiplaction factor for motor speed (when pushing on remote control) */
int factor = 23;

/* controlFlag vales */
enum controlFlag_e
{
    BLUE_TOOTH   = 0,
    IR_CONTROLER = 1
} controlflag = IR_CONTROLER;

/* mode values */
enum modes_e
{
    MODE_A,
    MODE_B,
    MODE_C
} mode = MODE_A;

/*
typedef struct MeModule
{
    int16_t device;
    int16_t port;
    int16_t slot;
    int16_t pin;
    int16_t index;
    float values[3];
} MeModule;
*/

String mVersion = "06.01.009";

double lastTime = 0.0;
double currentTime = 0.0;

int analogs[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

uint8_t command_index = 0;

/*
 * Protocol: 0xFF 0x55 <DATALEN> <data1> <data2> ... <dataN>
 *
 */
void serialHandle()
{
    static byte index = 0;
    static byte dataLen = 0;
    static boolean isStart = false;
    static unsigned char prevc = 0;

    boolean isAvailable = false;

    char serialRead = readSerial( isAvailable );

    if( isAvailable )
    {
        unsigned char c = serialRead & 0xff;
        if( ( c == 0x55 ) && ( isStart == false ) )
        {
            if( prevc == 0xff )
            {
                index = 1;
                isStart = true;
            }
        }
        else
        {
            prevc = c;
            if( isStart )
            {
                if( index == 2 )
                {
                    dataLen = c;
                }
                else if( index > 2 )
                {
                    dataLen--;
                }
                writeBuffer( index, c );
            }
        }
        index++;
        if( index >= BUFFER_LEN )
        {
            index = 0;
            isStart = false;
        }
        if( isStart && ( dataLen == 0 ) && ( index > 3 ) )
        {
            isStart = false;
            parseData();
            index = 0;
        }
    }
}

void buzzerOn()
{
    buzzer.tone( 500, 1000 );
}

void buzzerOff()
{
    buzzer.noTone();
}

void getIRCommand()
{
    static unsigned long time = millis();
    uint8_t punti;
    if( ir.decode() )
    {
        uint32_t value = ir.value;

        time = millis();
        Serial.print( time );
        Serial.print( " | IRVALUE: " );
        String hex( value, 16 );
        Serial.println( hex );

        switch( ( value >> 16 ) & 0xff )
        {
        case IR_BUTTON_A:
            controlflag = IR_CONTROLER;
            ChangeSpeed( 220 );
            mode = MODE_A;
            Stop();
            LedsOff();
            cli();
            buzzer.tone( NTD1, 300 );
            sei();

            internalRGB.setColor( 10, 10, 10 );
            internalRGB.show();
            break;
        case IR_BUTTON_B:
            controlflag = IR_CONTROLER;
            ChangeSpeed( 200 );
            mode = MODE_B;
            Stop();
            LedsOff();
            cli();
            buzzer.tone( NTD2, 300 );
            sei();
            internalRGB.setColor( 0, 10, 0 );
            internalRGB.show();
            break;
        case IR_BUTTON_C:
            controlflag = IR_CONTROLER;
            ChangeSpeed( 200 );
            mode = MODE_C;
            Stop();
            LedsOff();
            cli();
            buzzer.tone( NTD3, 300 );
            sei();
            internalRGB.setColor( 0, 0, 10 );
            internalRGB.show();
            break;

        case IR_BUTTON_D:
            TurnLeft();
            delay( random( 2000, 5000 ) );
            Stop();
            delay( 1000 );
            punti = random( 1, 10 );
            ChangeSpeed( 150 );
            switch( punti )
            {
            case 1:
                Forward();
                break;

            case 2:
                Backward();
                break;

            case 3:
                TurnLeft();
                break;

            case 4:
                TurnRight();
                break;

            case 5:
                internalRGB.setColor( 200, 0, 0 );
                internalRGB.show();
                break;

            case 6:
                internalRGB.setColor( 140, 60, 0 );
                internalRGB.show();
                break;

            case 7:
                internalRGB.setColor( 255, 240, 0 );
                internalRGB.show();

                break;

            case 8:
                internalRGB.setColor( 0, 200, 0 );
                internalRGB.show();

                break;

            case 9:
                internalRGB.setColor( 0, 0, 200 );
                internalRGB.show();

                break;

            case 10:
                internalRGB.setColor( 92, 53, 102 );
                internalRGB.show();

                break;

            }
            delay( 1500 );
            Stop();
            LedsOff();
            cli();
            buzzer.tone( NTDL1, 200 );
            buzzer.tone( NTD1, 200 );
            buzzer.tone( NTDH1, 200 );
            sei();
            break;
        case IR_BUTTON_E:
            {
                internalRGB.setColor( random( 255 ), random( 255 ), random( 255 ) );
                internalRGB.show();
            }
            break;
        case IR_BUTTON_SETTING:
            break;
        case IR_BUTTON_PLUS:
            controlflag = IR_CONTROLER;
            if( mode == MODE_A )
            {
                motorStatus = RUN_F;
                internalRGB.setColor( 10, 10, 0 );
                internalRGB.show();
            }
            break;
        case IR_BUTTON_MINUS:
            controlflag = IR_CONTROLER;
            if( mode == MODE_A )
            {
                motorStatus = RUN_B;
                internalRGB.setColor( 10, 0, 0 );
                internalRGB.show();
            }
            break;
        case IR_BUTTON_NEXT:
            controlflag = IR_CONTROLER;
            if( mode == MODE_A )
            {
                motorStatus = RUN_R;
                internalRGB.setColor( 1, 10, 10, 0 );
                internalRGB.show();
            }
            break;
        case IR_BUTTON_PREVIOUS:
            controlflag = IR_CONTROLER;
            if( mode == MODE_A )
            {
                motorStatus = RUN_L;
                internalRGB.setColor( 2, 10, 10, 0 );
                internalRGB.show();
            }
            break;
        case IR_BUTTON_9:
            controlflag = IR_CONTROLER;
            cli();
            buzzer.tone( NTDH2, 300 );
            sei();
            ChangeSpeed( factor * 9 + minSpeed );
            break;
        case IR_BUTTON_8:
            controlflag = IR_CONTROLER;
            cli();
            buzzer.tone( NTDH1, 300 );
            sei();
            ChangeSpeed( factor * 8 + minSpeed );
            break;
        case IR_BUTTON_7:
            controlflag = IR_CONTROLER;
            cli();
            buzzer.tone( NTD7, 300 );
            sei();
            ChangeSpeed( factor * 7 + minSpeed );
            break;
        case IR_BUTTON_6:
            controlflag = IR_CONTROLER;
            cli();
            buzzer.tone( NTD6, 300 );
            sei();
            ChangeSpeed( factor * 6 + minSpeed );
            break;
        case IR_BUTTON_5:
            controlflag = IR_CONTROLER;
            cli();
            buzzer.tone( NTD5, 300 );
            sei();
            ChangeSpeed( factor * 5 + minSpeed );
            break;
        case IR_BUTTON_4:
            controlflag = IR_CONTROLER;
            cli();
            buzzer.tone( NTD4, 300 );
            sei();
            ChangeSpeed( factor * 4 + minSpeed );
            break;
        case IR_BUTTON_3:
            controlflag = IR_CONTROLER;
            cli();
            buzzer.tone( NTD3, 300 );
            sei();
            ChangeSpeed( factor * 3 + minSpeed );
            break;
        case IR_BUTTON_2:
            controlflag = IR_CONTROLER;
            cli();
            buzzer.tone( NTD2, 300 );
            sei();
            ChangeSpeed( factor * 2 + minSpeed );
            break;
        case IR_BUTTON_1:
            controlflag = IR_CONTROLER;
            cli();
            buzzer.tone( NTD1, 300 );
            sei();
            ChangeSpeed( factor * 1 + minSpeed );
            break;
        case IR_BUTTON_0:
            controlflag = IR_CONTROLER;
            cli();
            buzzer.tone( NTDH1, 200 );
            buzzer.tone( NTD1, 200 );
            buzzer.tone( NTDL1, 200 );
            sei();
            ChangeSpeed( 0 );
            internalRGB.setColor( 255, 255, 255 );
            internalRGB.show();

            break;
        }
    }
    else if( ( controlflag == IR_CONTROLER ) && ( ( millis() - time ) > 120 ) )
    {
        motorStatus = STOP;
        time = millis();
        if( mode == MODE_A )
        {
            internalRGB.setColor( 10, 10, 10 );
            internalRGB.show();
        }
        else if( mode == MODE_B )
        {
            internalRGB.setColor( 0, 10, 0 );
            internalRGB.show();
        }
        else if( mode == MODE_C )
        {
            internalRGB.setColor( 0, 0, 10 );
            internalRGB.show();
        }
    }
}

void LedsOff()
{
    internalRGB.setColor( 0, 0, 0 );
    internalRGB.show();
}

void modeA()
{
    switch( motorStatus )
    {
    case RUN_F:
        Forward();
        prevMotorStatus = motorStatus;
        break;
    case RUN_B:
        Backward();
        prevMotorStatus = motorStatus;
        break;
    case RUN_L:
        TurnLeft();
        prevMotorStatus = motorStatus;
        break;
    case RUN_R:
        TurnRight();
        prevMotorStatus = motorStatus;
        break;
    case STOP:
        if( prevMotorStatus != motorStatus )
        {
            prevMotorStatus = motorStatus;
            Stop();
            LedsOff();
        }
        break;
    }
}

void modeB()
{
    uint16_t high = 20;
    uint16_t low  = 10;
    double dist = ultr.distanceCm( false );

    uint16_t d = ( uint16_t ) dist;
    uint8_t randNumber = random( 2 );

    if( ( d > high ) || ( d == 0 ) )
    {
        Forward();
        //Serial.println( "Forward" );
    }
    else if( ( d > low ) && ( d < high ) )
    {
        switch( randNumber )
        {
        case 0:
            TurnLeft();
            delay( 300 );
            //Serial.println( "Left 300" );

            break;
        case 1:
            TurnRight();
            delay( 300 );
            //Serial.println( "Right 300" );
            break;
        }
    }
    else // (d < low)
    {
        switch( randNumber )
        {
        case 0:
            TurnLeft();
            delay( 800 );
            //Serial.println( "Left 800" );
            break;
        case 1:
            TurnRight();
            delay( 800 );
            //Serial.println( "Right 800" );
            break;
        }
    }
    delay( 100 );
}

void modeC()
{
    static int LineFollowFlag = 0;

    uint8_t val;
    val = line.readSensors();
    if( getSpeed() > 230 )
    {
        ChangeSpeed( 230 );
    }
    switch( val )
    {
    case S1_IN_S2_IN:
        Forward();
        LineFollowFlag = 10;
        break;

    case S1_IN_S2_OUT:
        Forward();
        if( LineFollowFlag > 1 )
        {
            LineFollowFlag--;
        }
        break;

    case S1_OUT_S2_IN:
        Forward();
        if( LineFollowFlag < 20 )
        {
            LineFollowFlag++;
        }
        break;

    case S1_OUT_S2_OUT:
        if( LineFollowFlag == 10 )
        {
            Backward();
        }
        if( LineFollowFlag < 10 )
        {
            TurnLeft2();
        }
        if( LineFollowFlag > 10 )
        {
            TurnRight2();
        }
        break;
    }
}

/* action values of parseData()*/
enum action_e
{
    GET  = 1,
    RUN  = 2,
    RESET = 4,
    START = 5
};

void parseData()
{
    //isStart = false;
    int idx = readBuffer( 3 );
    command_index = ( uint8_t ) idx;
    enum action_e action = ( enum action_e )readBuffer( 4 );
    enum device_e device = ( enum device_e )readBuffer( 5 );
    switch( action )
    {
    case GET:
        {
            if( device != ULTRASONIC_SENSOR )
            {
                writeHead();
                writeSerial( idx );
            }
            readSensor( device );
            writeEnd();
        }
        break;
    case RUN:
        {
            runModule( device );
            callOK();
        }
        break;
    case RESET:
        {
            //reset
            Stop();
            LedsOff();
            buzzerOff();
            callOK();
        }
        break;
    case START:
        {
            //start
            callOK();
        }
        break;
    }
}

void runModule( device_e device )
{
    //0xff 0x55 0x6 0x0 0x2 0x22 0x9 0x0 0x0 0xa
    int port = readBuffer( 6 );
    int pin = port;
    switch( device )
    {
    case MOTOR:
        {
            controlflag = BLUE_TOOTH;
            int16_t speed = readShort( 7 );
            setMotor( ( port == M1 ) ? M_LEFT : M_RIGHT, speed );
        }
        break;
    case JOYSTICK:
        {
            controlflag = BLUE_TOOTH;
            int16_t leftSpeed = readShort( 6 );
            int16_t rightSpeed = readShort( 8 );
            setMotors( leftSpeed, rightSpeed );
        }
        break;
    case RGBLED:
        {
            controlflag = BLUE_TOOTH;
            int16_t slot = readBuffer( 7 );
            int16_t idx = readBuffer( 8 );
            int16_t r = readBuffer( 9 );
            int16_t g = readBuffer( 10 );
            int16_t b = readBuffer( 11 );

            if( ( rgb.getPort() != port ) || rgb.getSlot() != slot )
            {
                rgb.reset( port, slot );
            }
            if( idx > 0 )
            {
                rgb.setColorAt( idx - 1, r, g, b );
            }
            else
            {
                rgb.setColor( r, g, b );
            }
            rgb.show();
        }
        break;
    case SERVO:
        {
            int16_t slot = readBuffer( 7 );
            pin = slot == 1 ? mePort[port].s1 : mePort[port].s2;
            int16_t v = readBuffer( 8 );
            if( ( v >= 0 ) && ( v <= 180 ) )
            {
                servo.attach( pin );
                servo.write( v );
            }
        }
        break;
    case SEVSEG:
        {
            if( seg.getPort() != port )
            {
                seg.reset( port );
            }
            float v = readFloat( 7 );
            seg.display( v );
        }
        break;
    case LEDMATRIX:
        {
            if( ledMx.getPort() != port )
            {
                ledMx.reset( port );
            }
            int16_t action = readBuffer( 7 );
            if( action == 1 )
            {
                int16_t px = readBuffer( 8 );
                int16_t py = readBuffer( 9 );
                int16_t len = readBuffer( 10 );
                char* s = readString( 11, len );
                ledMx.drawStr( px, py, s );
            }
            else if( action == 2 )
            {
                int16_t px = readBuffer( 8 );
                int16_t py = readBuffer( 9 );
                uint8_t* ss = readUint8( 10, 16 );
                ledMx.drawBitmap( px, py, 16, ss );
            }
            else if( action == 3 )
            {
                int16_t point = readBuffer( 8 );
                int16_t hours = readBuffer( 9 );
                int16_t minutes = readBuffer( 10 );
                ledMx.showClock( hours, minutes, point );
            }
            else if( action == 4 )
            {
                ledMx.showNum( readFloat( 8 ), 3 );
            }
        }
        break;
    case LIGHT_SENSOR:
        {
            if( generalDevice.getPort() != port )
            {
                generalDevice.reset( port );
            }
            int16_t v = readBuffer( 7 );
            generalDevice.dWrite1( v );
        }
        break;
    case IR:
        {
            String Str_data;
            int16_t len = readBuffer( 2 ) - 3;
            for( int16_t i = 0; i < len; i++ )
            {
                Str_data += ( char ) readBuffer( 6 + i );
            }
            ir.sendString( Str_data );
            Str_data = "";
        }
        break;
    case SHUTTER:
        {
            if( generalDevice.getPort() != port )
            {
                generalDevice.reset( port );
            }
            int v = readBuffer( 7 );
            if( v < 2 )
            {
                generalDevice.dWrite1( v );
            }
            else
            {
                generalDevice.dWrite2( v - 2 );
            }
        }
        break;
    case DIGITAL:
        {
            pinMode( pin, OUTPUT );
            int v = readBuffer( 7 );
            digitalWrite( pin, v );
        }
        break;
    case PWM:
        {
            pinMode( pin, OUTPUT );
            int v = readBuffer( 7 );
            analogWrite( pin, v );
        }
        break;
    case TONE:
        {
            int hz = readShort( 6 );
            int tone_time = readShort( 8 );
            if( hz > 0 )
            {
                buzzer.tone( hz, tone_time );
            }
            else
            {
                buzzer.noTone();
            }
        }
        break;
    case SERVO_PIN:
        {
            int v = readBuffer( 7 );
            if( ( v >= 0 ) && ( v <= 180 ) )
            {
                servo.attach( pin );
                servo.write( v );
            }
        }
        break;
    case TIMER:
        {
            lastTime = millis() / 1000.0;
        }
        break;
    default:
        break;
    }
}
void readSensor( enum device_e device )
{
    /**************************************************
        ff    55      len idx action device port slot data a
        0     1       2   3   4      5      6    7    8
        0xff  0x55   0x4 0x3 0x1    0x1    0x1  0xa
    ***************************************************/
    float value = 0.0;
    int port, slot, pin;
    port = readBuffer( 6 );
    pin = port;
    switch( device )
    {
    case ULTRASONIC_SENSOR:
        {
            /*
            if( ultr.getPort() != port )
            {
                ultr.reset( port );
            }
            */
            value = ( float ) ultr.distanceCm();
            writeHead();
            writeSerial( command_index );
            sendFloat( value );
            //Serial.println( value );

        }
        break;
    case TEMPERATURE_SENSOR:
        {
            slot = readBuffer( 7 );
            if( ( ts.getPort() != port ) || ( ts.getSlot() != slot ) )
            {
                ts.reset( port, slot );
            }
            value = ts.temperature();
            sendFloat( value );
        }
        break;
    case LIGHT_SENSOR:
    case SOUND_SENSOR:
    case POTENTIONMETER:
        {
            if( generalDevice.getPort() != port )
            {
                generalDevice.reset( port );
                pinMode( generalDevice.pin2(), INPUT );
            }
            value = generalDevice.aRead2();
            sendFloat( value );
        }
        break;
    case JOYSTICK:
        {
            slot = readBuffer( 7 );
            if( joystick.getPort() != port )
            {
                joystick.reset( port );
            }
            value = joystick.read( slot );
            sendFloat( value );
        }
        break;
    case IR:
        {
//      if(ir.getPort() != port)
//      {
//        ir.reset(port);
//      }
//      if(irReady)
//      {
//        sendString(irBuffer);
//        irReady = false;
//        irBuffer = "";
//      }
        }
        break;
    case IRREMOTE:
        {
//      unsigned char r = readBuffer(7);
//      if((millis()/1000.0 - lastIRTime) > 0.2)
//      {
//        sendByte(0);
//      }
//      else
//      {
//        sendByte(irRead == r);
//      }
//      irRead = 0;
//      irIndex = 0;
        }
        break;
    case IRREMOTECODE:
        {
//      sendByte(irRead);
//      irRead = 0;
//      irIndex = 0;
        }
        break;
    case PIRMOTION:
        {
            if( generalDevice.getPort() != port )
            {
                generalDevice.reset( port );
                pinMode( generalDevice.pin2(), INPUT );
            }
            value = generalDevice.dRead2();
            sendFloat( value );
        }
        break;
    case LINEFOLLOWER:
        {
            if( generalDevice.getPort() != port )
            {
                generalDevice.reset( port );
                pinMode( generalDevice.pin1(), INPUT );
                pinMode( generalDevice.pin2(), INPUT );
            }
            value = generalDevice.dRead1() * 2 + generalDevice.dRead2();
            sendFloat( value );
        }
        break;
    case LIMITSWITCH:
        {
            slot = readBuffer( 7 );
            if( ( generalDevice.getPort() != port ) || ( generalDevice.getSlot() != slot ) )
            {
                generalDevice.reset( port, slot );
            }
            if( slot == 1 )
            {
                pinMode( generalDevice.pin1(), INPUT_PULLUP );
                value = !generalDevice.dRead1();
            }
            else
            {
                pinMode( generalDevice.pin2(), INPUT_PULLUP );
                value = !generalDevice.dRead2();
            }
            sendFloat( value );
        }
        break;
    case BUTTON_INNER:
        {
            pin = analogs[pin];
            char s = readBuffer( 7 );
            pinMode( pin, INPUT );
            boolean currentPressed = !( analogRead( pin ) > 10 );
            sendByte( s ^ ( currentPressed ? 1 : 0 ) );
            //buttonPressed = currentPressed;
        }
        break;
    case  GYRO:
        {
//      int axis = readBuffer(7);
//      gyro.update();
//      if(axis == 1)
//      {
//        value = gyro.getAngleX();
//        sendFloat(value);
//      }
//      else if(axis == 2)
//      {
//        value = gyro.getAngleY();
//        sendFloat(value);
//      }
//      else if(axis == 3)
//      {
//        value = gyro.getAngleZ();
//        sendFloat(value);
//      }
        }
        break;
    case VERSION:
        {
            sendString( mVersion );
        }
        break;
    case DIGITAL:
        {
            pinMode( pin, INPUT );
            sendFloat( digitalRead( pin ) );
        }
        break;
    case ANALOG:
        {
            pin = analogs[pin];
            pinMode( pin, INPUT );
            sendFloat( analogRead( pin ) );
        }
        break;
    case TIMER:
        {
            sendFloat( currentTime );
        }
        break;
    default:
        break;
    }
}

#define INTERNAL_LED 13
#define INTERNAL_BUTTON   7

void blink( void* arg )
{
    static bool toggle = TRUE;
    toggle = !toggle;
    digitalWrite( INTERNAL_LED, toggle );
}

void setup()
{
    delay( 5 );
    Stop();
    LedsOff();

    pinMode( INTERNAL_LED, OUTPUT );
    pinMode( INTERNAL_BUTTON, INPUT );
    digitalWrite( INTERNAL_LED, HIGH );
    delay( 300 );
    digitalWrite( INTERNAL_LED, LOW );

    /* Initialize random seed one time */
    randomSeed( analogRead( A6 ) );

    AnalogHandler::begin();

    simpleScheduler::addTask( 250, MeUltrasonicSensor::triggerTask, &ultr );
    simpleScheduler::addTask( 100, AnalogHandler::task );

    internalRGB.reset( PORT_7, SLOT2 );

    internalRGB.setColor( 10, 0, 0 );
    internalRGB.show();

    //buzzer.tone( NTD1, 300 );

    delay( 300 );

    internalRGB.setColor( 0, 10, 0 );
    internalRGB.show();

    //buzzer.tone( NTD2, 300 );
    delay( 300 );

    internalRGB.setColor( 0, 0, 10 );
    internalRGB.show();

    //buzzer.tone( NTD3, 300 );
    delay( 300 );

    internalRGB.setColor( 10, 10, 10 );
    internalRGB.show();

    Serial.begin( 115200 );

    buzzer.noTone();

    ir.begin();

    Serial.println( "[mBotEnhanced]" );
    Serial.print( "Version: " );
    Serial.println( mVersion );

    //ledMx.setBrightness( 6 );
    //ledMx.setColorIndex( 1 );

}

void loop()
{
    static boolean buttonWasPressed = false;

    getIRCommand();
    serialHandle();
    uint32_t elapsed = millis();
    static uint32_t lastPrinted = 0;
    float Vref = 5.0;
    if( elapsed - lastPrinted > 200 )
    {
        lastPrinted = elapsed;

        Serial.print( "ELAPSED: " );
        Serial.println( elapsed );

        Serial.print( "MODE: " );
        Serial.println( char( 'A' + mode ) );

        Serial.println( "TASKS" );
        for( uint8_t i = 0; i < MAX_TASKS; ++i )
        {
            Serial.print( i );
            Serial.print( ": " );
            Serial.println( simpleScheduler::getTaskDuration( i ) );
        }

        Serial.println( "ANALOGS" );
        Vref = 1.1 * 1023.0 / AnalogHandler::getValue( 9 );
        Serial.print( "Vref: " );
        Serial.println( Vref );

        for( uint8_t i = 0; i < MAX_ANALOGS; ++i )
        {
            Serial.print( i );
            Serial.print( ": " );
            Serial.print( AnalogHandler::getValue( i ) );
            Serial.print( "\t" );
            Serial.println( AnalogHandler::getValue( i ) * ( Vref / 1023.0 ) );

        }
        Serial.print( "DISTANCE [cm]: " );
        Serial.println( ultr.distanceCm( false ) );

    }

    /* Check if internal button was pressed. Executes actions only one time
     * even if the button remain pressed
     */
    //bool buttonPressed = !( analogRead( INTERNAL_BUTTON ) > 100 );
    bool buttonPressed = !( AnalogHandler::getValue( INTERNAL_BUTTON ) > 100 );

    if( buttonPressed != buttonWasPressed )
    {
        buttonWasPressed = buttonPressed;
        /* If button is pressed now, change mode*/
        if( buttonPressed == true )
        {
            if( mode == MODE_A )
            {
                mode = MODE_B;
                ChangeSpeed( 200 );
                Stop();
                LedsOff();
                cli();
                buzzer.tone( NTD2, 50 );
                sei();
                buzzer.noTone();

                internalRGB.setColor( 0, 10, 0 );
                internalRGB.show();
            }
            else if( mode == MODE_B )
            {
                mode = MODE_C;
                ChangeSpeed( 200 );
                Stop();
                LedsOff();
                cli();
                buzzer.tone( NTD2, 50 );
                sei();
                buzzer.noTone();

                internalRGB.setColor( 0, 0, 10 );
                internalRGB.show();
            }
            else if( mode == MODE_C )
            {
                mode = MODE_A;
                ChangeSpeed( 220 );
                Stop();
                LedsOff();
                cli();
                buzzer.tone( NTD1, 50 );
                sei();
                buzzer.noTone();

                internalRGB.setColor( 10, 10, 10 );
                internalRGB.show();
            }
        }
    }
    switch( mode )
    {
    case MODE_A:
        modeA();
        break;
    case MODE_B:
        modeB();
        break;
    case MODE_C:
        modeC();
        break;
    }
}
