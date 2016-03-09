/*
 * pendulum.ino
 * This is a demo to show you the color RGB
 *
 * Copyright (c) 2016 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : Jiankai.li
 * Create Time: Mar 2016
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Wire.h>
#include <I2C_LCD.h>

#define L1Pin     A0
#define L2Pin     A1
#define L3Pin     A2

#define L1Button  2
#define L2Button  3
#define L3Button  4

#define L1Laser   5
#define L2Laser   6
#define L3Laser   7

#define L1LaserOpen()   digitalWrite(L1Laser,LOW)
#define L2LaserOpen()   digitalWrite(L2Laser,LOW)
#define L3LaserOpen()   digitalWrite(L3Laser,LOW)

#define L1LaserClose()  digitalWrite(L1Laser,HIGH) 
#define L2LaserClose()  digitalWrite(L2Laser,HIGH)
#define L3LaserClose()  digitalWrite(L3Laser,HIGH)

#define ButtonPressed   HIGH
#define ButtonDeadTime  50


#define L1ButtonRead()  digitalRead(L1Button)
#define L2ButtonRead()  digitalRead(L2Button)
#define L3ButtonRead()  digitalRead(L3Button)

#define LaserReadLimit  700      // The analog read limit
#define L1LaserRead()   analogRead(L1Pin)
#define L2LaserRead()   analogRead(L2Pin)
#define L3LaserRead()   analogRead(L3Pin)

#define MeasureTimes    21      // n  = (MeasureTimes - 1)/2

enum Status 
{
    Standby        = 0,
    MeasureL1Cyle  = 1,
    MeasureL2Cyle  = 2,
    MeasureL3Cyle  = 3,
    Other          = 4,
};
typedef enum Status SystemStatus;
SystemStatus WorkingStatus;

I2C_LCD LCD;
uint8_t I2C_LCD_ADDRESS = 0x51; //Device address configuration, the default value is 0x51.
unsigned long StartTime = 0;

float L1Period  = 0;
float L2Period  = 0;
float L3Period  = 0;
uint8_t counter = 0;            // To count how many times 
void setup()
{
    Serial.begin(9600);
    Serial.flush();
    
    /* Init the button */
    
    pinMode(L1Button,INPUT);
    pinMode(L2Button,INPUT);
    pinMode(L3Button,INPUT);
    
    
    /* Init the Laser */
    
    pinMode(L1Laser,OUTPUT);
    pinMode(L2Laser,OUTPUT);
    pinMode(L3Laser,OUTPUT);
    L1LaserClose();
    L2LaserClose();
    L3LaserClose();
    
    /* Init the I2C_LCD */
    
    Wire.begin();
    LCD.CleanAll(WHITE);    //Clean the screen with black or white.
    delay(1000);      
    LCD.FontModeConf(Font_8x16_1, FM_ANL_AAA, BLACK_BAC); 
    LCD.CharGotoXY(36,0);
    //Print string on I2C_LCD.   
    LCD.println("Pendulum");
    LCD.FontModeConf(Font_6x12, FM_ANL_AAA, BLACK_BAC); 
    LCD.CharGotoXY(0,20);
    LCD.print("T1  =   No Data");
    LCD.CharGotoXY(0,35);
    LCD.print("T2  =   No Data");
    LCD.CharGotoXY(0,50);
    LCD.print("T3  =   No Data");
}

void loop()
{
    switch(WorkingStatus) {
    case Standby:
        switch(IfButtonPressed()) {
        case MeasureL1Cyle:
            L1LaserOpen();
            WorkingStatus = MeasureL1Cyle;
            counter = 0;
            break;
        case MeasureL2Cyle:
            L2LaserOpen();
            WorkingStatus = MeasureL2Cyle;
            counter = 0;
            break;
        case MeasureL3Cyle:
            L3LaserOpen();
            WorkingStatus = MeasureL3Cyle; 
            counter = 0;
            break;
        default:
            break;
        }
        break;
    case MeasureL1Cyle:
        //delay(5000);
        if (L1LaserRead() < LaserReadLimit) {
            delay(35);
            counter++;
         
            Serial.println(counter);
            delay(25);
        }
        if (counter == 1) {
            StartTime = millis();
        } else if (counter == MeasureTimes) {
            L1Period = (millis() - StartTime)/((MeasureTimes - 1)/2);
            Serial.println(L1Period);
            
            
            L1LaserClose();
            LCDDisplay((uint16_t) L1Period);
            WorkingStatus = Standby;
        }
        break;
    case MeasureL2Cyle:
        if (L2LaserRead() < LaserReadLimit) {
            delay(35);
            counter++;
            Serial.println(counter);
            delay(25);
        }
        if (counter == 1) {
            StartTime = millis();
        } else if (counter == MeasureTimes) {
            L2Period = (millis() - StartTime)/((MeasureTimes - 1)/2);
            Serial.println(L2Period);
            
            L2LaserClose();
            LCDDisplay((uint16_t) L2Period);
            WorkingStatus = Standby;
        }
        break;
    case MeasureL3Cyle:
        if (L3LaserRead() < LaserReadLimit) {
            delay(45);
            counter++;
         
            Serial.println(counter);
            delay(45);
        }
        if (counter == 1) {
            StartTime = millis();
        } else if (counter == MeasureTimes) {
            L3Period = (millis() - StartTime)/((MeasureTimes - 1)/2);
            Serial.println(L3Period);

            L3LaserClose();
            LCDDisplay((uint16_t) L3Period);
            WorkingStatus = Standby;
        }
        break;
    default:
        break;
    }
}

uint8_t IfButtonPressed(void) 
{
    if (L1ButtonRead() == ButtonPressed) {
        delay(ButtonDeadTime);
        if (L1ButtonRead() == ButtonPressed) {
            return MeasureL1Cyle;
        } else {
            return Standby;
        }
    } else if (L2ButtonRead() == ButtonPressed) {
        delay(ButtonDeadTime);
        if (L2ButtonRead() == ButtonPressed) {
            return MeasureL2Cyle;
        } else {
            return Standby;
        }
    } else if (L3ButtonRead() == ButtonPressed) {
        delay(ButtonDeadTime);
        if(L3ButtonRead() == ButtonPressed) {
            return MeasureL3Cyle;
        } else {
            return Standby;
        }
    } else {
        return Standby;
    }
}

void LCDDisplay(uint16_t period) 
{
    switch(WorkingStatus) {
    case MeasureL1Cyle:
        LCD.CharGotoXY(44,20);      
        break;
    case MeasureL2Cyle:
        LCD.CharGotoXY(44,35);   
        break;
    case MeasureL3Cyle:
        LCD.CharGotoXY(44,50);    
        break;
    default :
        break; 
    }
    LCD.print(" ");
    if(period<10) {
        LCD.print("00");
        LCD.print(period,DEC);
        LCD.print(" ms  ");
    } else if(period<100) {
        LCD.print("0");
        LCD.print(period,DEC);
        LCD.print(" ms  ");
    } else {
        LCD.print(period,DEC);
        LCD.print(" ms  ");
    }          
}