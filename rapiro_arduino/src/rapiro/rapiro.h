#ifndef _RAPIRO_H
#define _RAPIRO_H

#include <Arduino.h>

#define N_SERVOS 12

// power switch for servos
#define PIN_PWR A3

// pin allocation for servos in rapiro
#define PIN_HEAD      10
#define PIN_WAIST     11
#define PIN_R_SHLD_R   9
#define PIN_R_SHLD_P   8
#define PIN_R_HAND     7
#define PIN_R_FOOT_Y   4
#define PIN_R_FOOT_P   2
#define PIN_L_SHLD_R  12
#define PIN_L_SHLD_P  13
#define PIN_L_HAND    A0
#define PIN_L_FOOT_Y  A1
#define PIN_L_FOOT_P  A2

// pin allocation RGB LED
#define PIN_LED_R  6
#define PIN_LED_G  5
#define PIN_LED_B  3

// pin allocation I2C
#define PIN_I2C_SDA A4
#define PIN_I2C_SDL A5

// distance sensor pins
#define PIN_SENS_1 A6 // (no digital IO capabilities)
#define PIN_SENS_2 A7 // (no digital IO capabilities)
#define PIN_TRIG   PIN_I2C_SDA // need digital IO for HC-SR04
#define PIN_ECHO   PIN_I2C_SDL // need digital IO for HC-SR04

// servo fine tuning for correct kinematics
const char trim_angle[N_SERVOS] = {
                          -5,  // Head
                           6,  // Waist
                           0,  // R Shoulder roll
                           0,  // R Shoulder pitch
                           0,  // R Hand grip
                           0,  // L Shoulder roll
                           0,  // L Shoulder pitch
                           0,  // L Hand grip
                           2,  // R Foot yaw
                           3,  // R Foot pitch
                           1,  // L Foot yaw
                           2}; // L Foot pitch
// range for servos                 0    1    2    3    4    5    6    7    8    9   10   11
const byte min_angle[N_SERVOS] = {  0,   0,   0,  25,  55,   0,  45,  55,  50,  35,  50,  50};
const byte max_angle[N_SERVOS] = {180, 180, 180, 135, 110, 180, 160, 110, 130, 130, 130, 145};
// zero positions
const byte zero_pose[N_SERVOS] = { 90,  90,   0, 130,  90, 180,  50,  90,  90,  90,  90,  90};
                    
#endif // _RAPIRO_H

