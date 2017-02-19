#pragma once
#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

// ------------------------MUST-HAVE LIBS---------------------------
// I2C communication
#include "Wire.h"
#include "I2Cdev.h"
// SPI communication
#include <SPI.h>
// Gyroscope
#include "Gyroscope.h"
// Barometer
#include "BMP280.h"
// Servo library for ESC control
#include <Servo.h>
// Wifi
#include "nRF24L01.h"
#include "RF24.h"
// PID
#include "PID.h"
// -----------------------------------------------------------------


#endif