#include "Quadcopter.h"

/////////////////////////////////////////////////////////////////
// PID Controller
/////////////////////////////////////////////////////////////////
// Sample Time
int Ts = 250; // millis

// yaw
PID yawPID(0, 0, 0, 0);
// pitch
PID pitchPID(0.6, 0.2, 5, Ts / 1000);
// roll
PID rollPID(0.6, 0.2, 5, Ts / 1000);
// height
PID heightPID(0, 0, 0, Ts / 1000);
float Min_Signal = 700;
float Max_Signal = 1350;
// error
float yawError, pitchError, rollError, heightError;
float yawSP, pitchSP, rollSP, heightSP;
float outputHeight, outputPitch, outputRoll, outputYaw;
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
// Declaring Variables
/////////////////////////////////////////////////////////////////
// WiFi
struct MSG {
  int8_t  HeightRef, YawRef, PitchRef, RollRef;
} msg;
int MinSetHeight = 0;
int MaxSetHeight = 2;
int MinSetRoll = -10;
int MaxSetRoll = 10;
int MinSetPitch = -10;
int MaxSetPitch = 10;
int MinSetYaw = -30;
int MaxSetYaw = 30;
RF24 radio(7, 8);
const uint64_t rxAddr = 0xE8E8F0F0E1LL;

// Gyroscope
Gyroscope gyro(42);
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
float yawCurrent, pitchCurrent, rollCurrent;

// ESC control
#define MOTOR_PINur 9
#define MOTOR_PINul 10 // dod
#define MOTOR_PINdr 11 // dod 
#define MOTOR_PINdl 12 // dod
Servo motor_ur;
Servo motor_ul;
Servo motor_dr;
Servo motor_dl;
float servoSignal = Min_Signal;
float esc_ur, esc_ul, esc_dr, esc_dl;

// BMS
int batPin = 8;
int AlarmBuzzer = 42;
float battery_voltage;

// Time
unsigned long oldTime = 0, timepassed;

// Barometer
int BMP_CS = 10;
float currentPressure = 1020;
BMP280 barometer(BMP_CS);
float startHeight;
float heightCurrent;
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
//Setup routine
/////////////////////////////////////////////////////////////////
void setup() {
  // Serial.begin(9600);

  // PID set limits
  yawPID.setLimits(0, 0);
  pitchPID.setLimits(50, -50);
  rollPID.setLimits(50, -50);
  heightPID.setLimits(Max_Signal, Min_Signal);
  // Gyro
  attachInterrupt(2, dmpDataReady, RISING);
  gyro.bootUp();

  // Radio
  radio.begin();
  radio.openReadingPipe(1, rxAddr);
  radio.startListening();

  // Motors
  motor_ur.attach(MOTOR_PINur);
  motor_ul.attach(MOTOR_PINul);
  motor_dr.attach(MOTOR_PINdr);
  motor_dl.attach(MOTOR_PINdl);
  motor_ur.writeMicroseconds(Min_Signal);
  motor_ul.writeMicroseconds(Min_Signal);
  motor_dr.writeMicroseconds(Min_Signal);
  motor_dl.writeMicroseconds(Min_Signal);

  // Alarm
  pinMode(AlarmBuzzer, OUTPUT);

  // Barometer
  startHeight = barometer.readAltitude(currentPressure);
}
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
//Main program loop
/////////////////////////////////////////////////////////////////
void loop() {
  // Calculate the time passed from the start of the loop
  timepassed = millis() - oldTime;

  /* The BMS is measuring the remaining battery voltage.
     If the battery voltage is lower than treshold then BMS will sound the alarm */
  if (analogRead(batPin) < 190) digitalWrite(42, HIGH);

  // Read current data from Gyroscope
  if (mpuInterrupt) GyroRead(&gyro, mpuInterrupt, yawCurrent, pitchCurrent, rollCurrent);

  if (timepassed > Ts) {
    // if the Radio is available read the setPoints from remote controller
    if (radio.available()) {
      radio.read(&msg, sizeof(msg));
      msg.HeightRef = constrain(msg.HeightRef, MinSetHeight, MaxSetHeight);
      msg.RollRef = constrain(msg.RollRef, MinSetRoll, MaxSetRoll);
      msg.PitchRef = constrain(msg.PitchRef, MinSetPitch, MaxSetPitch);
      msg.YawRef = constrain(msg.YawRef, MinSetYaw, MaxSetYaw);
      heightSP = msg.HeightRef;
      rollSP = msg.RollRef ;
      pitchSP = msg.PitchRef;
      yawSP = msg.YawRef;
    }

    // Barometer
    heightCurrent = barometer.readAltitude(currentPressure) - startHeight;

    // calculate errors
    yawError = yawSP - yawCurrent;
    pitchError = pitchSP - pitchCurrent;
    rollError = rollSP - rollCurrent;
    heightError = heightSP - heightCurrent;

    // calculate pid output
    outputHeight = heightPID.calculate(heightError);
    outputPitch = pitchPID.calculate(pitchError);
    outputRoll = rollPID.calculate(rollError);
    outputYaw = yawPID.calculate(yawError);

    // calculate servo signals for ESC's
    servoSignal = outputHeight;
    esc_ur = servoSignal - outputPitch - outputRoll - outputYaw ;
    esc_ul = servoSignal - outputPitch + outputRoll + outputYaw ;
    esc_dr = servoSignal + outputPitch - outputRoll + outputYaw ;
    esc_dl = servoSignal + outputPitch + outputRoll - outputYaw ;
    esc_ur = constrain(esc_ur, Min_Signal, Max_Signal);
    esc_ul = constrain(esc_ul, Min_Signal, Max_Signal);
    esc_dr = constrain(esc_dr, Min_Signal, Max_Signal);
    esc_dl = constrain(esc_dl, Min_Signal, Max_Signal);

    // write to motors
    motor_ur.writeMicroseconds(esc_ur);
    motor_ul.writeMicroseconds(esc_ul);
    motor_dr.writeMicroseconds(esc_dr);
    motor_dl.writeMicroseconds(esc_dl);

    oldTime = millis();
  }
}
/////////////////////////////////////////////////////////////////
