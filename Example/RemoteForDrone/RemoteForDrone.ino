#include "Quadcopter.h"

/////////////////////////////////////////////////////////////////
// Declaring Variables
/////////////////////////////////////////////////////////////////
// WiFi
struct MSG {
  int8_t  HeightRef, YawRef, PitchRef, RollRef;
} msg;
RF24 radio(7, 8); // The NRF24L01 Pin CE and Pin CSN
const uint64_t pipe = 0xE8E8F0F0E1LL; //Communication Pip Address

// Remote Controller
int TurnOnOffPin = 3;
int JX1 = A0;
int JY1 = A1;
int JX2 = A2;
int JY2 = A3;
MovingAvgInt rX1;
MovingAvgInt rY1;
MovingAvgInt rX2;
MovingAvgInt rY2;
int MinHeight = 0;
int MaxHeight = 2;
int MinRoll = -10;
int MaxRoll = 10;
int MinPitch = -10;
int MaxPitch = 10;
int MinYaw = -30;
int MaxYaw = 30;
int HeightSP, RollSP, PitchSP, YawSP;
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
// Setup routine
/////////////////////////////////////////////////////////////////
void setup() {
  //  Serial.begin(9600);
  // WiFi
  radio.begin();
  radio.openWritingPipe(pipe); //Open Communication Pipe
  radio.stopListening();
  pinMode(TurnOnOffPin, OUTPUT);
}
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
// Loop routine
/////////////////////////////////////////////////////////////////
void loop() {
  // read values from joystick
  rX1.add(analogRead(JX1));
  rY1.add(analogRead(JY1));
  rX2.add(analogRead(JX2));
  rY2.add(analogRead(JY2));

  if (rX1.read() >= 510) {
    HeightSP = map(rX1.read(), 510, 1024, MinHeight, MaxHeight);
  }
  else HeightSP = MinHeight;
  msg.HeightRef = HeightSP;

  if (rY2.read() >= 520 || rY2.read() <= 490) {
    RollSP = map(rY2.read(), 0, 1024, MinRoll, MaxRoll);
  }
  else RollSP = 0;
  msg.RollRef = RollSP;

  if (rX2.read() >= 520) {
    PitchSP = map(rX2.read(), 520, 1024, 0, MaxPitch);
  }
  else if (rX2.read() < 490) {
    PitchSP = map(rX2.read(), 0, 490 , MinPitch, 0);
  }
  else PitchSP = 0;
  msg.PitchRef = PitchSP;

  if (rY1.read() >= 600) {
    YawSP = map(rY1.read(), 600, 1024, 0, MaxYaw);
  }
  else if ( rY1.read() < 400) {
    YawSP = map(rY1.read(), 0, 400, MinYaw, 0);
  }
  else YawSP = 0;
  msg.YawRef = YawSP;

  if (digitalRead(TurnOnOffPin)) {
    msg.HeightRef = 0;
  }
  //  Serial.print(msg.Throttle);
  //  Serial.print(" ");
  //  Serial.print(msg.Roll);
  //  Serial.print(" ");
  //  Serial.print(msg.Pitch);
  //  Serial.print(" ");
  //  Serial.print(msg.Yaw);
  //  Serial.println(" ");
  radio.write(&msg, sizeof(msg));
}
/////////////////////////////////////////////////////////////////
