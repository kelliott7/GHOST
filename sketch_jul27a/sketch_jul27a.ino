#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

//MOTOR SETUP
AccelStepper stepperX(AccelStepper::DRIVER, 2, 5);
AccelStepper stepperY(AccelStepper::DRIVER, 3, 6);
AccelStepper stepperZ(AccelStepper::DRIVER, 4, 7);
MultiStepper steppers;
const byte enablePin = 8;  // ***** pin 8 is the enable pin

//BLUETOOTH SERIAL SETUP
SoftwareSerial bluetooth(A0, A1); // RXD > Hold, TXD > Abort, GND > GND, VCC > 5V

//VARIABLE INITIALIZATION
float phoneYawRaw, phonePitchRaw, phoneRollRaw;
float phoneYaw = 0, phonePitch = 0, phoneRoll = 0;
float xsteps = 0, ysteps = 0, zsteps = 0;
float DCM [3][3], EulerPYR [3], rotPYR [3], stepPYR [3];
float currentPYR [3] = {0, 0, 0};
boolean runLoop;

void setup() {
  //BLUETOOTH SETUP 2
  Serial.begin(9600); //Adjust Serial monitor to this when testing
  bluetooth.begin(9600); 
  
  //MOTOR SETUP 2
  stepperX.setCurrentPosition(0);
  stepperX.setMaxSpeed(200.0);
  stepperX.setAcceleration(75);

  stepperY.setCurrentPosition(0);
  stepperY.setMaxSpeed(200.0);
  stepperY.setAcceleration(75);
  
  stepperZ.setCurrentPosition(0);
  stepperZ.setMaxSpeed(200.0);
  stepperZ.setAcceleration(75);

  steppers.addStepper(stepperX);
  steppers.addStepper(stepperY);
  steppers.addStepper(stepperZ);


  pinMode(enablePin, OUTPUT); // **** set the enable pin to output
  digitalWrite(enablePin, LOW); // *** set the enable pin low

}
void loop() {
  if (bluetooth.available() > 0) {
    char nextChar = ' ';
    String bluetoothString = "";
    while (nextChar != '/n') {
      nextChar = bluetooth.read();
      bluetoothString += nextChar;
    }
    Serial.println(bluetoothString);
  }
}
