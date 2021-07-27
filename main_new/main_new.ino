#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

AccelStepper stepperX(AccelStepper::DRIVER, 2, 5);
MultiStepper steppers;
SoftwareSerial bluetooth(A0, A1); // RXD > Hold, TXD > Abort, GND > GND, VCC > 5V

const byte enablePin = 8;  // ***** pin 8 is the enable pin
boolean runLoop;
float phoneYawRaw, phonePitchRaw, phoneRollRaw;
float phoneYaw = 0, phonePitch = 0, phoneRoll = 0;

void setup() {
  Serial.begin(9600); //Adjust Serial monitor to this when testing
  bluetooth.begin(9600); 
  stepperX.setCurrentPosition(0);
  stepperX.setMaxSpeed(200.0);
  stepperX.setAcceleration(75);
  steppers.addStepper(stepperX);
  pinMode(enablePin, OUTPUT); // **** set the enable pin to output
  digitalWrite(enablePin, LOW); // *** set the enable pin low
  
}

void loop() {
  if (bluetooth.available() > 0) {
    stepperX.setCurrentPosition((long)phonePitchRaw/1.8);
    runLoop = 1;
  while (runLoop == 1) {
    char input = bluetooth.read();
    switch (input)
    {
      case 'P': phonePitchRaw = bluetooth.parseFloat(); //break;
      case 'Y': phoneYawRaw = bluetooth.parseFloat(); //break;
        if (phoneYawRaw > 180) {
          phoneYawRaw = phoneYawRaw - 360;
        }
      case 'R': phoneRollRaw = bluetooth.parseFloat();
        Serial.print("PYR:\t");
        Serial.print(phonePitchRaw);
        Serial.print('\t');
        Serial.print(phoneYawRaw);
        Serial.print('\t');
        Serial.println(phoneRollRaw);
        runLoop = 0;
        break;

      default: break;
    }
  }

  
 
  stepperX.moveTo((long)phonePitchRaw/1.8);

  while(stepperX.distanceToGo() != 0) {
      Serial.println(stepperX.distanceToGo());
      stepperX.run();
  }

  //delay(1000);
  
  }
}
