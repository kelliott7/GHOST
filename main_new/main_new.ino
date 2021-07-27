#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
//#include <math.h>

AccelStepper stepperX(AccelStepper::DRIVER, 2, 5);
AccelStepper stepperY(AccelStepper::DRIVER, 3, 6);
AccelStepper stepperZ(AccelStepper::DRIVER, 4, 7);

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

  stepperY.setCurrentPosition(0);
  stepperY.setMaxSpeed(200.0);
  stepperY.setAcceleration(75);
  steppers.addStepper(stepperY);

  stepperZ.setCurrentPosition(0);
  stepperZ.setMaxSpeed(200.0);
  stepperZ.setAcceleration(75);
  steppers.addStepper(stepperZ);


  pinMode(enablePin, OUTPUT); // **** set the enable pin to output
  digitalWrite(enablePin, LOW); // *** set the enable pin low
  
}

void loop() {
  if (bluetooth.available() > 0) {
    stepperX.setCurrentPosition((long)phonePitch/1.8);
    stepperY.setCurrentPosition((long)phoneYaw/1.8);
    stepperZ.setCurrentPosition((long)phoneRoll/1.8);

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
  if (abs(phonePitchRaw - phonePitch) > 90){
    phonePitchRaw = phonePitch;
  }
  if (abs(phoneYawRaw   - phoneYaw)   > 90){
    phoneYawRaw = phoneYaw;
  }
  if (abs(phoneRollRaw   - phoneRoll)   > 90){
    phoneRollRaw = phoneRoll;
  }
    
  
  float weight = 0.90;
  phonePitch = (1.0-weight) * phonePitch + weight * phonePitchRaw;
  phoneYaw = (1.0-weight) * phoneYaw + weight * phoneYawRaw;
  phoneRoll = (1.0-weight) * phoneRoll + weight * phoneRollRaw;
 
  
 
  stepperX.moveTo((long)phonePitch/1.8);
  stepperY.moveTo((long)phoneYaw/1.8);
  stepperZ.moveTo((long)phoneRoll/1.8);

  //steppers.run();

  while(stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0 || stepperZ.distanceToGo() != 0) {
      //Serial.println(stepperX.distanceToGo());
      steppers.run();
  }
  

  //delay(1000);

  }
}
