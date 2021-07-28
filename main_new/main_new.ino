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
    //stepperX.setCurrentPosition((long)phonePitch / 1.8);
    //stepperY.setCurrentPosition((long)phoneYaw / 1.8);
    //stepperZ.setCurrentPosition((long)phoneRoll / 1.8);

    char nextChar = ' ';
    String bluetoothString = "";
    while (nextChar != '\n') {
      if (bluetooth.available()) {
        nextChar = bluetooth.read();
        bluetoothString.concat((String)nextChar);
      }
    }
    //Serial.print(bluetoothString);
    int indP = bluetoothString.indexOf('P');
    int indY = bluetoothString.indexOf('Y');
    int indR = bluetoothString.indexOf('R');
    String phonePitchStr = bluetoothString.substring((indP + 1), (indY - 1));
    String phoneYawStr = bluetoothString.substring((indY + 1), (indR - 1));
    String phoneRollStr = bluetoothString.substring((indR + 1), bluetoothString.length() - 1);
    phonePitch = phonePitchStr.toFloat();
    phoneYaw = phoneYawStr.toFloat();
    phoneRoll = phoneRollStr.toFloat();
    Serial.print('P');
    Serial.print(phonePitch);
    Serial.print(" Y");
    Serial.print(phoneYaw);
    Serial.print(" R");
    Serial.println(phoneRoll);



    /*
      if (abs(phonePitchRaw - phonePitch) > 90){
      phonePitchRaw = phonePitch;
      }
      if (abs(phoneYawRaw   - phoneYaw)   > 90){
      phoneYawRaw = phoneYaw;
      }
      if (abs(phoneRollRaw   - phoneRoll)   > 90){
      phoneRollRaw = phoneRoll;
      }
    */
    /*
      float weight = 0.90;
      phonePitch = (1.0-weight) * phonePitch + weight * phonePitchRaw;
      phoneYaw = (1.0-weight) * phoneYaw + weight * phoneYawRaw;
      phoneRoll = (1.0-weight) * phoneRoll + weight * phoneRollRaw;
    */


    stepperX.moveTo((long)phonePitch / 1.8);
    stepperY.moveTo((long)phoneYaw / 1.8);
    stepperZ.moveTo((long)phoneRoll / 1.8);
    Serial.println(stepperX.distanceToGo());
    Serial.println(stepperY.distanceToGo());
    Serial.println(stepperZ.distanceToGo());


    steppers.runSpeedToPosition();

    //while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0 || stepperZ.distanceToGo() != 0) {
      //Serial.println(stepperX.distanceToGo());
      //steppers.run();
    //}
  }

  //delay(1000);


}

