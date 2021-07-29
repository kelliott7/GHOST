#include <SoftwareSerial.h>
#include <FlexyStepper.h>

/*
   main_new.ino
   //// CONTRIBUTORS ////
   Katie Elliott
   katherine.elliott@hilltechnicalsolutions.com
   Evan Baker
   evan.baker@hilltechnicalsolutions.com
   Operates as the main brain of the table top TAMS
   project, "GHOST".  Reads orientation data from
   android app via bluetooth and performs matrix
   math to determine the commands to be sent to the
   stepper motors.
   Developed in the Arduino IDE
*/




FlexyStepper stepperX;
FlexyStepper stepperY;
FlexyStepper stepperZ;

const int STEPPER_SPEED = 150;
const int STEPPER_ACCEL = 100;

SoftwareSerial bluetooth(A0, A1); // RXD > Hold, TXD > Abort, GND > GND, VCC > 5V

const byte enablePin = 8;  // ***** pin 8 is the enable pin
boolean runLoop;
float phoneYawRaw, phonePitchRaw, phoneRollRaw;
float phoneYaw = 0, phonePitch = 0, phoneRoll = 0;
float DCM [3][3], EulerPYR [3], rotPYR [3], stepPYR [3];
float oldPitch = 0, oldYaw = 0, oldRoll = 0;

void setup() {
  Serial.begin(9600); //Adjust Serial monitor to this when testing
  bluetooth.begin(9600);

  stepperX.connectToPins(2, 5);
  stepperY.connectToPins(3, 6);
  stepperZ.connectToPins(4, 7);

  pinMode(enablePin, OUTPUT); // **** set the enable pin to output
  digitalWrite(enablePin, LOW); // *** set the enable pin low

}

void loop() {

  stepperX.setSpeedInStepsPerSecond(STEPPER_SPEED);
  stepperX.setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCEL);

  stepperY.setSpeedInStepsPerSecond(STEPPER_SPEED);
  stepperY.setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCEL);

  stepperZ.setSpeedInStepsPerSecond(STEPPER_SPEED);
  stepperZ.setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCEL);

  if (bluetooth.available() > 0) {
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

    

    if ((abs(oldPitch - phonePitch) < 15) &&  (abs(oldYaw - phoneYaw) < 15) && (abs(oldRoll - phoneRoll) < 15)) {

      EulerYPR_to_DCM(phoneYaw, phonePitch, phoneRoll, DCM);
      DCM_to_EulerPYR(DCM, EulerPYR);
    
      stepperX.setTargetPositionInSteps((long)EulerPYR[0] / 1.8);
      stepperY.setTargetPositionInSteps((long)EulerPYR[1] / 1.8);
      stepperZ.setTargetPositionInSteps((long)EulerPYR[2] / 1.8);

      while ((!stepperX.motionComplete()) || (!stepperY.motionComplete()) || (!stepperY.motionComplete()))
      {
        stepperX.processMovement();
        stepperY.processMovement();
        stepperZ.processMovement();
      }
    }
    oldPitch = phonePitch;
    oldYaw = phoneYaw;
    oldRoll = phoneRoll;

    Serial.println(stepperX.getCurrentPositionInSteps());  
    Serial.println(stepperY.getCurrentPositionInSteps());  
    Serial.println(stepperZ.getCurrentPositionInSteps());  


  }
}

void EulerYPR_to_DCM(float phoneYaw, float phonePitch, float phoneRoll, float DCM [3][3]) {
  float phoneRoll_rad = phoneRoll * PI / 180;
  float phonePitch_rad = phonePitch * PI / 180;
  float phoneYaw_rad = phoneYaw * PI / 180;
  DCM[1][1] = cos(phonePitch_rad) * cos(phoneYaw_rad);
  DCM[2][1] = cos(phonePitch_rad) * sin(phoneYaw_rad);
  DCM[3][1] = -sin(phonePitch_rad);
  DCM[1][2] = -cos(phoneRoll_rad) * sin(phoneYaw_rad) + sin(phoneRoll_rad) * sin(phonePitch_rad) * cos(phoneYaw_rad);
  DCM[2][2] = cos(phoneRoll_rad) * cos(phoneYaw_rad) + sin(phoneRoll_rad) * sin(phonePitch_rad) * sin(phoneYaw_rad);
  DCM[3][2] = sin(phoneRoll_rad) * cos(phonePitch_rad);
  DCM[1][3] = sin(phoneRoll_rad) * sin(phoneYaw_rad) + cos(phoneRoll_rad) * sin(phonePitch_rad) * cos(phoneYaw_rad);
  DCM[2][3] = -sin(phoneRoll_rad) * cos(phoneYaw_rad) + cos(phoneRoll_rad) * sin(phonePitch_rad) * sin(phoneYaw_rad);
  DCM[3][3] = cos(phoneRoll_rad) * cos(phonePitch_rad);
}

void DCM_to_EulerPYR(float DCM[3][3], float EulerPYR [3]) {
  float Pitch, Yaw, Roll;
  if (DCM[2][1] < -0.99987) {
    Yaw = -90;
    Roll = 0;
    Pitch = (atan2(DCM[1][3], DCM[3][3])) * 180 / PI;
  }
  else if (DCM[2][1] > 0.99987) {
    Yaw = 90;
    Roll = 0;
    Pitch = (atan2(DCM[1][3], DCM[3][3])) * 180 / PI;
  }
  else {
    Pitch = (atan2(-DCM[3][1], DCM[1][1])) * 180 / PI;
    Yaw = (asin(DCM[2][1])) * 180 / PI;
    Roll = (atan2(-DCM[2][3], DCM[2][2])) * 180 / PI;
  }
  EulerPYR[0] = Pitch;
  EulerPYR[1] = Yaw;
  EulerPYR[2] = Roll;
}
