#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <MultiStepper.h>



/*
   main_controller.ino
   //// CONTRIBUTORS ////
   Katie Elliott
   katherine.elliott@hilltechnicalsolutions.com
   Evan Baker
   evan.baker@hilltechnicalsolutions.com

   Operates as the main brain of the table top TAMS
   project, "GHOST".  Reads orientational data from
   android app via bluetooth and performs matrix
   math to determine the commands to be send to the
   stepper motors.

   Developed in the Arduino IDE
*/


//MOTOR SETUP
AccelStepper stepperX(AccelStepper::DRIVER, 2, 5);
AccelStepper stepperY(AccelStepper::DRIVER, 3, 6);
AccelStepper stepperZ(AccelStepper::DRIVER, 4, 7);
MultiStepper steppers;
const byte enablePin = 8;  // ***** pin 8 is the enable pin

//BLUETOOTH SERIAL SETUP
SoftwareSerial bluetooth(A0, A1); // RXD > Hold, TXD > Abort, GND > GND, VCC > 5V
  float phoneYaw, phonePitch, phoneRoll;
  float currentPYR [3] = {0, 0, 0};
  float DCM [3][3], EulerPYR [3], rotPYR [3], stepPYR [3];

void setup() {

  //BLUETOOTH SETUP 2
  Serial.begin(9600); //Adjust Serial monitor to this when testing
  bluetooth.begin(9600); 
  
  //MOTOR SETUP 2
  stepperX.setMaxSpeed(100.0);
  stepperX.setAcceleration(50);

  stepperY.setMaxSpeed(100.0);
  stepperY.setAcceleration(50);

  stepperZ.setMaxSpeed(100.0);
  stepperZ.setAcceleration(50);

  steppers.addStepper(stepperX);
  steppers.addStepper(stepperY);
  steppers.addStepper(stepperZ);


  pinMode(enablePin, OUTPUT); // **** set the enable pin to output
  digitalWrite(enablePin, LOW); // *** set the enable pin low

}

void loop() {
  if (bluetooth.available() > 0) {
    bluePhone();
    EulerYPR_to_DCM(phoneYaw, phonePitch, phoneRoll, DCM);
    DCM_to_EulerPYR(DCM, EulerPYR);

    for (int i = 0; i < 3; i++) {
     rotPYR[i] = EulerPYR[i] - currentPYR[i];
     Serial.println(stepPYR[i]);
     stepPYR[i] = rotPYR[i]/1.8;
     currentPYR[i] = EulerPYR[i];
    }
    steppers.moveTo((long)stepPYR);
    steppers.runSpeedToPosition();
    
  }
}

/*
 * Formats data from phone to be ready to print
 */
void bluePhone() {
  phonePitch = bluetooth.parseFloat();
  phoneYaw = bluetooth.parseFloat();
  phoneRoll = bluetooth.parseFloat();
  /*Serial.print("Pitch: ");
  Serial.print(phonePitch);
  Serial.print('\t');
  Serial.print("Yaw: ");
  Serial.print(phoneYaw);
  Serial.print('\t');
  Serial.print("Roll: ");
  Serial.println(phoneRoll);
  */
}


void EulerYPR_to_DCM(  float phoneYaw,   float phonePitch,   float phoneRoll,   float DCM [3][3]) {
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
