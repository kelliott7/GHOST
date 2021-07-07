float phoneYaw, phonePitch, phoneRoll;
float currentPYR [3], DCM [3][3], EulerPYR [3], rotPYR [3], stepPYR [3];

void setup(){

}

void loop(){
  EulerYPR_to_DCM(phoneYaw, phonePitch, phoneRoll, DCM);
  DCM_to_EulerPYR(DCM, EulerPYR);
  for (int i = 0; i < 3; i++) {
    rotPYR[i] = EulerPYR[i] - currentPYR[i];
    stepPYR[i] = rotPYR[i] / 1.8;
    currentPYR[i] = stepPYR[i] * 1.8;
  }
}

void EulerYPR_to_DCM(float phoneYaw, float phonePitch, float phoneRoll, float DCM [3][3]) {
  float phoneRoll_rad = phoneRoll*PI/180;
  float phonePitch_rad = phonePitch*PI/180;
  float phoneYaw_rad = phoneYaw*PI/180;
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
    Pitch = (atan2(DCM[1][3], DCM[3][3]))*180/PI;
  }
  else if (DCM[2][1] > 0.99987) {
    Yaw = 90;
    Roll = 0;
    Pitch = (atan2(DCM[1][3], DCM[3][3]))*180/PI;
  }
  else {
    Pitch = (atan2(-DCM[3][1],DCM[1][1]))*180/PI;
    Yaw = (asin(DCM[2][1]))*180/PI;
    Roll = (atan2(-DCM[2][3],DCM[2][2]))*180/PI;
  }
  EulerPYR[0] = Pitch;
  EulerPYR[1] = Yaw;
  EulerPYR[2] = Roll;
}
