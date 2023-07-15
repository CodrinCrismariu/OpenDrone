/*
 *  Mandatory includes
 */
#include <Arduino.h>
#include <TinyMPU6050.h>
#include "quadcopter.h"
#include <PID_v1.h>

MPU6050 mpu (Wire);
QuadcopterController quad;

double predictedPitch = 0;
double predictedRoll = 0;

double realPitch = 0;
double realRoll = 0;
double realYaw = 0;

double desiredPitch = 0;
double desiredRoll = 0;
double desiredYaw = 0;

double pitchOutput = 0;
double rollOutput = 0;
double yawOutput = 0;

double x = 0, y = 0, z = 0;
double slamRoll = 0, slamPitch = 0, slamYaw = 0;

double hover = 520;
double hoverOutput = 0;
double desiredY = 0;
double desiredX = 0;
double desiredZ = 0;

PID pitchPID(&predictedPitch, &pitchOutput, &desiredPitch, 1.5, 2, 0, DIRECT);
PID rollPID(&predictedRoll, &rollOutput, &desiredRoll, 1.5, 2, 0, DIRECT);
PID yawPID(&realYaw, &yawOutput, &desiredYaw, 0.5, 0, 0, DIRECT);

PID hoverPID(&desiredY, &hoverOutput, &y, 20, 0, 0, DIRECT);
PID positionPitchPID(&x, &desiredPitch, &desiredX, 2, 0, 0, DIRECT);
PID positionRollPID(&z, &desiredRoll, &desiredZ, 2, 0, 0, DIRECT);

unsigned long lastTime = 0;

void setup() {

  // Initialization
  mpu.Initialize();
  quad.init();

  // Calibration
  Serial.begin(115200);
  Serial.println("=====================================");
  Serial.println("Starting calibration...");
  mpu.Calibrate();
  Serial.println("Calibration complete!");
  Serial.println("Offsets:");
  Serial.print("GyroX Offset = ");
  Serial.println(mpu.GetGyroXOffset());
  Serial.print("GyroY Offset = ");
  Serial.println(mpu.GetGyroYOffset());
  Serial.print("GyroZ Offset = ");
  Serial.println(mpu.GetGyroZOffset());

  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);
  hoverPID.SetMode(AUTOMATIC);
  positionPitchPID.SetMode(AUTOMATIC);
  positionRollPID.SetMode(AUTOMATIC);
  

  pitchPID.SetSampleTime(4);
  rollPID.SetSampleTime(4);
  yawPID.SetSampleTime(4);
  hoverPID.SetSampleTime(4);
  positionPitchPID.SetSampleTime(4);
  positionRollPID.SetSampleTime(4);

  for(int i = 0; i < 10; i++) mpu.Execute();

  desiredYaw = mpu.GetAngZ();

  pitchPID.SetOutputLimits(-400, 400);
  rollPID.SetOutputLimits(-400, 400);
  yawPID.SetOutputLimits(-400, 400);
  hoverPID.SetOutputLimits(-400, 400);
  positionPitchPID.SetOutputLimits(-10, 10);
  positionRollPID.SetOutputLimits(-10, 10);

  lastTime = micros();
}

/*
 *  Loop
 */
double Data[6];
bool STARTED_SLAM = false;
 
double PITCH_OFFSET = -0.96;
double ROLL_OFFSET = 0.12 + 1;

bool STOP = false;

// double Setpoint, Input, Output;
// PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

double deceleration = 180; // deg per second

double filter = 0.9;
double lastPredictedPitch = 0;
double lastPredictedRoll = 0;
 
void loop() {

  updateBuffer();

  x = Data[0];
  y = Data[1];
  z = Data[2];
  slamPitch = Data[3] / PI * 180;
  slamYaw = Data[4] / PI * 180;
  slamRoll = Data[5] / PI * 180;

  mpu.Execute();

  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();
  hoverPID.Compute();
  positionPitchPID.Compute();
  positionRollPID.Compute();

  realPitch = mpu.GetAngX() + PITCH_OFFSET;
  realRoll = mpu.GetAngY() + ROLL_OFFSET;
  realYaw = mpu.GetAngZ();

  if(STARTED_SLAM) realYaw = slamYaw;

  double angVelPitch = mpu.GetGyroX();
  double angVelRoll = mpu.GetGyroY();

  double pitchTime = abs(angVelPitch / deceleration);
  double rollTime = abs(angVelRoll / deceleration);

  predictedPitch = (pitchTime * angVelPitch / 2 + realPitch) * filter + (1 - filter) * lastPredictedPitch;
  predictedRoll = (rollTime * angVelRoll / 2 + realRoll) * filter + (1 - filter) * lastPredictedRoll;

  for(int i = 0; i < 6; i++) {
    Serial.print(Data[i]);
    Serial.print(' ');  
  }

  Serial.print(STARTED_SLAM);

  Serial.println();

  if(realPitch > 30 || realRoll > 30 || realPitch < -30 || realRoll < -30) STOP = true;

  if(STOP) quad.setSpeed(0, 0, 0, 0);
  else if(STARTED_SLAM) quad.setSpeed(hover + hoverOutput, rollOutput, pitchOutput, yawOutput);

  lastPredictedRoll = predictedRoll;
  lastPredictedPitch = predictedPitch;

  while(micros() - lastTime < 3900);

  // Serial.print("loop time(us): ");
  // Serial.println(micros() - lastTime);
  lastTime = micros();
  
}

unsigned short bufferIndex = 0;
char serialBuffer[256];
unsigned short dataIndex = 0;

void processBuffer() {
   char* token = strtok(serialBuffer, " ");
   while(token != NULL && dataIndex < 6) {
     Data[dataIndex++] = atof(token);
     token = strtok(NULL, " "); 
   }

   bufferIndex = 0;
   dataIndex = 0;

   STARTED_SLAM = true;
}

void updateBuffer() {
  while(Serial.available() > 0) {
    serialBuffer[bufferIndex++] = Serial.read();

    if(serialBuffer[bufferIndex - 1] == '\n') {
      serialBuffer[bufferIndex - 1] = 0;
      processBuffer();
      break;  
    }
  }
}
