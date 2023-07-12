/*
 *  Mandatory includes
 */
#include <Arduino.h>
#include <TinyMPU6050.h>
#include "quadcopter.h"
#include <PID_v1.h>

MPU6050 mpu (Wire);
QuadcopterController quad;

double realPitch = 0;
double realRoll = 0;

double desiredPitch = 0;
double desiredRoll = 0;

double pitchOutput = 0;
double rollOutput = 0;

double hover = 0.46;
double pitchHover = 0.01;
double rollHover = 0;

// PID pitchPID(&realPitch, &pitchOutput, &desiredPitch, 0.005, 0.002, 0.001, DIRECT);
// PID rollPID(&realRoll, &rollOutput, &desiredRoll, 0.005, 0.002, 0.001, DIRECT);
PID pitchPID(&realPitch, &pitchOutput, &desiredPitch, 0.003, 0, 0.001, DIRECT);
PID rollPID(&realRoll, &rollOutput, &desiredRoll, 0.003, 0, 0.001, DIRECT);

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

  pitchPID.SetSampleTime(8);
  rollPID.SetSampleTime(8);

  pitchPID.SetOutputLimits(-100, 100);
  rollPID.SetOutputLimits(-100, 100);
}

/*
 *  Loop
 */

double x = 0, y = 0, z = 0;
double slamRoll = 0, slamPitch = 0, slamYaw = 0;
double Data[6];
 
double PITCH_OFFSET = -1.11;
double ROLL_OFFSET = 0.1;

bool STOP = true;

// double Setpoint, Input, Output;
// PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

 
void loop() {

  mpu.Execute();

  Serial.print(pitchPID.Compute());
  Serial.print(rollPID.Compute());

  realPitch = mpu.GetAngX() + PITCH_OFFSET;
  realRoll = mpu.GetAngY() + ROLL_OFFSET;
  // Yaw not accurate enough
  // Yaw not accurate enough
  
  updateBuffer();

//  for(int i = 0; i < 6; i++) {
//    Serial.print(Data[i]);
//    Serial.print(' ');  
//  }
//
//  Serial.println();
  x = Data[0];
  y = Data[1];
  z = Data[2];
  slamPitch = Data[3] / PI * 180;
  slamYaw = Data[4] / PI * 180;
  slamRoll = Data[5] / PI * 180;

  if(realPitch > 30 || realRoll > 30 || realPitch < -30 || realRoll < -30) STOP = true;

  Serial.print("Pitch: ");
  Serial.print(realPitch);
  Serial.print("\t");
  Serial.print("Roll: ");
  Serial.println(realRoll);

  if(STOP) quad.setSpeed(0, 0, 0, 0);
  else quad.setSpeed(hover, rollOutput + rollHover, pitchOutput + pitchHover, 0);
  if(STOP)
  Serial.println("Emergency stop");
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
