#include <TinyMPU6050.h>
#include "quadcopter.h"

MPU6050 mpu (Wire);
QuadcopterController quad;
bool STOP = true;

double hover = 0;
double hover_speed = 530; // 530

void setup() {
  mpu.Initialize();
  quad.init();

  Serial.begin(115200);
  Serial.println("=====================================");
  Serial.println("Starting calibration...");
  mpu.Calibrate();
  Serial.println("Calibration complete!");

  quad.setPos(0, 0, 0);
}

double Data[6];
bool STARTED_SLAM = false;
double slamYaw = 0, slamRoll = 0, slamPitch = 0;
double x = 0, y = 0, z = 0;

PID xpid = PID(10, 0, 2, 10, 0.004);
PID ypid = PID(0, 0, 0, 10, 0.004);
PID zpid = PID(10, 0, 2, 10, 0.004);

void loop() {
  updateBuffer();
  mpu.Execute();

  double pitch = mpu.GetAngX() - 5.5;
  double roll = mpu.GetAngY(); 

  double angVelPitch = mpu.GetGyroX();
  double angVelRoll = mpu.GetGyroY();

  x = Data[0];
  y = Data[1];
  z = Data[2];
  slamPitch = Data[3] / PI * 180;
  slamYaw = Data[4] / PI * 180;
  slamRoll = Data[5] / PI * 180;

  if(pitch > 30 || roll > 30 || pitch < -30 || roll < -30) STOP = true;
  // print("z", z);
  // print("Yaw", slamYaw);

  print("Pitch", pitch);
  print("Roll", roll);

  hover = min(hover + 100.0 / 250, hover_speed); // increment of 100 per second

  quad.setPointRoll = xpid.calculate(x, 0);
  quad.setPointPitch = zpid.calculate(z, 0);

  if(!STOP) quad.update(hover, pitch, roll, slamYaw * STARTED_SLAM, angVelPitch, angVelRoll);
  else quad.setSpeed(0, 0, 0, 0);

  Serial.println();
  regulateLoopTime(250); // Keep code running for 250Hz for stability
}


unsigned long lastTime = 0;
void regulateLoopTime(int hz) {
  while(micros() - lastTime < 1000000/hz);
  lastTime = micros();
}

void print(String name, double var) {
  Serial.print(name + ": ");
  Serial.print(var);
  Serial.print(" ");
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
