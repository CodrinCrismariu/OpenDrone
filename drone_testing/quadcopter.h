#include <Servo.h>

class QuadcopterController {

  Servo fR, fL, bR, bL;

public:

  void init() {

    fR.attach(5);
    fL.attach(6);
    bL.attach(9);
    bR.attach(10);
    
    fR.writeMicroseconds(1000);
    fL.writeMicroseconds(1000);
    bL.writeMicroseconds(1000);
    bR.writeMicroseconds(1000);

  }

  void setSpeed(double Power, double Roll, double Pitch, double Yaw) {

    Power *= 1000;
    Roll *= 1000;
    Pitch *= 1000;
    Yaw *= 1000;

    fR.writeMicroseconds(1000 + Power - Yaw - Roll + Pitch);
    fL.writeMicroseconds(1000 + Power + Yaw - Roll - Pitch);
    bL.writeMicroseconds(1000 + Power - Yaw + Roll - Pitch);
    bR.writeMicroseconds(1000 + Power + Yaw + Roll + Pitch);

  }

};