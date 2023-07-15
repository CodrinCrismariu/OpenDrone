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

    fR.writeMicroseconds(1000 + Power - Yaw - Roll + Pitch);
    fL.writeMicroseconds(1000 + Power + Yaw - Roll - Pitch);
    bL.writeMicroseconds(1000 + Power - Yaw + Roll - Pitch);
    bR.writeMicroseconds(1000 + Power + Yaw + Roll + Pitch);

  }

};