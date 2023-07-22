#include <Servo.h>
#include "PID.h"

class QuadcopterController {

public:

  Servo fR, fL, bR, bL;
  double setPointPitch = 0, setPointRoll = 0, setPointYaw = 0;

  // PID pitchPI = PID(-1.5, -0.006, 0, 400); // we only use the PI controller because the d is susceptible to noise
  // PID rollPI = PID(-1.5, -0.006, 0, 400); // we only use the PI controller because the d is susceptible to noise
  PID pitchPI = PID(1.5, 0.006, 0, 400); // we only use the PI controller because the d is susceptible to noise
  PID rollPI = PID(1.5, 0.006, 0, 400); // we only use the PI controller because the d is susceptible to noise

  PID pitchVelP = PID(-0.7, 0, 0, 400); // this is used as a D controller of the main controller for reduced noise
  PID rollVelP = PID(-0.7, 0, 0, 400); // this is used as a D controller of the main controller for reduced noise

  PID yawPID = PID(2, 0, 0.7, 400);

  void init() {

    fR.attach(9);
    fL.attach(6);
    bL.attach(5);
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

  void setPos(double desiredPitch, double desiredRoll, double desiredYaw) {

    setPointPitch = desiredPitch;
    setPointRoll = desiredRoll;
    setPointYaw = desiredYaw;

  }

  void update(double hover, double pitch, double roll, double yaw, double pitchVelocity, double rollVelocity) {
    setSpeed(hover, 
             rollPI.calculate(roll, setPointRoll) + rollVelP.calculate(0, rollVelocity),
             pitchPI.calculate(pitch, setPointPitch) + pitchVelP.calculate(0, pitchVelocity),
             yawPID.calculate(yaw, setPointYaw));
  }

};