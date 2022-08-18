#include "main.h"

using namespace okapi;
Controller controller;

// inline int frontLeftMotorPort = -1;
// inline int frontRightMotorPort = 2;
// inline int bottomRightMotorPort = 3;
// inline int bottomLeftMotorPort = -4;

// inline double maximum_velocity = 400;
// inline double distancePIDCoefficient[] = {0.001, 0, 0.00001};
// inline double turnPIDCoefficient[] = {0.001, 0, 0.00001};
// inline double anglePIDCoefficient[] = {0.001, 0, 0.00001};


// QLength wheelDiameter = 4_in;
// QLength wheeltrackLength = 13.38_in;
// QLength trackingWheelDiameter = 2.75_in;
// QAngle turnRightAngle = 163.7_deg;
OdomState position;
OdomStateSI positionSI;


// char leftEncoderPort[] = {'A', 'B'};
// char rightEncoderPort[] = {'C', 'D'};
// char middleEncoderPort[] = {'E', 'F'};

Motor LFMotor(-1);
Motor RFMotor(2);
Motor RBMotor(3);
Motor LBMotor(-4);
Motor FlywheelMotor1(5);
Motor FlywheelMotor2(6);
ADIEncoder leftTW = ADIEncoder(leftEncoderPort[0], leftEncoderPort[1]);
ADIEncoder rightTW = ADIEncoder(rightEncoderPort[0], rightEncoderPort[1]);

pros::Imu imu_sensor(10);