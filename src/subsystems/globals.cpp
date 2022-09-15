#include "main.h"

using namespace okapi;

teamColor team = REDTEAM;

OdomState position;
OdomStateSI positionSI;

Controller controller(okapi::ControllerId::master);
Controller partner(okapi::ControllerId::partner);

Motor LFMotor(frontLeftMotorPort);
Motor RFMotor(frontRightMotorPort);
Motor RBMotor(bottomRightMotorPort);
Motor LBMotor(bottomLeftMotorPort);
Motor FlywheelMotor1(5);
Motor FlywheelMotor2(6);
Motor IntakeMotor(7);
Motor RollerMotor(8);

// remember to also change the reversin in odometry.cpp
ADIEncoder leftTW = ADIEncoder(leftEncoderPort[0], leftEncoderPort[1]);
ADIEncoder rightTW = ADIEncoder(rightEncoderPort[0], rightEncoderPort[1], true);

pros::ADIDigitalOut indexer('G');
pros::ADIDigitalOut piston('H');

pros::Imu imu_sensor(10);