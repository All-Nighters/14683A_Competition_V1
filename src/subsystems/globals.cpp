#include "main.h"

using namespace okapi;

teamColor team = REDTEAM;

OdomState position;
OdomStateSI positionSI;

Controller controller(okapi::ControllerId::master);
Controller partner(okapi::ControllerId::partner);

Motor LFMotor(-1);
Motor RFMotor(2);
Motor RBMotor(3);
Motor LBMotor(-4);
Motor FlywheelMotor1(5);
Motor FlywheelMotor2(6);
Motor IntakeMotor(7);
Motor RollerMotor(8);

// remember to also change the reversin in odometry.cpp
ADIEncoder leftTW = ADIEncoder(leftEncoderPort[0], leftEncoderPort[1], true);
ADIEncoder rightTW = ADIEncoder(rightEncoderPort[0], rightEncoderPort[1]);

pros::ADIDigitalOut indexer('G');
pros::ADIDigitalOut piston('H');

pros::Imu imu_sensor(10);