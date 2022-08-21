#include "main.h"

using namespace okapi;

// 0 for red, 1 for blue
int teamColor = 0;

OdomState position;
OdomStateSI positionSI;

Controller controller;

Motor LFMotor(-1);
Motor RFMotor(2);
Motor RBMotor(3);
Motor LBMotor(-4);
Motor FlywheelMotor1(5);
Motor FlywheelMotor2(6);
Motor IntakeMotor(7);

ADIEncoder leftTW = ADIEncoder(leftEncoderPort[0], leftEncoderPort[1]);
ADIEncoder rightTW = ADIEncoder(rightEncoderPort[0], rightEncoderPort[1]);

pros::ADIDigitalOut indexer('G');

pros::Imu imu_sensor(10);