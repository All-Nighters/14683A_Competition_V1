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
Motor FlywheelMotor2(-6);
Motor IntakeMotor(7);
Motor RollerMotor(8);
Motor IndexerMotor(-11);
pros::Vision vision_sensor(12);

// remember to also change the reversin in odometry.cpp
// ADIEncoder leftTW = ADIEncoder(leftEncoderPort[0], leftEncoderPort[1], false);
ADIEncoder rightTW = ADIEncoder(rightEncoderPort[0], rightEncoderPort[1], true);
ADIEncoder midTW = ADIEncoder(middleEncoderPort[0], middleEncoderPort[1], false);

pros::ADIDigitalOut indexer('G');
pros::ADIDigitalOut piston('H');
pros::ADIDigitalIn load_sensor('A');
pros::ADIDigitalIn intake_sensor('B');

pros::Imu imu_sensor_1(9);
pros::Imu imu_sensor_2(10);