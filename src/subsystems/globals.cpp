#include "main.h"

using namespace okapi;

teamColor team = REDTEAM;
AutoProcedure auto_procedure_running;

OdomState position;
OdomStateSI positionSI;

Controller controller(okapi::ControllerId::master);
Controller partner(okapi::ControllerId::partner);

Motor LFMotor(frontLeftMotorPort);
Motor RFMotor(frontRightMotorPort);
Motor RBMotor(bottomRightMotorPort);
Motor LBMotor(bottomLeftMotorPort);
Motor FlywheelMotor1(flywheelMotorPort1);
Motor FlywheelMotor2(flywheelMotorPort2);
Motor IntakeMotor(intakeMotorPort);
Motor RollerMotor(rollerMotorPort);
Motor IndexerMotor(indexerMotorPort);
pros::Vision vision_sensor(visionPort);

// remember to also change the reversin in odometry.cpp
// ADIEncoder leftTW = ADIEncoder(leftEncoderPort[0], leftEncoderPort[1], false);
ADIEncoder rightTW = ADIEncoder(rightEncoderPort[0], rightEncoderPort[1], true);
ADIEncoder midTW = ADIEncoder(middleEncoderPort[0], middleEncoderPort[1], true);

pros::ADIDigitalOut piston1(pistonPort1);
pros::ADIDigitalOut piston2(pistonPort2);
pros::ADIDigitalIn load_sensor(loadSensorPort);
pros::ADIDigitalIn intake_sensor(intakeSensorPort);

pros::Imu imu_sensor_1(imuSensorPort1);
pros::Imu imu_sensor_2(imuSensorPort2);