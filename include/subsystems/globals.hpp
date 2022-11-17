/**
 * @file globals.hpp
 *
 * @brief Initializes global objects
 *
 */

using namespace okapi;

typedef struct OdomStateSI {
	float x; // x position in meters
	float y; // y position in meters
	float theta; // rotation in degrees
	float xPercent; // x position in percentage
	float yPercent; // y position in percentage
}OdomStateSI;

enum odomMode {THREEWHEEL, TWOWHEELIMU, BASIC}; // Odometry mode
enum teamColor {REDTEAM, BLUETEAM}; // Allnighters team color 

// 15 second autonomous procedures
enum AutoProcedure 
{
	RED_FIRST_SCORING,
	RED_FIRST_SUPPORTIVE,
	RED_SECOND_SCORING,
	RED_SECOND_SUPPORTIVE,

	BLUE_FIRST_SCORING,
	BLUE_FIRST_SUPPORTIVE,
	BLUE_SECOND_SCORING,
	BLUE_SECOND_SUPPORTIVE,

	IDLE_FIRST,
	IDLE_SECOND,

	DQ
};

// disk shooting behavior
enum shootMode
{
	ACCURATE_MODE, // don't shoot until the flywheel is running in desired velocity
	FORCE_MODE // shoot immediately when command is called
};


/**
 * Motor ports
*/
inline int frontLeftMotorPort = 1;
inline int frontRightMotorPort = 2;
inline int bottomRightMotorPort = -3;
inline int bottomLeftMotorPort = -4;

inline int flywheelMotorPort1 = 5;
inline int flywheelMotorPort2 = -6;
inline int intakeMotorPort = 7;
inline int rollerMotorPort = 7;
inline int indexerMotorPort = -11;

/**
 * Sensor ports
*/

inline int visionPort = 12;
inline int imuSensorPort1 = 9;
inline int imuSensorPort2 = 10;
inline char loadSensorPort = 'A';
inline char intakeSensorPort = 'B';

/**
 * Encoder ports
*/
// inline char leftEncoderPort[] = {'A', 'B'};
inline char rightEncoderPort[] = {'C', 'D'};
inline char middleEncoderPort[] = {'E', 'F'};

/**
 * ADI out ports
*/

inline char pistonPort1 = 'G';
inline char pistonPort2 = 'H';

/**
 * Driver Controls
*/
inline ControllerAnalog ForwardAxis = ControllerAnalog::leftY;
inline ControllerAnalog TurnAxis = ControllerAnalog::rightX;

inline ControllerDigital IntakeButton = ControllerDigital::B;
inline ControllerDigital RollerUpButton = ControllerDigital::L1;
inline ControllerDigital RollerDownButton = ControllerDigital::L2;
inline ControllerDigital AimButton = ControllerDigital::down;
inline ControllerDigital ShootButton = ControllerDigital::R1;
inline ControllerDigital TripleShootButton = ControllerDigital::R2;
inline ControllerDigital ExpansionButton = ControllerDigital::X;

/**
 * Goal position corordinates
*/
inline float redHighGoalPosition_m[] = {0.45, 3.15, 0.76835};
inline float blueHighGoalPosition_m[] = {3.15, 0.45, 0.76835};
inline float redHighGoalPosition_percent[] = {14.54, 86, 20.99};
inline float blueHighGoalPosition_percent[] = {86, 14.54, 20.99};

/**
 * Flywheel constants
*/
inline float aimAngleDeviation = 20; // deviation of angle when aiming a point
inline float g = 9.81; // gravitational constant
inline float maxEjectVel = 8; // maximum eject velocity in m/s
inline float minEjectVel = 1; // minimum eject velocity in m/s
inline float Cv = 2; // vertical drag coefficient
inline float Ch = 2; // horizontal drag coefficient
inline float velocityLossConstant = 0.7; // velocity gain to calculate velocity loss due to the disk 
inline float Av = 0.015393804; // vertical cross section area
inline float Ah = 0.0028; // horizontal cross section area
inline float m = 0.06; // mass of the disk
inline float p = 1.225; // air density

/**
 * Odometry state
*/
extern OdomState position; // getState() original value
extern OdomStateSI positionSI; // odometry state in SI units and percentage

/**
 * Hardware parameters
*/
inline QLength wheelDiameter = 4_in;
inline QLength wheeltrackLength = 13.38_in;
inline QLength middleEncoderDistance = 1_in;
inline QLength trackingWheelDiameter = 2.75_in;
inline QAngle turnRightAngle = 163.7_deg;
inline QLength flyWheelDiameter = 4_in;
inline QLength diskDiameter = 14_cm;
inline QMass diskMass = 0.06_kg;
inline float diskHorizontalArea = 0.015393804;
inline float diskVerticalArea = 0.0028;
inline QLength launcher_height = 0.31_m;
inline QAngle launch_angle = 45_deg;
inline float fieldLength = 3.6576;



extern teamColor team; // the team color of the robot
extern AutoProcedure auto_procedure_running;

extern Controller controller; // main controller
extern Controller partner; // partner controller

// chassis motors
extern Motor LFMotor;
extern Motor RFMotor;
extern Motor RBMotor;
extern Motor LBMotor;

// flywheel motors
extern Motor FlywheelMotor1;
extern Motor FlywheelMotor2;

// intake-roller motor
extern Motor IntakeMotor;
extern Motor RollerMotor;

extern Motor IndexerMotor;
extern pros::Vision vision_sensor;

extern ADIEncoder leftTW; // left tracking wheel
extern ADIEncoder rightTW; // right tracking wheel
extern ADIEncoder midTW; // middle tracking wheel

extern pros::ADIDigitalOut piston1;
extern pros::ADIDigitalOut piston2;
extern pros::ADIDigitalIn load_sensor;
extern pros::ADIDigitalIn intake_sensor;

extern pros::Imu imu_sensor_1;
extern pros::Imu imu_sensor_2;