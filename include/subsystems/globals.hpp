using namespace okapi;

typedef struct OdomStateSI {
	float x;
	float y;
	float theta;
	float xPercent;
	float yPercent;
}OdomStateSI;

extern int teamColor; // 0 for red, 1 for blue

extern Controller controller;
extern Motor LFMotor;
extern Motor RFMotor;
extern Motor RBMotor;
extern Motor LBMotor;
extern Motor FlywheelMotor1;
extern Motor FlywheelMotor2;
extern Motor IntakeMotor;

extern ADIEncoder leftTW; // left tracking wheel
extern ADIEncoder rightTW; // right tracking wheel

extern pros::ADIDigitalOut indexer; // indexer piston
extern pros::Imu imu_sensor;

/**
 * Motor ports
*/
inline int frontLeftMotorPort = -1;
inline int frontRightMotorPort = 2;
inline int bottomRightMotorPort = 3;
inline int bottomLeftMotorPort = -4;

/**
 * Encoder ports
*/
inline char leftEncoderPort[] = {'A', 'B'};
inline char rightEncoderPort[] = {'C', 'D'};
inline char middleEncoderPort[] = {'E', 'F'};

/**
 * Goal position corordinates
*/
inline float redHighGoalPosition_m[] = {0.45, 3.15, 0.76835};
inline float blueHighGoalPosition_m[] = {3.15, 0.45, 0.76835};
inline float redHighGoalPosition_percent[] = {12.33, 86, 20.99};
inline float blueHighGoalPosition_percent[] = {86, 12.33, 20.99};

/**
 * Constants
*/

inline float aimAngleDeviation = 10; // deviation of angle when aiming a point
inline float g = 9.81; // gravitational constant
inline float maxEjectVel = 8; // maximum eject velocity in m/s
inline float minEjectVel = 1; // minimum eject velocity in m/s
inline float Cv = 2; // vertical drag coefficient
inline float Ch = 2; // horizontal drag coefficient
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
inline QLength trackingWheelDiameter = 2.75_in;
inline QAngle turnRightAngle = 163.7_deg;
inline QLength flyWheelDiameter = 4_in;
inline QLength diskDiameter = 14_cm;
inline int fieldLength = 3.6576;