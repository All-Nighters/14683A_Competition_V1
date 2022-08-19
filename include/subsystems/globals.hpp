using namespace okapi;

typedef struct OdomStateSI {
	float x;
	float y;
	float theta;
	float xPercent;
	float yPercent;
	// float facing;
}OdomStateSI;



extern Controller controller;
extern Motor LFMotor;
extern Motor RFMotor;
extern Motor RBMotor;
extern Motor LBMotor;
extern Motor FlywheelMotor1;
extern Motor FlywheelMotor2;
extern ADIEncoder leftTW;
extern ADIEncoder rightTW;

inline int frontLeftMotorPort = -1;
inline int frontRightMotorPort = 2;
inline int bottomRightMotorPort = 3;
inline int bottomLeftMotorPort = -4;
inline int fieldLength = 3.6576;

inline float maximum_velocity = 400;
inline float distancePIDCoefficient[] = {0.001, 0, 0.00001};
inline float turnPIDCoefficient[] = {0.001, 0, 0.00001};
inline float anglePIDCoefficient[] = {0.001, 0, 0.00001};
inline float redHighGoalPosition_m[] = {0.45, 3.15, 0.76835};
inline float blueHighGoalPosition_m[] = {3.15, 0.45, 0.76835};
inline float redHighGoalPosition_percent[] = {12.33, 86, 20.99};
inline float blueHighGoalPosition_percent[] = {86, 12.33, 20.99};

extern OdomState position;
extern OdomStateSI positionSI;

inline QLength wheelDiameter = 4_in;
inline QLength wheeltrackLength = 13.38_in;
inline QLength trackingWheelDiameter = 2.75_in;
inline QAngle turnRightAngle = 163.7_deg;
inline QLength flyWheelDiameter = 4_in;
inline QLength diskDiameter = 14_cm;
 
inline char leftEncoderPort[] = {'A', 'B'};
inline char rightEncoderPort[] = {'C', 'D'};
inline char middleEncoderPort[] = {'E', 'F'};

extern pros::Imu imu_sensor;