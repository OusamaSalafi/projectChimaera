/* Defines */

#define	PWM_FREQUENCY_US	20000
#define	MIN_DUTY_CYCLE_US	1000
#define	MAX_DUTY_CYCLE_US	2000
#define	ZERO_DUTY_CYCLE_US	(MIN_DUTY_CYCLE_US + MAX_DUTY_CYCLE_US)/2

#define DEPTH_RIGHT_CHANNEL 	2
#define DEPTH_LEFT_CHANNEL 	3
#define YAW_RIGHT_CHANNEL 	4
#define YAW_LEFT_CHANNEL 	5
#define PITCH_CHANNEL 		6

#define	DEPTH_RIGHT_PWM_OFFSET	0
#define	DEPTH_LEFT_PWM_OFFSET	0
#define	YAW_RIGHT_PWM_OFFSET	0
#define	YAW_LEFT_PWM_OFFSET		0
#define	PITCH_PWM_OFFSET		0

#define	DEBUGSPEED		1800

/* Globals */

unsigned int 	depthRightPWM,
				depthLeftPWM,
				yawRightPWM,
				yawLeftPWM,
				pitchPWM;

unsigned int go = 0, go2 = 0, go3 = 0, go4 = 0, go5 = 0;

/* Function Declarations */

void goCallback(const std_msgs::UInt32::ConstPtr& pilotGo);
void updatePWM(unsigned int channel, unsigned int rate);

void depthRightCallback(const std_msgs::UInt32::ConstPtr& pidRampDepthRight);
void depthLeftCallback(const std_msgs::UInt32::ConstPtr& pidRampDepthLeft);
void yawRightCallback(const std_msgs::UInt32::ConstPtr& pidRampYawRight);
void yawLeftCallback(const std_msgs::UInt32::ConstPtr& pidRampYawLeft);
void pitchCallback(const std_msgs::UInt32::ConstPtr& pidRampPitch);

void alertCallback(const std_msgs::UInt32::ConstPtr& alertFront);
int initMotors(void);
