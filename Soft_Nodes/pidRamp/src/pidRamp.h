/* Defines */

#define ROLLBONUS		2

#define CATCHRATE		20
#define CATCHRATEF		40
#define MAXSPEED		50
#define	MINSPEED		-50

#define MAXSPEEDD		90
#define	MINSPEEDD		-90

#define	PITCH			0
#define	LEFT			1
#define RIGHT			2
#define DEPTHR			3
#define DEPTHL			4

#define	SCALAR			5
#define	MIN_DUTY_CYCLE_US	1000
#define	MAX_DUTY_CYCLE_US	2000
#define	ZERO_DUTY_CYCLE_US	(MIN_DUTY_CYCLE_US + MAX_DUTY_CYCLE_US)/2

/* Globals */

int targetRate[5];
int currentRate[5];
int outRate[5];
unsigned int first = 1;
float speed, roll;
unsigned int go = 0;

/* Function Declerations */

void goCallback(const std_msgs::UInt32::ConstPtr& pilotGo);
void pitchRateCallback(const std_msgs::Float32::ConstPtr& pitchRate);
void leftRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
void rightRateCallback(const std_msgs::Float32::ConstPtr& pidRate);
void depthRRateCallback(const std_msgs::Float32::ConstPtr& depthRRate);
void depthLRateCallback(const std_msgs::Float32::ConstPtr& depthLRate);
void speedCallback(const std_msgs::Float32::ConstPtr& pilotSpeed);
void rollCallback(const std_msgs::Float32::ConstPtr& compassRoll);
unsigned int slewer(unsigned int pos);

