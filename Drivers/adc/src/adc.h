/* Defines */

#define GO	0

#define ADCTHRESH 100

/* Globals */

unsigned int counter = 0;

unsigned int accRaw = 0;

unsigned int latch = 0;

int goTmp = 0;

/* Function Declerations */

unsigned int checkGo(void);
void readADC(void);
