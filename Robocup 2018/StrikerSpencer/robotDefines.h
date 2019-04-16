

void MotorsInit();

#define STOP 0

/*

   GLOBAL VARIABLES

*/
enum State {ON_LINE, HAS_BALL, DOESNT_SEE_BALL, SEES_BALL};
State currentState = DOESNT_SEE_BALL; //initial state

/****Motor Variables****/
unsigned int _offsetM1;
unsigned int _offsetM2;
unsigned int _offsetM3;
unsigned int _offsetM4;

static const unsigned char _M1PWM = 3;
static const unsigned char _M2PWM = 4;
static const unsigned char _M3PWM = 29;
static const unsigned char _M4PWM = 30;

unsigned char _M1DIR = 2;
unsigned char _M2DIR = 5;
unsigned char _M3DIR = 6;
unsigned char _M4DIR = 7;

unsigned char _M1CS = A15;
unsigned char _M2CS = A17;
unsigned char _M3CS = A18;
unsigned char _M4CS = A19;

int max_speed = 255;
int pause = 10;

//******CAMERA********//
double ballAngle;
int lastTimeSawBall = 0;
boolean spinningToBall = false;
double xPos;
double yPos;

//****QTR*******//
#define INTERRUPT_PIN           39


//---------------------IMU---------------------//
Adafruit_BNO055 bno = Adafruit_BNO055();
float g_goal = 0; //magnetic direction of goal
float g_facing = 0; //where you want robot to go
float g_error; //how far off goal robot is
float g_xPos; //current x-position of compass (0-360)
double gAngle; //global angle
uint8_t sys, gyro, accel, mag;


//-------------------LED--------------------//
//LED Defines
#define RED_PIN   21
#define GREEN_PIN 23
#define BLUE_PIN  22

//--------------------LIDARS-----------------//
int numLidars = 4;
int lidarPinArray[] = {15, 20, 17, 16}; //SS pins: 20 = front, 17 = left, 16 = back, 15 = (nonexistent)
char lidarI2cAdress[] = {0x64, 0x66, 0x68, 0x62}; //0x62 must always be the last one
int rightDist;
int leftDist;
int frontDist;
int backDist;

//--------------------OTHER-----------------//
boolean facingGoal = false;
char hemisphere;
int backwardSpeedWithBall  = 110;
int forwardSpeedWithBall =  220;
int sideSpeedWithBall = 150;

//---------------------Distance IR---------------------//
SharpIR distIR(A15, 1080);

