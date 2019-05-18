//FROM MASTER
#define MAX_SPEED 230.0
#define MAX_BACK_SPEED 120.0

#define INTERRUPT_PIN 39
#define SOLENOID_PIN 27
#define BUTTON_PIN 12
#define FRONT_IR_PIN 20
#define RED_PIN 14
#define GREEN_PIN 10
#define BLUE_PIN 9
#define WHITEA_PIN 17
#define WHITEB_PIN 28

#define FIELD_WIDTH 185
#define FIELD_LENGTH 244

#define PIVOT_K 4.0

float ballAngle;
float goalAngle;
int ballRanges [5] = {100, 100, 100, 100, 100};

float lastBallReadTime = 0.0;
int lostBallDueToPosition = 0;


enum State { has_ball, sees_ball, invisible_ball};
State state = invisible_ball;

float xPos = 1;
float yPos = 1;
float oldXPos = 0.0;
float oldYPos = 0.0;
int failedBallReadingCount = 0;
float tPos = 1;
float oPos = 1;
float oldTPos = 0.0;
float oldOPos = 0.0;
int failedGoalReadingCount = 0;

float frontSensor;
float backSensor;
float leftSensor;
float rightSensor;
float oldFrontSensor;
float oldBackSensor;
float oldLeftSensor;
float oldRightSensor;
int failedFrontReadCount = 0;
int failedBackReadCount = 0;
int failedLeftReadCount = 0;
int failedRightReadCount = 0;

bool interrupted = false;
bool turnFixed = false;

bool readyToShoot = false;


//FROM BALL FOLLOWERS
float xTargetDiff = 1000;

float coneSize = 2.0;
float coneSizeIncreaseTime = 0;

//FROM HAS BALL
#define DODGE_RADIUS 8.0
// Inches per second
#define SPEED_ESTIMATE 30.0
#define MAXIMUM_SHOT_DISTANCE 400
#define CORNER_WALL_DISTANCE 63.0

#define TOTAL_DODGE_TIME 1000.0*DODGE_RADIUS*PI/SPEED_ESTIMATE

bool hasClearShot = false; 
float strafeStatus = 0;
bool shouldStrafe = false;

float frontDistIR = 0.0;
bool isExecutingForwardSpinMove = false;
bool isExecutingDodgeOnGoalie = false;

float dodgeStartTime = 0.0;

bool turningToShoot = false;

//FROM STAGNANT FUNCTIONS
#define X_ORIGIN_CALIBRATION 30.0 
#define Y_ORIGIN_CALIBRATION 5.0 
#define PATH_CURVINESS 3.0
#define TARGET_DIST_BEHIND_BALL 110.0
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480
