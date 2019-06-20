#define MAX_SPEED 200.0
#define MAX_BACK_SPEED 140.0

float BACK_SPEED = 70.0;

#define INTERRUPT_PIN 39
#define SOLENOID_PIN 27
#define BUTTON_PIN 12
#define FRONT_IR_PIN 20
#define BACK_IR_PIN 16
#define RED_PIN 14
#define GREEN_PIN 10
#define BLUE_PIN 9
#define WHITEA_PIN 17
#define WHITEB_PIN 28

#define FIELD_WIDTH 183
#define FIELD_LENGTH 244

#define MINIMUM_HAS_BALL_X 40
#define MAXIMUM_HAS_BALL_X 60
#define MINIMUM_HAS_BALL_Y -15
#define MAXIMUM_HAS_BALL_Y 15

#define PIVOT_K 2.0

#define MAXIMUM_SHOT_DISTANCE 90.0
#define CORNER_WALL_DISTANCE 63.0

#define MAX_FAILED_BALL_READS 4
#define MAX_FAILED_GOAL_READS 30

bool gyroHathBeenSet = false;

float ballAngle;
float goalAngle;
int ballRanges [5] = {100, 100, 100, 100, 100};

float lastCalledTurnToShoot = 0.0;

float lastChangedStrategy = 0.0;

float lastHadBall = 0.0;

float lastShootTime = 0.0;

float lastBallReadTime = 0.0;
int lostBallDueToPosition = 0;
int lostBallDueToTOF = 0;

bool shouldKissForwards = false;

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

float frontDistance;
float backDistance;
float leftDistance;
float rightDistance;

//FROM BALL FOLLOWERS
float xTargetDiff = 1000;

float coneSize = 2.0;
float coneSizeIncreaseTime = 0;

bool hasClearShot = false; 
float strafeStatus = 0;
bool shouldStrafe = false;

float frontDistIR = 0.0;
bool isExecutingForwardSpinMove = false;
bool isExecutingDodgeOnGoalie = false;

float dodgeStartTime = 0.0;

bool turningToShoot = false;

//FROM STAGNANT FUNCTIONS
#define X_ORIGIN_CALIBRATION 40.0 
#define Y_ORIGIN_CALIBRATION 8.0 
#define PATH_CURVINESS 3.0
#define TARGET_DIST_BEHIND_BALL 110.0
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480
