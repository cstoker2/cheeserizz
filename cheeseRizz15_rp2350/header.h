//DEBUGGING
#define CRSF_DEBUG 0    // show all crf channels on serial
#define DEBUG_HOTLOOP 0 // debugs in calculateW, other melty loop unctions
#define DEBUG_HOTHZ 1 // debugs counts hz of loop
#define DEBUG_SERIAL 1  // Serial debug output toggle
#define DEBUG2_SERIAL 0  // Serial debug output toggle
#define DEBUG_TIMERS 0  // Serial debug output toggle
#define DEBUG_ACCEL 1  // Serial debug output toggle
#define DEBUG_TELEMETRY 1 // send serial2 data for espnow module

//#define PIN_LED 25 // D19 xiao, LOW = on this is in the board definition
#define NEOPIXEL_BUILTIN 22
#define NEOPIXEL_POWER 23
#define NUMPIXELS 11  // Number of LEDs in dotstar strip

// Dotstar LEDS:
#define DATAPIN D4
#define CLOCKPIN D5

// Used for  SPI of accelerometer
#define H3LIS331_SCK D8    //13
#define H3LIS331_MISO D9   //12
#define H3LIS331_MOSI D10  ///11
#define H3LIS331_CS D3     // 10

//DSHOT
#define DSHOT1_PIN 26  //D0 xiao2350
#define DSHOT2_PIN 27  //D1 xiao2350

//Interrupt timers
#define ACCEL_IRQ_INTERVAL 1 // interrupt in ms 1000hz
#define DSHOT_IRQ_INTERVAL 1.6 // interrupt in ms 625hz
#define LED_IRQ_INTERVAL 100 // interrupt in ms 10hz

// Motor direction constants
#define MOTOR1_DIRECTION 1.0   // Set to -1 to reverse motor 1, 1 for normal
#define MOTOR2_DIRECTION 1.0   // Set to -1 to reverse motor 2, 1 for normal
#define HEADING_SMOOTHING 0.85  // Smoothing factor for heading updates (0-1)

//colors: rgb
#define RED 0xFF0000
#define GREEN 0x00FF00
#define BLUE 0x0000FF
#define WHITE 0xFFFFFF
#define OFF 0x000000

// LED pattern definitions
#define LED_PATTERN_ERROR 0xDEDE, 0x0000, 0x0000           // Red flashing sos
#define LED_PATTERN_RADIO_ERROR 0xF0F0, 0x0000, 0x0F00     // Red/blue flashing
#define LED_PATTERN_THROTTLE_ERROR 0x0c0c, 0xF0F0, 0x0F0F  // blue green flashing
#define LED_PATTERN_CALIBRATION 0x5555, 0x5555, 0x0000     // Yellow (R+G) fast
#define LED_PATTERN_GREEN_S 0x0000, 0xFFFF, 0x0000         // solid green
#define LED_PATTERN_GREEN_F 0x0000, 0x0F0F, 0x0000         // Flash green
#define LED_PATTERN_OFF 0x0000, 0x0000, 0x0000             // off // also need to disable led interrupt in meltymode
#define LED_PATTERN_CYCLE 0x1000, 0x0F00, 0x00F0           // cycles rgb
#define LED_PATTERN_LOWRPS 0x0000, 0x0F0F, 0xF0F0          // gb flash

//Variables:
int rcChannelCount = crsfProtocol::RC_CHANNEL_COUNT;
const char *rcChannelNames[] = {
  "A",  //1
  "E",  //2
  "T",  //3
  "R",  //4
  "--",  //5 sA
  "sB",  //6 sB
  "sC",  //7 sC
  "p1",  //8 sD
  "sE",
  "p1",  //
  "Aux7",
  "Aux8",
  "Aux9",
  "Aux10",
  "Aux11",
  "Aux12"
};
bool FAILSAFE = true;  //global failsafe variable, set in onRecieveChannels
uint32_t rpm = 0;
uint16_t pulse = 0;  // pulse led

//initial acceleration offsets
float accelOffsetX;
float accelOffsetY;
float accelOffsetZ;
float estimated_accel;  // for kalman filter
volatile uint32_t hotMicros = 0;  // dshot interrupt microseconds
volatile float hotHz = 0;
sensors_event_t accel_event;
uint32_t sensMicros = 0;  // sensor interrupt measurment microseconds
float sensHz = 0;
uint32_t ledMicros = 0;  // led interrupt microseconds
float ledHz = 0;
uint32_t dshotMicros = 0;  // dshot interrupt microseconds
float dshotHz = 0;

// Global variables for motor throttles
volatile float motor1Throttle = 0.0;
volatile float motor2Throttle = 0.0;
int lastMotor1 = -1;
int lastMotor2 = -1;
const float ZERO_THROTTLE_THRESHOLD = 0.1;  // Threshold for detecting throttle input

// Constants for motor control
const float boostThreshold = 0.2;        // Minimum throttle value for motor to start spinning
const float boostSpeed = 0.5;            // Throttle output during "on" period when modulating
const unsigned long baseCycleTime = 50;  // Base boost cycle time in milliseconds
unsigned long endBoost[2] = { 0, 0 };
bool isBoost[2] = { true, true };  // Flag to track whether we're sending modSpeed or 0
const float RPS_THRESHOLD = 7.0;   // 7 revolutions per second = 420 RPM
volatile float maxRPS = 0.0;       // Stores the maximum RPS value
volatile float lastRPS = 0.0;      // stored for logging
//const unsigned long LOOP_DELAY_MICROS = 200;  // Delay for each loop iteration in microseconds

// Variables to store inputs
volatile float stickVert = 0.0;      // up/down = ch0
volatile float stickHoriz = 0.0;     // left/right = ch1
volatile float stickAngle = 0.0;     // Stick angle as a fraction of a circle
volatile float stickLength = 0.0;    // Length of the stick vector
volatile float throttle = 0.0;       // Throttle input = ch2
//volatile float sidewaysInput = 0.0;  // Sideways input = ch7 right toggle for tank drive
volatile float rudderInput = 0.0;    // rudder input = ch3
volatile float radiusInput = 0.0;    // Radius input = ch4
volatile float radiusSize = 0.01;    // Mapped radius size in meters
volatile float ledOffset = 0.0;      // offset leds from 12 o'clock
volatile float kalmanQ = 0.01;       // rateparam for kalman
volatile float kalmanInput = 0.0;    // input placeholder
volatile float inputPot = 0.0;       // ch7 right potentiometer
volatile float inputToggleL = 0.0;    // ch5 left 3-way toggle
volatile float inputToggleR = 0.0;    // ch6 right 3-way toggle

// Phase tracking variables
static float continuousPhase = 0.0f;       // Current phase position (0.0-1.0)
static float previousW = 0.0f;             // Previous angular velocity for trapezoidal integration
static unsigned long lastPhaseUpdate = 0;  // Timestamp of last phase update (microseconds)

//LED states (16-bit values) for flash sequences
volatile bool ledInterruptsEnabled = true;
volatile uint16_t redState = 0xAAAA;    // 1010101010101010 Example pattern for red (alternates on/off)
volatile uint16_t greenState = 0x0F0F;  // 0000111100001111 Example pattern for green (on/off in 128ms blocks)
volatile uint16_t blueState = 0xFFFF;   // 1111111111111111 Example pattern for blue (always on)
volatile uint8_t rbit;
volatile uint8_t gbit;
volatile uint8_t bbit;

//Telemetry Labels
const char *telemLbl[] = {
  "CheeseRizz",  // f1: , version number
  "Failsafe",      // fl2:
  "hotHZ",       //fl3:
  "m1Throttle",      // fl4:
  "m2Throttle",      // fl5:
  "radmm",      // fl6:
  "rpm",      // fl7:
  "accelZ"      // fl8:
};
//telemetry Values
volatile float telemVal[] = {
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
};