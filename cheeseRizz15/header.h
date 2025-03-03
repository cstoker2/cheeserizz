#define LED_COUNT_TOP 8        // 8 top
#define LED_COUNT_BOT 2        // 2 bottom
#define BRIGHTNESS 100         // Set BRIGHTNESS (max = 255) keep low for testing
#define I2C_ADDR_ACCEL 0x19    // sparkfun
#define SERIAL_IBUS Serial2    // pin 7 serial2
#define SERIAL_TX_LEDS 17      // top string of 8 leds
#define SERIAL_TX_LEDS_BOT 20  // bottom string of 2 leds
#define WIRE_ACCELMAG Wire     // 19 scl, 18 sda
#define PIN_ESC1 15            //blue wire -> left motor
#define PIN_ESC2 14            //yellow wire -> right motor
#define PIN_MAG_INT 16         // magnetometer interrupt

// Motor direction constants
#define MOTOR1_DIRECTION -1.0       // Set to -1 to reverse motor 1, 1 for normal
#define MOTOR2_DIRECTION -1.0       // Set to -1 to reverse motor 2, 1 for normal
#define MAG_BUFFER_SIZE 128         // Buffer size for peak/valley detection
#define HEADING_SMOOTHING 0.85      // Smoothing factor for heading updates (0-1)
#define MIN_PEAK_VALLEY_DIFF 10000  // Minimum difference to qualify as a peak/valley

// Replace the DiagnosticData structure with this more flexible structure
#define TELEMETRY_BUFFER_SIZE 256  // Size of telemetry data buffer to hold a full rotation

// Phase tracking variables
static float continuousPhase = 0.0f;        // Current phase position (0.0-1.0)
static float previousW = 0.0f;              // Previous angular velocity for trapezoidal integration
static unsigned long lastPhaseUpdate = 0;    // Timestamp of last phase update (microseconds)


// Generic telemetry data structure
struct TelemetryData {
  uint32_t timestamp;  // microseconds timestamp
  float fl1, fl2, fl3, fl4, fl5, fl6, fl7, fl8, fl9, fl10;  // 10 flexible float values
  int32_t int1, int2, int3;  // 3 flexible integer values
};

// Create buffer and tracking variables
TelemetryData telemetryBuffer[TELEMETRY_BUFFER_SIZE];
uint16_t telemetryBufferIndex = 0;
uint16_t telemetryBufferCount = 0;

// String constants for telemetry field labels
const char* telemetryLabels[] = {
  "timestamp",
  "magHeading",         // fl1: Magnetometer heading (0-1)
  "accelPhase",         // fl2: Phase calculated from accelerometer (0-1)
  "targetPhase",        // fl3: Target phase from stick input
  "ledPhase",           // fl4: LED target phase
  "forwardDiff",        // fl5: Phase difference for forward direction
  "backwardDiff",       // fl6: Phase difference for backward direction
  "ledDiff",            // fl7: Phase difference for LED
  "cos_ph1",            // fl8: Cosine value for forward motor
  "currentRPS",         // fl9: Current Revolutions Per Second
  "throttle",           // fl10: Throttle value (0-1)
  "hotLoopCount",       // int1: Iteration count in hot loop
  "LEDOn",              // int2: LED state (1=on, 0=off)
  "unused"              // int3: Currently unused
};

// Magnetometer data structure
struct MagData {
  uint32_t rawX;
  uint32_t rawY;
  uint32_t rawZ;
  uint32_t timestamp;
  bool calibrated;   //flag to track calibration status
};

// Magnetometer global variables
// Calibration variables
int32_t magXOffset = 131072;  // Default mid-point (2^17)
int32_t magYOffset = 131072;
int32_t magZOffset = 131072;
float magXScale = 1.0;
float magYScale = 1.0;
volatile bool newMagDataAvailable = false;
MagData magBuffer[MAG_BUFFER_SIZE];
uint8_t magBufferIndex = 0;
uint8_t magBufferCount = 0;
int32_t xMax = 0, xMin = 262144;  // 18-bit max value is 262144
int32_t yMax = 0, yMin = 262144;
uint32_t xMaxTime = 0, xMinTime = 0, yMaxTime = 0, yMinTime = 0;
uint32_t lastPeakValleyDetectionTime = 0;

float robotHeading = 0.0;         // Current heading in degrees (0-360)
float headingCorrection = 0.0;    // Offset to align with desired forward direction
float lastHeading = 0.0;          // Last stable heading for validation
bool headingInitialized = false;  // Flag to indicate if we have a valid heading
float RPMestimate = 0.0;          // Estimated RPM from magnetometer

//colors: wrgb
#define RED 0x00FF0000
#define GREEN 0x0000FF00
#define BLUE 0x000000FF
#define WHITE 0xFF000000
#define OFF 0x00000000

// Variables to store the current LED states (16-bit values) for flash sequences
volatile bool ledInterruptsEnabled = true;
volatile uint16_t redState = 0xAAAA;    // 1010101010101010 Example pattern for red (alternates on/off)
volatile uint16_t greenState = 0x0F0F;  // 0000111100001111 Example pattern for green (on/off in 128ms blocks)
volatile uint16_t blueState = 0xFFFF;   // 1111111111111111 Example pattern for blue (always on)
volatile uint16_t whiteState = 0;
volatile uint8_t rbit;
volatile uint8_t gbit;
volatile uint8_t bbit;

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

// Define the ESC servo rate (typically 50Hz for most ESCs)
const int ESC_SERVO_RATE = 400;
const float ZERO_THROTTLE_THRESHOLD = 0.1;  // Threshold for detecting throttle input
// PWM frequency is 400Hz, so 1 cycle is 2500us.
const int PWM_1000us = 65536 * 40 / 100;  // 40% of a 2500us cycle = 1000us
const int PWM_2000us = 65536 * 80 / 100;  // 80% of a 2500us cycle = 2000us
bool pwmIntCalled;

// for check radio state
volatile uint32_t last_ibus_seen_millis;
volatile uint8_t last_cnt_rec;
volatile uint8_t last_cnt_poll;

//initial acceleration offsets
float accelOffsetX;
float accelOffsetY;
float accelOffsetZ;
float estimated_accel;  // for kalman filter

// magnetometer offsets
static uint32_t magOffsetX = 131072;
static uint32_t magOffsetY = 131072;
static uint32_t magOffsetZ = 131072;

// Global variables for motor throttles
float motor1Throttle = 0.0;
float motor2Throttle = 0.0;
int lastMotor1 = -1;
int lastMotor2 = -1;

// Variables to store inputs
volatile float stickVert = 0.0;      // up/down = ch0
volatile float stickHoriz = 0.0;     // left/right = ch1
volatile float stickAngle = 0.0;     // Stick angle as a fraction of a circle
volatile float stickLength = 0.0;    // Length of the stick vector
volatile float throttle = 0.0;       // Throttle input = ch2
volatile float sidewaysInput = 0.0;  // Sideways input = ch7 right toggle for tank drive
volatile float rudderInput = 0.0;    // rudder input = ch3
volatile float radiusInput = 0.0;    // Radius input = ch4
volatile float radiusSize = 0.01;    // Mapped radius size in meters
volatile float ledOffset = 0.0;      // ch5 offset leds from 12 o'clock
volatile float kalmanQ = 0.01;       // rateparam for kalman
volatile float kalmanInput = 0.0;    // input placeholder
volatile float inputPot = 0.0;       // ch4 left potentiometer
volatile float inputToggle = 0.0;    // ch6 left 3-way toggle
volatile float heading = 0.0;
volatile uint32_t headingMark = 0;  // time of heading measurement in micros

//accelerometer  mag readings
volatile float accel_x = 0.0;
volatile float accel_y = 0.0;
volatile float accel_z = 0.0;
volatile float mag_x = 0.0;
volatile float mag_y = 0.0;
volatile float mag_z = 0.0;


unsigned long usRevStartTime = 0;  // Time in microseconds when the revolution started

// Constants for motor control
const float boostThreshold = 0.2;        // Minimum throttle value for motor to start spinning
const float boostSpeed = 0.5;            // Throttle output during "on" period when modulating
const unsigned long baseCycleTime = 50;  // Base cycle time in milliseconds
unsigned long endBoost[2] = { 0, 0 };
bool isBoost[2] = { true, true };             // Flag to track whether we're sending modSpeed or 0
const float RPS_THRESHOLD = 7.0;              // 7 revolutions per second = 420 RPM
volatile float maxRPS = 0.0;                  // Stores the maximum RPS value
volatile float lastRPS = 0.0;                 // stored for logging
const unsigned long LOOP_DELAY_MICROS = 200;  // Delay for each loop iteration in microseconds

// Debug Control
#define DEBUG_PRINT_INTERVAL 250     // Print interval in milliseconds
static uint32_t lastDebugPrint = 0;  // Tracks last debug print time

// Regular debug prints - no rate limiting
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(format, ...) Serial.printf(format, __VA_ARGS__)
#define BT_PRINT(x) Serial1.print(x)
#define BT_PRINTLN(x) Serial1.println(x)
#define BT_PRINTF(format, ...) Serial1.printf(format, __VA_ARGS__)

// Rate-limited debug prints
#define DEBUG_PRINT_RATE(x) \
  do { \
    if ((millis() - lastDebugPrint) > DEBUG_PRINT_INTERVAL) { \
      Serial.print(x); \
      lastDebugPrint = millis(); \
    } \
  } while (0)

#define DEBUG_PRINTLN_RATE(x) \
  do { \
    if ((millis() - lastDebugPrint) > DEBUG_PRINT_INTERVAL) { \
      Serial.println(x); \
      lastDebugPrint = millis(); \
    } \
  } while (0)

#define DEBUG_PRINTF_RATE(format, ...) \
  do { \
    if ((millis() - lastDebugPrint) > DEBUG_PRINT_INTERVAL) { \
      Serial.printf(format, __VA_ARGS__); \
      lastDebugPrint = millis(); \
    } \
  } while (0)
