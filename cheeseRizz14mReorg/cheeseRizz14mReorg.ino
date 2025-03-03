//Complified Creations meltybrain controller
// based on Melontype

#include <IBusBM.h>             //pin15 Serial3 RX
#include "Adafruit_H3LIS331.h"  //Wire on pin18,pin19
#include <WS2812Serial.h>       //pin17
#include <SimpleKalmanFilter.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include "header.h"
//#include "initialize.h"

// Declare objects and variables
IBusBM ibus;                                    // IBus object for the radio receiver
Adafruit_H3LIS331 accel = Adafruit_H3LIS331();  // on Wire at 0x19
SFE_MMC5983MA mag;                              // on Wire at 0x30                         // on Wire at 0x30
SimpleKalmanFilter kalmanFilter(20, 20, 0.001);

// top leds teensy ws2812 library
byte drawingMemory[LED_COUNT_TOP * 4];          //  4 bytes per LED for RGBW
DMAMEM byte displayMemory[LED_COUNT_TOP * 16];  // 16 bytes per LED for RGBW
WS2812Serial leds(LED_COUNT_TOP, displayMemory, drawingMemory, SERIAL_TX_LEDS, WS2812_GRBW);

//bottom leds teensy ws2812 library
byte drawingMemory_bot[LED_COUNT_BOT * 4];          //  4 bytes per LED for RGBW
DMAMEM byte displayMemory_bot[LED_COUNT_BOT * 16];  // 16 bytes per LED for RGBW
WS2812Serial ledsBottom(LED_COUNT_BOT, displayMemory_bot, drawingMemory_bot, SERIAL_TX_LEDS_BOT, WS2812_GRBW);

// IntervalTimer objects
IntervalTimer ledTimer;
IntervalTimer pwmTimer;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  delay(100);
  Serial.println("\n cheeseRizz14.ino");
  Serial.begin(115200);
  Serial1.begin(115200);  // connect BT12 module here, pin0 RX1, pin1 TX1
  Serial.println("Initializing Meltybrain Robot...");
  initLEDs();
  initESCs();
  initAccel();
  initMag();
  initIBus();
  Serial.println("Setup complete. Ready for operation.");
  digitalWrite(LED_BUILTIN, HIGH);  // power light on to show setup complete
}

void pwmInterruptHandler() {
  int pwmValueMotor1 = motor1Throttle;
  int pwmValueMotor2 = motor2Throttle;

  if (lastMotor1 != pwmValueMotor1) {
    analogWrite(PIN_ESC1, pwmValueMotor1);
    lastMotor1 = pwmValueMotor1;
  }
  if (lastMotor2 != pwmValueMotor2) {
    analogWrite(PIN_ESC2, pwmValueMotor2);
    lastMotor2 = pwmValueMotor2;
  }
  pwmIntCalled = true;
}

void magInterrupt() {
  // Capture timestamp immediately
  uint32_t timestamp = micros();

  // Read raw values
  uint32_t rawX, rawY, rawZ;
  mag.readFieldsXYZ(&rawX, &rawY, &rawZ);

  // Store in buffer
  magBuffer[magBufferIndex].rawX = rawX;
  magBuffer[magBufferIndex].rawY = rawY;
  magBuffer[magBufferIndex].rawZ = rawZ;
  magBuffer[magBufferIndex].timestamp = timestamp;
  magBuffer[magBufferIndex].calibrated = false;  // Mark as uncalibrated

  // Increment buffer index
  magBufferIndex = (magBufferIndex + 1) % MAG_BUFFER_SIZE;
  if (magBufferCount < MAG_BUFFER_SIZE) {
    magBufferCount++;
  }

  // Clear interrupt and set flag
  mag.clearMeasDoneInterrupt();
  newMagDataAvailable = true;
}

void setRGB(uint16_t r, uint16_t g, uint16_t b, uint16_t w = 0) {
  redState = r;
  greenState = g;
  blueState = b;
  whiteState = w;
}

void setThrottle(float throttle1, float throttle2) {
  if (throttle1 == 0 && throttle2 == 0) {
    lastMotor1 = -1;
    lastMotor2 = -1;
  }
  motor1Throttle = throttleToPWM(throttle1 * MOTOR1_DIRECTION);
  motor2Throttle = throttleToPWM(throttle2 * MOTOR2_DIRECTION);
  pwmIntCalled = false;
}

void updateLEDs() {
  if (!ledInterruptsEnabled) return;  // quick bypass option
  static uint8_t i = 0;
  rbit = (redState >> i) & 0x1;
  gbit = (greenState >> i) & 0x1;
  bbit = (blueState >> i) & 0x1;
  volatile uint32_t sumbits = rbit * RED + gbit * GREEN + bbit * BLUE;

  leds.clear();  // do we have to clear?
  leds.setPixel(0, sumbits);
  leds.setPixel(1, sumbits);
  //leds.setPixel(3, rbit * RED + gbit * GREEN + bbit * BLUE);
  //leds.setPixel(5, rbit * RED + gbit * GREEN + bbit * BLUE);
  leds.show();
  i = (i + 1) & 0xF;
}

float normalize(float phase, float minVal = -0.5, float maxVal = 0.5) {
  // Normalize to 0-1 range first
  phase = fmod(phase, 1.0);
  if (phase < 0) phase += 1.0;
  
  // Then shift to desired range
  if (minVal != 0 || maxVal != 1.0) {
    if (phase > maxVal && phase <= 1.0) phase -= 1.0;
    if (phase < minVal) phase += 1.0;
  }
  
  return phase;
}

int throttleToPWM(float throttle) {
  throttle = constrain(throttle, -1.0, 1.0);
  return map(throttle, -1, 1, PWM_1000us, PWM_2000us);
}

float modulateThrottle(float inputThrottle, int idx) {
  unsigned long now = millis();
  if (now > endBoost[idx] + 100) {
    endBoost[idx] = 0;
    return inputThrottle;
  }
  if (now > endBoost[idx] || inputThrottle > boostThreshold) {
    isBoost[idx] = false;
    return inputThrottle;
  }
  if (endBoost[idx] == 0) {
    isBoost[idx] = true;
    endBoost[idx] = now + baseCycleTime;
  }
  if (!isBoost[idx]) return inputThrottle;
  return boostSpeed;
}

void TankDrive() {
  float x = stickHoriz / 6.0;  // Reduced for less twitchy steering
  float y = stickVert / 5.0;
  float m1 = y + x;  // Left motor
  float m2 = y - x;  // Right motor

  m1 = modulateThrottle(m1, 0);
  m2 = modulateThrottle(m2, 1);

  pwmIntCalled = false;
  setThrottle(m1, m2);
}

bool rc_signal_is_healthy() {
  uint32_t now = millis();
  bool res = false;
  if (now - last_ibus_seen_millis < 500) {
    res = true;
  } else {
    ibus.loop();
  }
  if (last_cnt_rec != ibus.cnt_rec || last_cnt_poll != ibus.cnt_poll) {
    res = true;
    last_cnt_rec = ibus.cnt_rec;
    last_cnt_poll = ibus.cnt_poll;
    last_ibus_seen_millis = now;
  }
  return res;
}

void updateInputs() {
  int horiz = ibus.readChannel(0);
  int vert = ibus.readChannel(1);

  stickVert = (vert - 1500) / 500.0;
  stickHoriz = (horiz - 1500) / 500.0;
  stickAngle = atan2(stickHoriz, stickVert) / (2 * PI);
  stickLength = sqrt(sq(stickVert) + sq(stickHoriz));
  stickLength = min(stickLength, 1.0);
  if (stickLength < 0.05) {
    stickAngle = 0;
    stickLength = 0;
  }

  throttle = (ibus.readChannel(2) - 1000) / 1000.0;
  rudderInput = ibus.readChannel(3);
  inputPot = ibus.readChannel(4);
  inputToggle = ibus.readChannel(6);

  if (inputToggle > 1500) { kalmanInput = inputPot; }
  if (inputToggle == 1500) {}
  if (inputToggle < 1500) { radiusInput = inputPot; }

  rudderInput = map(rudderInput, 1000, 2000, -100, 100);
  radiusSize = map(radiusInput + rudderInput, 1000, 2000, 0.001, 0.050);

  kalmanQ = constrain(kalmanInput, 1000, 2000);
  kalmanQ = map(kalmanQ, 1000, 2000, 0, 3);
  kalmanQ = 0.5 + (1.0 / pow(10, kalmanQ));
  kalmanFilter.setProcessNoise(kalmanQ);

  ledOffset = (ibus.readChannel(5) - 1500) / 1000.0;  //ledOffset
  sidewaysInput = (ibus.readChannel(7) - 1500) / 500.0;
}

float getHeading() {
  uint32_t magX = 131072;
  uint32_t magY = 131072;
  uint32_t magZ = 131072;
  mag.getMeasurementXYZ(&magX, &magY, &magZ);
  headingMark = micros();  // record time of measurement

  // Normalize readings
  mag_x = (magX - magOffsetX) / 131072.0;
  mag_y = (magY - magOffsetY) / 131072.0;
  mag_z = (magZ - magOffsetZ) / 131072.0;

  float heading_raw = atan2(mag_y, mag_x) / (2 * PI);

  // Normalize heading to -0.5 to 0.5 range
  while (heading_raw < -0.5) heading_raw += 1.0;
  while (heading_raw > 0.5) heading_raw -= 1.0;

  return heading_raw;
}

float calculateW() {
  // Get accelerometer data
  sensors_event_t s;
  accel.getEvent(&s);

  // 1. Validate raw Z-axis acceleration
  if (isnan(s.acceleration.z) || isinf(s.acceleration.z)) {
    DEBUG_PRINTF(" Bad Z-accel: %.2f (X=%.2f, Y=%.2f)\n", s.acceleration.z, s.acceleration.x, s.acceleration.y);
    return RPS_THRESHOLD * 2 * PI;  // Return safe value in radians/sec
  }

  // 2. Apply calibration offsets
  float z = s.acceleration.z - accelOffsetZ;

  // 3. Update Kalman filter with calibrated value
  float spinAccel = kalmanFilter.updateEstimate(z);
  estimated_accel = spinAccel; // Store for telemetry

  // 4. Check accelerometer data after filtering
  if ((spinAccel <= 0) || isnan(spinAccel) || isinf(spinAccel)) {
    DEBUG_PRINTF(" Invalid accel data: %1.2f\n", spinAccel);
    return RPS_THRESHOLD * 2 * PI;  // Fallback to safe value in radians/sec
  }

  // 5. Check radius size
  if (radiusSize <= 0 || isnan(radiusSize) || isinf(radiusSize)) {
    DEBUG_PRINTF(" Invalid radius: %.4f\n", radiusSize);
    return RPS_THRESHOLD * 2 * PI;  // Fallback to safe value in radians/sec
  }

  // 6. Calculate w (angular velocity in radians/sec)
  float w = sqrt(spinAccel / radiusSize);
  
  // 7. Calculate RPS for compatibility with existing code
  float rps = w / (2 * PI);
  
  // 8. Update maxRPS if valid
  if (rps > maxRPS) {
    maxRPS = rps;
  }
  
  lastRPS = rps; // Store for telemetry and other uses

  // 9. Ensure w does not fall below threshold
  if (w < RPS_THRESHOLD * 2 * PI) {
    w = RPS_THRESHOLD * 2 * PI;
  }

  return w;
}

void updatePhaseTracking(float w) {
  // Get current time
  unsigned long now = micros();
  
  // Calculate time since last update
  unsigned long deltaTime = now - lastPhaseUpdate;
  
  // Skip if this is the first call or after a long delay
  if (lastPhaseUpdate == 0 || deltaTime > 100000) { // 100ms
    lastPhaseUpdate = now;
    previousW = w;
    return;
  }
  
  // Trapezoidal integration: use average of current and previous angular velocity
  float deltaAngle = (w + previousW) * 0.5f * deltaTime * 0.000001f;
  
  // Update continuous phase
  continuousPhase += deltaAngle / (2 * PI);
  
  // Keep phase in 0.0-1.0 range
  continuousPhase = normalize(continuousPhase, 0, 1.0);
  
  // Store for next iteration
  previousW = w;
  lastPhaseUpdate = now;
}

void MeltybrainDrive1() {
  // Calculate initial angular velocity (w)
  float w = calculateW();
  
  heading = getHeading();  // Basic magnetometer heading for logging comparison only
  
  // Initialize phase tracking
  unsigned long usCurrentTime = micros();
  lastPhaseUpdate = usCurrentTime;
  continuousPhase = 0.0f; // Reset phase at the start
  previousW = w; // Initial angular velocity
  
  ledInterruptsEnabled = false;  // Bypass LED interrupt action
  
  // Track hot loop iterations for diagnostic purposes
  int32_t hotLoopCount = 0;
  
  // HOT LOOP
  while (true) {
    // Get current time
    unsigned long currentTimeMicros = micros();
    
    // Exit conditions
    if (throttle < ZERO_THROTTLE_THRESHOLD || (currentTimeMicros - usCurrentTime) > 2000000) {
      DEBUG_PRINTLN(" Throttle zero or Timeout");
      break;
    }
    
    // Update angular velocity and phase tracking
    float currentW = calculateW();
    updatePhaseTracking(currentW);
    
    hotLoopCount++;  // Count iterations of the hot loop
    
    // Reverse input angle for correct steering
    stickAngle = -stickAngle;
    
    // Use continuousPhase instead of time-based phase
    float targetPhase = normalize(stickAngle, 0, 1.0);
    float backwardPhase = normalize(targetPhase + 0.5, 0, 1.0);
    float ledPhase = normalize(targetPhase + ledOffset, 0, 1.0);
    
    // Calculate phase differences (how far are we from target positions)
    float forwardDiff = normalize(targetPhase - continuousPhase);
    float backwardDiff = normalize(backwardPhase - continuousPhase);
    float ledDiff = normalize(ledPhase - continuousPhase);
    
    // Calculate motor and LED outputs directly from phase differences
    float cos_ph1 = cos(forwardDiff * 2 * PI);
    float cos_ph2 = cos(backwardDiff * 2 * PI);
    float cos_led = cos(ledDiff * 2 * PI);
    
    float widthScale = max(stickLength, throttle);
    float th1 = max(0, (cos_ph1 * 0.25f * stickLength) + throttle);
    float th2 = max(0, (cos_ph2 * 0.25f * stickLength) + throttle);
    
    setThrottle(th1, -th2);  // -th2 because of CW spin direction
    
    bool LEDOn = cos_led > 0.7071 * (1.4 - widthScale * 0.9);  // Magic numbers for ~45deg arc
    uint32_t COLOR = LEDOn * WHITE;
    float currentRPS = currentW / (2 * PI);
    if (currentRPS <= RPS_THRESHOLD) { COLOR = LEDOn * WHITE + LEDOn * GREEN; }  // Add green if below threshold
    leds.setPixel(0, COLOR);
    leds.setPixel(1, COLOR);
    leds.show();
    
    // Capture telemetry data every few iterations
    if (hotLoopCount % 4 == 0) {
      captureTelemetryData(
        heading,               // fl1: magnetometer heading (0-360)
        continuousPhase,       // fl2: current phase from accelerometer (0-1)
        targetPhase,           // fl3: target phase from stick input
        ledPhase,              // fl4: LED target phase
        forwardDiff,           // fl5: phase difference for forward direction
        backwardDiff,          // fl6: phase difference for backward direction
        ledDiff,               // fl7: phase difference for LED
        cos_ph1,               // fl8: cosine value for forward motor
        cos_ph2,               // fl9: cosine value for backward motor
        cos_led,               // fl10: cosine value for LED
        hotLoopCount,          // int1: iteration count in hot loop
        LEDOn ? 1 : 0,         // int2: LED state (1=on, 0=off)
        (int32_t)(throttle * 1000)  // int3: throttle value (scaled by 1000)
      );
    }
    
    // Implement a small delay to prevent excessive looping
    delayMicroseconds(LOOP_DELAY_MICROS);
  }
  
  ledInterruptsEnabled = true;  // Resume LED interrupt action
}

void loop() {
  ibus.loop();

  while (!rc_signal_is_healthy()) {
    setRGB(LED_PATTERN_RADIO_ERROR);
    ibus.loop();
    setThrottle(0, 0);
    delay(5);
  }

  updateInputs();

  sendMagDataOverBT();    // Controlled by channel 9
  sendTelemetryOverBT();  // Controlled by channel 8

  bool meltyMode = (throttle > ZERO_THROTTLE_THRESHOLD);
  bool tankMode = (throttle < ZERO_THROTTLE_THRESHOLD);
  // telemode = (toggleRight3way < 0);  // Use toggleRight3way to enable/disable telemode

  if (tankMode) {  // Tank mode
    setRGB(LED_PATTERN_CYCLE);
    TankDrive();
  } else {  // Melty mode
    setRGB(LED_PATTERN_OFF);
    MeltybrainDrive1();
  }
}