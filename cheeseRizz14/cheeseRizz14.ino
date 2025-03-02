//Complified Creations meltybrain controller
// based on Melontype

#include <IBusBM.h>             //pin15 Serial3 RX
#include "Adafruit_H3LIS331.h"  //Wire on pin18,pin19
#include <WS2812Serial.h>       //pin17
#include <SimpleKalmanFilter.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include "header.h"

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

void setRGB(uint16_t r, uint16_t g, uint16_t b, uint16_t w = 0) {
  redState = r;
  greenState = g;
  blueState = b;
  whiteState = w;
}

void initLEDs() {
  Serial.print("LEDs ");
  ledTimer.begin(updateLEDs, 50000);  // 50ms interval for LED updates
  leds.begin();
  leds.show();
  leds.setBrightness(BRIGHTNESS);
  Serial.println("initialized.");
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

void initESCs() {
  Serial.print("ESCs ");
  analogWriteResolution(16);
  analogWriteFrequency(PIN_ESC1, ESC_SERVO_RATE);
  analogWriteFrequency(PIN_ESC2, ESC_SERVO_RATE);
  pwmTimer.begin(pwmInterruptHandler, 2500);  // 400Hz PWM update rate
  setThrottle(0, 0);
  Serial.println("initialized, throttle set to 0.");
}

void initAccel() {
  Serial.print("Accelerometer ");
  if (!accel.begin_I2C(I2C_ADDR_ACCEL)) {
    Serial.println("Failed to initialize accelerometer! System halt.");
    setRGB(LED_PATTERN_ERROR);
    while (1)
      ;
  }
  Wire.setClock(400000);  // Set I2C to fast mode (400 kHz)
  accel.setDataRate(LIS331_DATARATE_400_HZ);
  accel.setRange(H3LIS331_RANGE_400_G);
  accel.setLPFCutoff(LIS331_LPF_292_HZ);
  Serial.print(" configured 400hz, 400g, lpf292? ");
  accelCalibration();
  Serial.println("initialized and calibrated.");
}

void accelCalibration() {
  Serial.print(". calibration... ");
  setRGB(LED_PATTERN_CALIBRATION);

  int sampleCount = 150;
  float xSum = 0, ySum = 0, zSum = 0;

  for (int i = 0; i < sampleCount; i++) {
    sensors_event_t s;
    accel.getEvent(&s);
    xSum += s.acceleration.x;
    ySum += s.acceleration.y;
    zSum += s.acceleration.z;
    delayMicroseconds(2500);
  }

  accelOffsetX = xSum / sampleCount;
  accelOffsetY = ySum / sampleCount;
  accelOffsetZ = zSum / sampleCount;

  setRGB(LED_PATTERN_GREEN_F);
  Serial.printf(" complete. \n Offsets %.3f %.3f %.3f stored.\n", accelOffsetX, accelOffsetY, accelOffsetZ);
}

void initMag() {
  // Note: Wire.begin() is already called in initAccel()

  // Configure the interrupt pin for the "Measurement Done" interrupt
  pinMode(PIN_MAG_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_MAG_INT), magInterrupt, RISING);

  if (!mag.begin()) {
    Serial.println("MMC5983MA did not respond - check wiring");
    while (true)
      ;
  }

  mag.softReset();
  Serial.println("MMC5983MA connected");

  // Configure magnetometer for maximum performance
  mag.setFilterBandwidth(800);           // 800Hz bandwidth for fast response
  mag.setContinuousModeFrequency(1000);  // 1000Hz continuous measurements
  mag.enableAutomaticSetReset();         // Enable auto set/reset for calibration

  // Calibration routine - collect data for magnetic offsets
  Serial.println("Starting magnetometer calibration...");

  // Initialize min and max values for calibration
  int32_t calXMin = 262144, calXMax = 0;
  int32_t calYMin = 262144, calYMax = 0;
  int32_t calZMin = 262144, calZMax = 0;

  // Turn off continuous mode during calibration
  mag.disableContinuousMode();

  // Collect calibration data over multiple measurements
  const int calSamples = 50;
  uint32_t rawX, rawY, rawZ;  // Declare variables before use

  for (int i = 0; i < calSamples; i++) {
    // Trigger a measurement manually
    mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);

    // Update min/max values
    calXMin = min(calXMin, (int32_t)rawX);
    calXMax = max(calXMax, (int32_t)rawX);
    calYMin = min(calYMin, (int32_t)rawY);
    calYMax = max(calYMax, (int32_t)rawY);
    calZMin = min(calZMin, (int32_t)rawZ);
    calZMax = max(calZMax, (int32_t)rawZ);

    // Display progress
    if (i % 10 == 0) {
      Serial.print("Calibrating: ");
      Serial.print(i * 100 / calSamples);
      Serial.println("%");
    }

    // Add a small delay
    delay(100);
  }

  // Calculate offsets (midpoint of min and max)
  magXOffset = (calXMin + calXMax) / 2;
  magYOffset = (calYMin + calYMax) / 2;
  magZOffset = (calZMin + calZMax) / 2;

  // Calculate scale factors to normalize X and Y ranges
  int32_t xRange = calXMax - calXMin;
  int32_t yRange = calYMax - calYMin;

  // Use the average range to normalize both axes
  int32_t avgRange = (xRange + yRange) / 2;

  // Set scale factors (if ranges are valid)
  if (xRange > 1000 && yRange > 1000) {
    magXScale = (float)avgRange / xRange;
    magYScale = (float)avgRange / yRange;
  } else {
    // Default to 1.0 if calibration data is insufficient
    magXScale = 1.0;
    magYScale = 1.0;
    Serial.println("Warning: Insufficient calibration data range");
  }

  // Log calibration results
  Serial.println("Magnetometer calibration complete");
  Serial.print("X Offset: ");
  Serial.print(magXOffset);
  Serial.print(", Y Offset: ");
  Serial.println(magYOffset);
  Serial.print("X Scale: ");
  Serial.print(magXScale, 4);
  Serial.print(", Y Scale: ");
  Serial.println(magYScale, 4);

  // Re-enable continuous mode and interrupts
  mag.enableContinuousMode();
  mag.enableInterrupt();

  // Clear any pending interrupts
  mag.clearMeasDoneInterrupt();
  newMagDataAvailable = true;

  // Initialize magnetometer buffer
  for (int i = 0; i < MAG_BUFFER_SIZE; i++) {
    magBuffer[i].rawX = 0;
    magBuffer[i].rawY = 0;
    magBuffer[i].rawZ = 0;
    magBuffer[i].timestamp = 0;
  }
}

void initIBus() {
  Serial.print("Radio ");
  ibus.begin(SERIAL_IBUS, IBUSBM_NOTIMER);
  ibus.loop();

  Serial.println("waiting for signal");
  while (!ibus.readChannel(0)) {
    setRGB(LED_PATTERN_RADIO_ERROR);
    ibus.loop();
    Serial.print(".");
    BT_PRINT("r");
    delay(500);
  }
  Serial.println("Remote signal received.");

  int throttle = ibus.readChannel(2);
  while (throttle > 1050) {
    setRGB(LED_PATTERN_THROTTLE_ERROR);
    Serial.println(throttle);
    Serial.println("Please lower throttle to zero.");
    throttle = ibus.readChannel(2);
    delay(500);
    ibus.loop();
  }
  setRGB(LED_PATTERN_GREEN_F);
  Serial.println("Throttle confirmed down.");
  Serial.println("Radio initialized.");
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

void processMagnetometerData() {
  if (newMagDataAvailable) {
    newMagDataAvailable = false;

    // Process at a lower frequency (200Hz)
    static uint32_t lastProcessTime = 0;
    uint32_t currentTime = millis();

    if (currentTime - lastProcessTime >= 5) {
      lastProcessTime = currentTime;

      // Calibrate any uncalibrated data in-place
      for (int i = 0; i < magBufferCount; i++) {
        uint8_t idx = (magBufferIndex + MAG_BUFFER_SIZE - i - 1) % MAG_BUFFER_SIZE;

        if (!magBuffer[idx].calibrated) {
          // Apply calibration to raw values
          magBuffer[idx].rawX = ((int32_t)magBuffer[idx].rawX - magXOffset) * magXScale;
          magBuffer[idx].rawY = ((int32_t)magBuffer[idx].rawY - magYOffset) * magYScale;
          magBuffer[idx].calibrated = true;
        }
      }

      // Check for peaks and valleys if we have enough data
      if (magBufferCount >= MAG_BUFFER_SIZE / 2) {
        updateHeading();
      }
    }
  }
}

void updateHeading() {
  // Only update peaks/valleys every 10ms (100Hz) to avoid excessive processing
  uint32_t currentTime = millis();
  if (currentTime - lastPeakValleyDetectionTime < 10) {
    return;
  }
  lastPeakValleyDetectionTime = currentTime;

  // Reset min/max values for each detection cycle
  xMax = 0;
  xMin = 262144;
  yMax = 0;
  yMin = 262144;

  // Calculate the signal noise floor to adapt to different environments
  uint32_t xRange = 0;
  uint32_t yRange = 0;

  // Find the min/max values in the buffer
  for (int i = 0; i < magBufferCount; i++) {
    uint8_t idx = (magBufferIndex + MAG_BUFFER_SIZE - i - 1) % MAG_BUFFER_SIZE;

    // Check X min/max
    if (magBuffer[idx].rawX > xMax) {
      xMax = magBuffer[idx].rawX;
      xMaxTime = magBuffer[idx].timestamp;
    }
    if (magBuffer[idx].rawX < xMin) {
      xMin = magBuffer[idx].rawX;
      xMinTime = magBuffer[idx].timestamp;
    }

    // Check Y min/max
    if (magBuffer[idx].rawY > yMax) {
      yMax = magBuffer[idx].rawY;
      yMaxTime = magBuffer[idx].timestamp;
    }
    if (magBuffer[idx].rawY < yMin) {
      yMin = magBuffer[idx].rawY;
      yMinTime = magBuffer[idx].timestamp;
    }
  }

  // Calculate signal ranges
  xRange = xMax - xMin;
  yRange = yMax - yMin;

  // Adaptive threshold based on actual signal strength
  uint32_t adaptiveThreshold = min(xRange, yRange) / 10;
  uint32_t minPeakValleyDiff = max(MIN_PEAK_VALLEY_DIFF, adaptiveThreshold);

  // Verify we have valid peak/valley data with sufficient signal strength
  bool validPeaks = (xRange > minPeakValleyDiff) && (yRange > minPeakValleyDiff);

  if (validPeaks) {
    // Calculate new heading
    float newHeading = calculateHeadingFromPeaks();

    // Apply smoothing to the heading
    if (headingInitialized) {
      // Smooth the heading with the previous value
      float headingDiff = newHeading - robotHeading;

      // Handle wrapping around 0/360 degrees
      if (headingDiff > 180) headingDiff -= 360;
      if (headingDiff < -180) headingDiff += 360;

      // Apply adaptive smoothing based on rotation speed
      float smoothingFactor = HEADING_SMOOTHING;
      if (lastRPS > 30) {         // At high speeds >1800 RPM
        smoothingFactor = 0.7;    // Less smoothing to track faster changes
      } else if (lastRPS < 10) {  // At low speeds <600 RPM
        smoothingFactor = 0.9;    // More smoothing at lower speeds
      }

      robotHeading += headingDiff * (1.0 - smoothingFactor);

      // Keep heading in 0-360 range
      if (robotHeading < 0) robotHeading += 360;
      if (robotHeading >= 360) robotHeading -= 360;
    } else {
      // First valid heading
      robotHeading = newHeading;
      headingInitialized = true;
    }

    lastHeading = robotHeading;

    // Update RPM calculation based on peak-to-peak timing
    float currentRPM = getRPM();
    if (currentRPM > 0) {
      // Simple exponential smoothing for RPM
      RPMestimate = RPMestimate * 0.7 + currentRPM * 0.3;
    }
  }
}

float calculateHeadingFromPeaks() {
  // Calculate approximate rotation period in microseconds
  uint32_t xHalfPeriod = abs((int32_t)(xMaxTime - xMinTime));
  uint32_t yHalfPeriod = abs((int32_t)(yMaxTime - yMinTime));

  // Check for valid half periods (prevent division by zero)
  if (xHalfPeriod < 100 || yHalfPeriod < 100) {
    // If timing data is invalid, return the last heading
    return robotHeading;
  }

  // Estimate full rotation period (in microseconds)
  uint32_t rotationPeriod = (xHalfPeriod + yHalfPeriod) * 2;  // Full period

  // Current time for reference
  uint32_t currentTime = micros();

  // Create array of peak/valley times and their expected angles
  uint32_t referencePoints[4] = { xMaxTime, yMaxTime, xMinTime, yMinTime };
  float referenceAngles[4] = { 0.0, 90.0, 180.0, 270.0 };  // X-max, Y-max, X-min, Y-min

  // Check which points are valid (non-zero and recent enough)
  bool validPoint[4] = { false, false, false, false };
  uint8_t validCount = 0;
  const uint32_t maxTimeDiff = rotationPeriod * 2;  // Don't use points older than 2 rotations

  for (int i = 0; i < 4; i++) {
    if (referencePoints[i] > 0 && (currentTime - referencePoints[i]) < maxTimeDiff) {
      validPoint[i] = true;
      validCount++;
    }
  }

  // Need at least two valid points for calculation
  if (validCount < 2) {
    return robotHeading;  // Can't calculate with fewer than 2 points
  }

  // Normalize all reference points to what time it would have been at 0 degrees
  uint32_t zeroTimeEstimates[4] = { 0, 0, 0, 0 };
  float weights[4] = { 0.0, 0.0, 0.0, 0.0 };
  float totalWeight = 0.0;

  for (int i = 0; i < 4; i++) {
    if (validPoint[i]) {
      // Calculate what time it would have been at 0 degrees
      // This formula adjusts the timestamp based on the expected angle
      // (referenceAngles[i] / 360.0) gives us the fraction of a full rotation
      uint32_t angleOffset = (uint32_t)(rotationPeriod * (referenceAngles[i] / 360.0));

      // If the point is at 0 degrees, don't adjust
      // Otherwise, subtract the time it would take to get to that angle from 0
      if (angleOffset == 0) {
        zeroTimeEstimates[i] = referencePoints[i];
      } else {
        // Adjust for wraparound - if point is in the past
        if (angleOffset > referencePoints[i]) {
          zeroTimeEstimates[i] = referencePoints[i] + rotationPeriod - angleOffset;
        } else {
          zeroTimeEstimates[i] = referencePoints[i] - angleOffset;
        }
      }

      // Weight based on recency (newer points weighted higher)
      // Exponential decay based on age of measurement
      float age = (float)(currentTime - referencePoints[i]) / (float)rotationPeriod;
      weights[i] = exp(-age * 0.5);  // Exponential decay factor can be tuned

      totalWeight += weights[i];
    }
  }

  // Weighted average of all zero time estimates
  uint32_t weightedZeroTime = 0;
  for (int i = 0; i < 4; i++) {
    if (validPoint[i]) {
      weightedZeroTime += (uint32_t)((float)zeroTimeEstimates[i] * (weights[i] / totalWeight));
    }
  }

  // Calculate how many degrees we've gone since the estimated zero time
  float angleSinceZero = 0.0;
  if (weightedZeroTime <= currentTime) {
    // Normal case - zero reference time is in the past
    angleSinceZero = ((float)(currentTime - weightedZeroTime) / (float)rotationPeriod) * 360.0;
  } else {
    // Handle wraparound case - zero reference time appears to be in the future
    // This can happen due to micros() overflow or calculation quirks
    angleSinceZero = ((float)(currentTime + UINT32_MAX - weightedZeroTime) / (float)rotationPeriod) * 360.0;
    // Bound the result to avoid extreme values from this edge case
    if (angleSinceZero > 360.0) {
      angleSinceZero = fmod(angleSinceZero, 360.0);
    }
  }

  // Keep heading in 0-360 range
  while (angleSinceZero >= 360.0) angleSinceZero -= 360.0;
  while (angleSinceZero < 0.0) angleSinceZero += 360.0;

  return angleSinceZero;
}

float getRPM() {  //mag
  uint32_t xPeriod = abs((int32_t)(xMaxTime - xMinTime)) * 2;
  uint32_t yPeriod = abs((int32_t)(yMaxTime - yMinTime)) * 2;

  // Average the two period estimates in microseconds
  uint32_t rotationPeriod = (xPeriod + yPeriod) / 2;

  if (rotationPeriod > 0) {
    // Convert microseconds per rotation to RPM
    return 60000000.0 / rotationPeriod;
  }

  return 0.0;
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

float getSpinAcceleration() {
  sensors_event_t s;
  accel.getEvent(&s);

  // 1. Validate raw Z-axis acceleration
  if (isnan(s.acceleration.z) || isinf(s.acceleration.z)) {
    DEBUG_PRINTF(" Bad Z-accel: %.2f (X=%.2f, Y=%.2f)\n", s.acceleration.z, s.acceleration.x, s.acceleration.y);
    return estimated_accel;  // Return last valid estimate
  }

  // 2. Apply calibration offsets
  float z = s.acceleration.z - accelOffsetZ;

  // redundant// 3. Validate calibrated Z-axis acceleration
  // if (isnan(z) || isinf(z)) {
  //   DEBUG_PRINTF(" Bad calibrated Z: %.2f\n", z);
  //   return estimated_accel;  // Skip Kalman update
  // }

  estimated_accel = kalmanFilter.updateEstimate(z);
  return estimated_accel;
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

float calculateRPS() {  // with error checks
  float spinAccel = getSpinAcceleration();
  // 1. Check accelerometer data
  if ((spinAccel <= 0) || isnan(spinAccel) || isinf(spinAccel)) {
    DEBUG_PRINTF(" Invalid accel data: %1.2f\n", spinAccel);
    return RPS_THRESHOLD;  // Fallback to safe value
  }

  // 2. Check radius size
  if (radiusSize <= 0 || isnan(radiusSize) || isinf(radiusSize)) {
    DEBUG_PRINTF(" Invalid radius: %.4f\n", radiusSize);
    return RPS_THRESHOLD;  // Fallback to safe value
  }

  // 3. Calculate omega (angular velocity)
  float omega = sqrt(spinAccel / radiusSize);
  float rps = omega / (2 * PI);
  if (isnan(rps) || isinf(rps)) {
    DEBUG_PRINTF(" Invalid RPS: %.2f (omega: %.2f)\n", rps, omega);
    return RPS_THRESHOLD;  // Fallback to safe value
  }

  // 6. Update maxRPS if valid
  if (rps > maxRPS && !isnan(rps) && !isinf(rps)) {
    maxRPS = rps;
  }

  // 7. Ensure RPS does not fall below threshold
  if (rps < RPS_THRESHOLD) {
    rps = RPS_THRESHOLD;
  }

  return rps;
}

void captureTelemetryData(
  float f1, float f2, float f3, float f4, float f5,
  float f6, float f7, float f8, float f9, float f10,
  int32_t i1, int32_t i2, int32_t i3) {

  // Store current telemetry data
  telemetryBuffer[telemetryBufferIndex].timestamp = micros();
  telemetryBuffer[telemetryBufferIndex].fl1 = f1;
  telemetryBuffer[telemetryBufferIndex].fl2 = f2;
  telemetryBuffer[telemetryBufferIndex].fl3 = f3;
  telemetryBuffer[telemetryBufferIndex].fl4 = f4;
  telemetryBuffer[telemetryBufferIndex].fl5 = f5;
  telemetryBuffer[telemetryBufferIndex].fl6 = f6;
  telemetryBuffer[telemetryBufferIndex].fl7 = f7;
  telemetryBuffer[telemetryBufferIndex].fl8 = f8;
  telemetryBuffer[telemetryBufferIndex].fl9 = f9;
  telemetryBuffer[telemetryBufferIndex].fl10 = f10;
  telemetryBuffer[telemetryBufferIndex].int1 = i1;
  telemetryBuffer[telemetryBufferIndex].int2 = i2;
  telemetryBuffer[telemetryBufferIndex].int3 = i3;

  // Update buffer indices
  telemetryBufferIndex = (telemetryBufferIndex + 1) % TELEMETRY_BUFFER_SIZE;
  if (telemetryBufferCount < TELEMETRY_BUFFER_SIZE) {
    telemetryBufferCount++;
  }
}

void sendMagDataOverBT() {
  static bool transmissionActive = false;
  static uint16_t transmissionIndex = 0;
  static uint8_t entriesPerIteration = 10;
  static uint32_t lastTransmissionTime = 0;

  // Read channel 9 to check if magnetometer telemetry is requested
  int telemetryTrigger = ibus.readChannel(9);

  // Start transmission when trigger is activated
  if (telemetryTrigger > 1300 && !transmissionActive && (millis() - lastTransmissionTime > 1000)) {
    transmissionActive = true;
    transmissionIndex = 0;

    // Calculate starting index (oldest data point in the circular buffer)
    uint16_t startIndex;
    if (magBufferCount < MAG_BUFFER_SIZE) {
      // Buffer not full yet, start from beginning
      startIndex = 0;
    } else {
      // Buffer is full, start from oldest entry
      startIndex = (magBufferIndex) % MAG_BUFFER_SIZE;
    }

    // Send header
    Serial1.printf("START_MAG,%lu,%u,%u\n", millis(), magBufferCount, MAG_BUFFER_SIZE);
    Serial1.println("FORMAT,timestamp,magX,magY,magZ,z_accel,rpm");

    // Reset transmission index to start position
    transmissionIndex = startIndex;

    // Record last transmission time
    lastTransmissionTime = millis();
  }

  // If transmission is active, send a batch of entries
  if (transmissionActive) {
    uint16_t entriesSent = 0;

    while (entriesSent < entriesPerIteration && entriesSent < magBufferCount) {
      // Calculate current buffer index to send
      uint16_t currentIndex = (transmissionIndex + entriesSent) % MAG_BUFFER_SIZE;

      // Send magnetometer data
      Serial1.printf("MAG,%lu,%lu,%lu,%lu,%.2f,%.2f\n",
                     magBuffer[currentIndex].timestamp,
                     magBuffer[currentIndex].rawX,
                     magBuffer[currentIndex].rawY,
                     magBuffer[currentIndex].rawZ,
                     estimated_accel,
                     RPMestimate);

      entriesSent++;

      // Check if we've sent all entries
      if ((transmissionIndex + entriesSent) % MAG_BUFFER_SIZE == magBufferIndex) {
        // We've reached the newest data point
        uint32_t endTime = millis();
        Serial1.printf("END_MAG,%lu\n", endTime);
        transmissionActive = false;
        break;
      }
    }

    // Update transmission index for next iteration
    transmissionIndex = (transmissionIndex + entriesSent) % MAG_BUFFER_SIZE;

    // If we've gone full circle, end transmission
    if (transmissionIndex == magBufferIndex) {
      uint32_t endTime = millis();
      Serial1.printf("END_MAG,%lu\n", endTime);
      transmissionActive = false;
    }
  }
}

void sendTelemetryOverBT() {
  static bool transmissionActive = false;
  static uint16_t transmissionIndex = 0;
  static uint8_t entriesPerIteration = 10;
  static uint32_t lastTransmissionTime = 0;

  // Read channel 8 to check if telemetry is requested
  int telemetryTrigger = ibus.readChannel(8);

  // Start transmission when trigger is activated
  if (telemetryTrigger > 1300 && !transmissionActive && (millis() - lastTransmissionTime > 1000)) {
    transmissionActive = true;
    transmissionIndex = 0;

    // Calculate starting index (oldest data point in the circular buffer)
    uint16_t startIndex;
    if (telemetryBufferCount < TELEMETRY_BUFFER_SIZE) {
      // Buffer not full yet, start from beginning
      startIndex = 0;
    } else {
      // Buffer is full, start from oldest entry
      startIndex = (telemetryBufferIndex) % TELEMETRY_BUFFER_SIZE;
    }

    // Send header with field labels
    Serial1.printf("START_TELEM,%lu,%u,%u\n", millis(), telemetryBufferCount, TELEMETRY_BUFFER_SIZE);

    // Send format line with field descriptions
    Serial1.print("FORMAT,timestamp");
    for (int i = 0; i < 10; i++) {
      Serial1.printf(",fl%d=%s", i + 1, telemetryLabels[i + 1]);
    }
    for (int i = 0; i < 3; i++) {
      Serial1.printf(",int%d=%s", i + 1, telemetryLabels[11 + i]);
    }
    Serial1.println();

    // Reset transmission index to start position
    transmissionIndex = startIndex;

    // Record last transmission time
    lastTransmissionTime = millis();
  }

  // If transmission is active, send a batch of entries
  if (transmissionActive) {
    uint16_t entriesSent = 0;

    while (entriesSent < entriesPerIteration && entriesSent < telemetryBufferCount) {
      // Calculate current buffer index to send
      uint16_t currentIndex = (transmissionIndex + entriesSent) % TELEMETRY_BUFFER_SIZE;

      // Get the current telemetry data entry
      TelemetryData* data = &telemetryBuffer[currentIndex];

      // Send telemetry data record
      Serial1.printf("TELEM,%lu", data->timestamp);

      // Send all float values
      Serial1.printf(",%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                     data->fl1, data->fl2, data->fl3, data->fl4, data->fl5,
                     data->fl6, data->fl7, data->fl8, data->fl9, data->fl10);

      // Send all integer values
      Serial1.printf(",%d,%d,%d",
                     data->int1, data->int2, data->int3);

      Serial1.println();

      entriesSent++;

      // Check if we've sent all entries
      if ((transmissionIndex + entriesSent) % TELEMETRY_BUFFER_SIZE == telemetryBufferIndex) {
        // We've reached the newest data point
        uint32_t endTime = millis();
        Serial1.printf("END_TELEM,%lu\n", endTime);
        transmissionActive = false;
        break;
      }
    }

    // Update transmission index for next iteration
    transmissionIndex = (transmissionIndex + entriesSent) % TELEMETRY_BUFFER_SIZE;

    // If we've gone full circle, end transmission
    if (transmissionIndex == telemetryBufferIndex) {
      uint32_t endTime = millis();
      Serial1.printf("END_TELEM,%lu\n", endTime);
      transmissionActive = false;
    }
  }
}

// void MeltybrainDrive2() {      // mag version
//   float rps = calculateRPS();  // from accel
//   lastRPS = rps;
//   unsigned long revTimeMicros = (1000000 / rps);
//   unsigned long usCurrentTime = micros();
//   usRevStartTime = usCurrentTime;

//   // For diagnostic capture
//   static uint32_t lastDiagCaptureTime = 0;
//   static const uint32_t diagCaptureInterval = 2000;  // Capture every 2ms

//   ledInterruptsEnabled = false;  // Bypass LED interrupt action
//   while (true) {                 // hot loop
//     unsigned long currentTimeMicros = micros() - usRevStartTime;
//     if (throttle < ZERO_THROTTLE_THRESHOLD || currentTimeMicros > 2000000) {  // 2sec timeout emergency exit
//       break;
//     }
//     if (currentTimeMicros >= revTimeMicros) {  // stop loop after 1 rev
//       break;
//     }

//     // Get the current heading from magnetometer (0-360°)
//     // Convert to phase angle (0-1 range)
//     float currentPhase = robotHeading / 360.0;

//     // Calculate target phase based on stick input
//     float targetPhase = stickAngle;
//     float backwardsPhase = (stickAngle + 0.5) > 1.0 ? (stickAngle + 0.5) - 1.0 : (stickAngle + 0.5);
//     float ledPhase = (stickAngle + ledOffset) > 1.0 ? (stickAngle + ledOffset) - 1.0 : (stickAngle + ledOffset);

//     // Calculate phase differences (normalized to -0.5 to 0.5 range)
//     float forwardDiff = targetPhase - currentPhase;
//     if (forwardDiff > 0.5) forwardDiff -= 1.0;
//     if (forwardDiff < -0.5) forwardDiff += 1.0;

//     float backwardDiff = backwardsPhase - currentPhase;
//     if (backwardDiff > 0.5) backwardDiff -= 1.0;
//     if (backwardDiff < -0.5) backwardDiff += 1.0;

//     float ledDiff = ledPhase - currentPhase;
//     if (ledDiff > 0.5) ledDiff -= 1.0;
//     if (ledDiff < -0.5) ledDiff += 1.0;

//     // Convert phase differences to angles for cosine calculation
//     float ph1 = forwardDiff * 2.0 * M_PI;
//     float ph2 = backwardDiff * 2.0 * M_PI;
//     float ledPh = ledDiff * 2.0 * M_PI;

//     float widthScale = max(stickLength, throttle);

//     float cos_ph1 = cos(ph1);
//     float cos_ph2 = cos(ph2);
//     float cos_led = cos(ledPh);

//     float th1 = max(0, (cos_ph1 * 0.25f * stickLength) + throttle);
//     float th2 = max(0, (cos_ph2 * 0.25f * stickLength) + throttle);

//     setThrottle(th1, -th2);  // -th2 because of CW spin direction

//     bool LEDOn = cos_led > 0.7071 * (1.4 - widthScale * 0.9);  // magic numbers to get about 45deg of arc
//     uint32_t COLOR = LEDOn * WHITE;
//     if (rps <= RPS_THRESHOLD) { COLOR = LEDOn * WHITE + LEDOn * GREEN; }  // add green if below threshold rps
//     leds.setPixel(0, COLOR);
//     leds.setPixel(1, COLOR);
//     leds.show();

//     // Diagnostic data capture at regular intervals
//     // This avoids capturing too much data but still gives us enough sampling points
//     uint32_t currentTime = micros();
//     if (currentTime - lastDiagCaptureTime >= diagCaptureInterval) {
//       lastDiagCaptureTime = currentTime;

//       // Capture diagnostic data
//       captureDiagnosticData(currentPhase, targetPhase, ledPhase,
//                             forwardDiff, backwardDiff, ledDiff,
//                             cos_ph1, cos_ph2, cos_led, LEDOn);
//     }
//   }
//   ledInterruptsEnabled = true;  // Resume LED interrupt action
// }

void MeltybrainDrive1() {  //accelerometer version
  float rps = calculateRPS();
  lastRPS = rps;
  heading = getHeading();
  unsigned long revTimeMicros = (1000000 / rps);
  unsigned long usCurrentTime = micros();
  usRevStartTime = usCurrentTime;
  float currentPhase = robotHeading / 360.0;  // from magnetometer
  ledInterruptsEnabled = false;               // Bypass LED interrupt action

  // Track hot loop iterations for diagnostic purposes
  int32_t hotLoopCount = 0;

  //HOT LOOP
  while (true) {
    unsigned long currentTimeMicros = micros() - usRevStartTime;

    if (throttle < ZERO_THROTTLE_THRESHOLD || currentTimeMicros > 2000000) {  // 2sec timeout emergency exit
      DEBUG_PRINTLN(" Throttle zero or Timeout");
      break;
    }
    if (currentTimeMicros >= revTimeMicros) {  // stop loop after 1 rev, no debug msg
      break;
    }

    hotLoopCount++;  // Count iterations of the hot loop

    float acc_ph = (float)currentTimeMicros / (float)revTimeMicros;  // phase from accel calc.
    stickAngle = -stickAngle;                                        // reverse for correct steering
    // Calculate target phase based on stick input
    float targetPhase = stickAngle;
    float backwardsPhase = (stickAngle + 0.5) > 1.0 ? (stickAngle + 0.5) - 1.0 : (stickAngle + 0.5);
    float ledPhase = (stickAngle + ledOffset) > 1.0 ? (stickAngle + ledOffset) - 1.0 : (stickAngle + ledOffset);
    // magnetometer demonstrationsection for comparison:
    // Calculate phase differences (normalized to -0.5 to 0.5 range)
    float forwardDiff = targetPhase - currentPhase;
    if (forwardDiff > 0.5) forwardDiff -= 1.0;
    if (forwardDiff < -0.5) forwardDiff += 1.0;

    float backwardDiff = backwardsPhase - currentPhase;
    if (backwardDiff > 0.5) backwardDiff -= 1.0;
    if (backwardDiff < -0.5) backwardDiff += 1.0;

    float ledDiff = ledPhase - currentPhase;
    if (ledDiff > 0.5) ledDiff -= 1.0;
    if (ledDiff < -0.5) ledDiff += 1.0;

    // Convert phase differences to angles for cosine calculation
    float mag_ph1 = forwardDiff * 2.0 * M_PI;
    float mag_ph2 = backwardDiff * 2.0 * M_PI;
    float mag_ledPh = ledDiff * 2.0 * M_PI;

    //accelerometer version, actuall doing the steering
    float timeToForward = stickAngle * revTimeMicros;
    float timeToBackward = (stickAngle + 0.5) * revTimeMicros;  // 1/2 revolution further along for backwards
    float timeLED = (stickAngle + ledOffset) * revTimeMicros;   // modified by right knob
    if (timeToBackward > revTimeMicros) {
      timeToBackward -= revTimeMicros;
    }
    if (timeLED > revTimeMicros) {
      timeLED -= revTimeMicros;
    }

    float widthScale = max(stickLength, throttle);

    float ph1 = ((currentTimeMicros + timeToForward) / revTimeMicros) * M_PI * 2.0f;
    float ph2 = ((currentTimeMicros + timeToBackward) / revTimeMicros) * M_PI * 2.0f;
    float ledPh = ((currentTimeMicros + timeLED) / revTimeMicros) * M_PI * 2.0f;
    float cos_ph1 = cos(ph1);
    float cos_ph2 = cos(ph2);
    float cos_led = cos(ledPh);
    float th1 = max(0, (cos_ph1 * 0.25f * stickLength) + throttle);
    float th2 = max(0, (cos_ph2 * 0.25f * stickLength) + throttle);

    setThrottle(th1, -th2);  // -th2 because of CW spin direction

    bool LEDOn = cos_led > 0.7071 * (1.4 - widthScale * 0.9);  // magic numbers to get about 45deg of arc
    uint32_t COLOR = LEDOn * WHITE;
    if (rps <= RPS_THRESHOLD) { COLOR = LEDOn * WHITE + LEDOn * GREEN; }  // add green if below threshold rps
    leds.setPixel(0, COLOR);
    leds.setPixel(1, COLOR);
    leds.show();

    // Capture telemetry data every few iterations
    // We're using modulo 4 to capture roughly every 4th iteration (25% sample rate)
    if (hotLoopCount % 4 == 0) {
      // Capture telemetry using our new generic function
      // Each parameter is explicitly named for clarity:
      captureTelemetryData(
        robotHeading,               // fl1: current heading in degrees (0-360)
        currentPhase,               // fl2: current phase (0-1 range)
        targetPhase,                // fl3: target phase from stick input
        ledPhase,                   // fl4: LED phase (adjusted by ledOffset)
        forwardDiff,                // fl5: phase difference for forward direction
        backwardDiff,               // fl6: phase difference for backward direction
        ledDiff,                    // fl7: phase difference for LED
        cos_ph1,                    // fl8: cosine value for forward motor
        cos_ph2,                    // fl9: cosine value for backward motor
        cos_led,                    // fl10: cosine value for LED
        hotLoopCount,               // int1: iteration count in hot loop
        LEDOn ? 1 : 0,              // int2: LED state (1=on, 0=off)
        (int32_t)(throttle * 1000)  // int3: throttle value (scaled by 1000)
      );
    }
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
  processMagnetometerData();

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

  // BT_PRINTF(" %f %f\n", lastRPS * 60.0, robotHeading);  // Updated to use robotHeading instead of heading
}