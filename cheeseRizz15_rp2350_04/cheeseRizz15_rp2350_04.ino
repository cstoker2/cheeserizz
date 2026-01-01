//cheeseRizz15_rp2350_04.ino
#include <PIO_DShot.h>
#include "CRSFforArduino.hpp"
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DotStar.h>
#include <SimpleKalmanFilter.h>
#define TIMER_INTERRUPT_DEBUG 0      // activate print debugs in isr
#define _TIMERINTERRUPT_LOGLEVEL_ 4  // Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#include "RPi_Pico_TimerInterrupt.h"
#include "header.h"

Adafruit_NeoPixel pixel(1, NEOPIXEL_BUILTIN, NEO_GRB + NEO_KHZ800);
Adafruit_DotStar leds(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
Adafruit_H3LIS331 lis = Adafruit_H3LIS331();
CRSFforArduino *crsf = nullptr;
BidirDShotX1 *esc1;
BidirDShotX1 *esc2;
SimpleKalmanFilter kalmanFilter(20, 20, 0.001);
RPI_PICO_Timer AccelTimer(0);
RPI_PICO_Timer DshotTimer(1);
RPI_PICO_Timer LEDTimer(2);

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

void setup() {
  // Initialise the serial port & wait for the port to open.
  Serial.begin(115200);
  Serial2.begin(115200);  // D12 is TX pin on back
  delay(2000);
  Serial.println("cheeseRizz_rp2350_04");
  //telemVal[0] = 15.3;  // bump version number display

  // Initialise CRSF for Arduino.
  crsf = new CRSFforArduino(&Serial1);  //A0=RX A1=TX on qtrp2040  D6,D7 = Serial1 on rp2350xiao
  if (!crsf->begin()) {
    crsf->end();

    delete crsf;
    crsf = nullptr;

    Serial.println("CRSF for Arduino initialisation failed!");
    while (1) {
      delay(10);
    }
  }
  rcChannelCount = rcChannelCount > crsfProtocol::RC_CHANNEL_COUNT ? crsfProtocol::RC_CHANNEL_COUNT : rcChannelCount;
  crsf->setRcChannelsCallback(onReceiveRcChannels);

  // make the DSHOTs
  esc1 = new BidirDShotX1(DSHOT1_PIN);
  esc2 = new BidirDShotX1(DSHOT2_PIN);

  //Accelerometer
  if (!lis.begin_SPI(H3LIS331_CS, H3LIS331_SCK, H3LIS331_MISO, H3LIS331_MOSI)) {
    //if (! lis.begin_I2C()) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start accel");
    while (1) yield();
  }
  Serial.println("H3LIS331 found!");
  // lis.setRange(H3LIS331_RANGE_100_G);   // 100, 200, or 400 G!
  Serial.print("Range set to: ");
  switch (lis.getRange()) {
    case H3LIS331_RANGE_100_G: Serial.println("100 g"); break;
    case H3LIS331_RANGE_200_G: Serial.println("200 g"); break;
    case H3LIS331_RANGE_400_G: Serial.println("400 g"); break;
  }
  // lis.setDataRate(LIS331_DATARATE_1000_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {

    case LIS331_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS331_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS331_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS331_DATARATE_400_HZ: Serial.println("400 Hz"); break;
    case LIS331_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
    case LIS331_DATARATE_LOWPOWER_0_5_HZ: Serial.println("0.5 Hz Low Power"); break;
    case LIS331_DATARATE_LOWPOWER_1_HZ: Serial.println("1 Hz Low Power"); break;
    case LIS331_DATARATE_LOWPOWER_2_HZ: Serial.println("2 Hz Low Power"); break;
    case LIS331_DATARATE_LOWPOWER_5_HZ: Serial.println("5 Hz Low Power"); break;
    case LIS331_DATARATE_LOWPOWER_10_HZ: Serial.println("10 Hz Low Power"); break;
  }
  // accel callibrate offsets
  int sampleCount = 150;
  float xSum = 0, ySum = 0, zSum = 0;
  for (int i = 0; i < sampleCount; i++) {
    sensors_event_t s;
    lis.getEvent(&s);
    xSum += s.acceleration.x;
    ySum += s.acceleration.y;
    zSum += s.acceleration.z;
    delayMicroseconds(2500);
  }
  accelOffsetX = xSum / sampleCount;
  accelOffsetY = ySum / sampleCount;
  accelOffsetZ = zSum / sampleCount;

  Serial.printf("\nCalibrated acccelerometer offsets %.3f %.3f %.3f \n", accelOffsetX, accelOffsetY, accelOffsetZ);

  // LEDS: (builtin yellow, builtin Neopixel, and indicator dotstar led strip)
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);  //low is on

  pinMode(NEOPIXEL_POWER, OUTPUT);  // turn on neopixel
  digitalWrite(NEOPIXEL_POWER, HIGH);
  pixel.begin();
  leds.begin();  // Initialize pins for output
  leds.show();
  pixel.show();  // the first show() seems to get skipped? So use this one so the next one works.
  Serial.println("LED check");
  leds.fill(0xFF0000);                             // all green
  pixel.setPixelColor(0, pixel.Color(0, 250, 0));  //green
  leds.show();
  pixel.show();

  delay(500);  // show off the lights

  digitalWrite(PIN_LED, HIGH);                   //off
  leds.fill(0x000000);                           // all off
  pixel.setPixelColor(0, pixel.Color(0, 0, 0));  //off
  leds.show();
  pixel.show();

  //start interrupt timers:
  if (AccelTimer.attachInterruptInterval(ACCEL_IRQ_INTERVAL * 1000, AccelTimerHandler)) {  // run 1000hz
    AccelTimer.restartTimer();
  }
  if (DshotTimer.attachInterruptInterval(DSHOT_IRQ_INTERVAL * 1000, DshotTimerHandler)) {  // run 625hz
    DshotTimer.restartTimer();
  }
  if (LEDTimer.attachInterruptInterval(LED_IRQ_INTERVAL * 1000, LEDTimerHandler)) {  // run 20hz
    LEDTimer.restartTimer();
  }

  Serial.println("Setup complete");
  delay(500);
}

bool DshotTimerHandler(struct repeating_timer *t) {
  (void)t;
  if (DEBUG_TIMERS) {
    dshotHz = 1000000.0 / (micros() - dshotMicros);
    dshotMicros = micros();
  }
  int dshotValueMotor1 = pwm2dshot(motor1Throttle);
  int dshotValueMotor2 = pwm2dshot(motor2Throttle);
  esc1->sendThrottle(dshotValueMotor1);
  esc2->sendThrottle(dshotValueMotor2);

  return true;
}

bool AccelTimerHandler(struct repeating_timer *t) {
  (void)t;
  lis.getEvent(&accel_event);
  if (DEBUG_ACCEL) {
    sensHz = 1000000.0 / (micros() - sensMicros);
    sensMicros = micros();
  }
  return true;
}

bool LEDTimerHandler(struct repeating_timer *t) {
  (void)t;
  if (!ledInterruptsEnabled) return true;  // quick bypass option
  static uint8_t i = 0;                    // steps through the pattern
  rbit = (redState >> i) & 0x1;
  gbit = (greenState >> i) & 0x1;
  bbit = (blueState >> i) & 0x1;
  volatile uint32_t sumbits = rbit * RED + gbit * GREEN + bbit * BLUE;
  leds.clear();  // do we have to clear?
  for (int k = 0; k <= NUMPIXELS; k++) { leds.setPixelColor(k, sumbits); }
  leds.show();
  i = (i + 1) & 0xF;

  if (DEBUG_TIMERS) {
    ledHz = 1000000.0 / (micros() - ledMicros);
    ledMicros = micros();
  }
  return true;
}

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels) {
  FAILSAFE = rcChannels->failsafe;

  if (CRSF_DEBUG) {
    /* Print RC channels every 100 ms. */
    unsigned long thisTime = millis();
    static unsigned long lastTime = millis();

    /* Compensate for millis() overflow. */
    if (thisTime < lastTime) {
      lastTime = thisTime;
    }

    if (thisTime - lastTime >= 100) {
      lastTime = thisTime;

      Serial.print("RC Channels <");
      Serial.print("FS: ");
      Serial.print(rcChannels->failsafe);
      Serial.print(" ");
      // for (int i = 1; i <= rcChannelCount; i++) {
      for (int i = 1; i <= 8; i++) {  // only using 1st 8
        Serial.print(rcChannelNames[i - 1]);
        Serial.print(": ");
        Serial.print(crsf->rcToUs(crsf->getChannel(i)));

        if (i < rcChannelCount) {
          Serial.print(", ");
        }
      }
      Serial.println(">");
    }
  }
}

void setThrottle(float throttle1, float throttle2) {

  motor1Throttle = throttleToPWM(throttle1 * MOTOR1_DIRECTION);
  motor2Throttle = throttleToPWM(throttle2 * MOTOR2_DIRECTION);
}

int throttleToPWM(float throttle) {
  throttle = constrain(throttle, -1.0, 1.0);

  // Linear interpolation: output = min + (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin)
  float pwmFloat = 1000.0 + (throttle - (-1.0)) * (2000.0 - 1000.0) / (1.0 - (-1.0));

  return (int)pwmFloat;
}

int pwm2dshot(int pwm_throttle) {  // maps  1500 +/- DEADBAND to 0, (1501,2000) to (0,1000), (1499,1000) to (1000,1999)
  // DShot wants throttle 0 to 1999

  // In DShot, any throttle value < 48 is a motor configuration command.
  // The throttle ranges are then 1000 steps wide, from 48 -> 1047 for direction 1 and 1049-2047 for direction 2
  // Yes, this means that we go from full forwards throttle at 1047 to minimum reverse throttle at 1049.
  // Also note, 1048 is not 0! We need to send an actual 0 for 0.
  // See: https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs

  const int DEADBAND = 10;

  if ((pwm_throttle <= 1500 + DEADBAND) && (pwm_throttle >= 1500 - DEADBAND)) {
    return 0;
  }

  if (pwm_throttle > 1500 + DEADBAND) {
    pwm_throttle = map(pwm_throttle, 1501, 2000, 0, 1000);
    return min(pwm_throttle, 999);
    ;
  }

  // otherwise, pwm_throttle must be less than zero
  pwm_throttle = map(pwm_throttle, 1000, 1499, 1000, 0);
  return min((pwm_throttle), 999) + 1000;  //48;
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
  float x = stickHoriz / 6.0;  // Reduced to 16% for less twitchy steering
  float y = stickVert / 5.0;
  float m1 = y + x;  // Left motor
  float m2 = y - x;  // Right motor

  m1 = modulateThrottle(m1, 0);
  m2 = modulateThrottle(m2, 1);

  setThrottle(m1, m2);
}

void updateInputs() {
  int horiz = crsf->rcToUs(crsf->getChannel(1));
  int vert = crsf->rcToUs(crsf->getChannel(2));

  stickVert = (vert - 1500) / 500.0;
  stickHoriz = (horiz - 1500) / 500.0;
  stickAngle = -atan2(stickHoriz, stickVert) / (2 * PI);
  stickLength = sqrt(sq(stickVert) + sq(stickHoriz));
  stickLength = min(stickLength, 1.0);
  if (stickLength < 0.05) {
    stickAngle = 0;
    stickLength = 0;
  }
  throttle = crsf->rcToUs(crsf->getChannel(3));
  throttle = (throttle - 1000) / 1000.0;  // normalize to [0-1]
  rudderInput = crsf->rcToUs(crsf->getChannel(4));
  inputToggleL = crsf->rcToUs(crsf->getChannel(6));
  inputToggleR = crsf->rcToUs(crsf->getChannel(7));
  inputPot = crsf->rcToUs(crsf->getChannel(8));

  if (inputToggleR > 1600) {  // switch up = leds
    ledOffset = (inputPot - 1500) / 1000.0;
  }
  if (inputToggleR < 1400) {  // switch down = radius
    radiusInput = inputPot;
  }

  rudderInput = map(rudderInput, 1000, 2000, -300, 300);                // rudder can change by =/- 3mm
  radiusSize = map(radiusInput + rudderInput, 1000, 2000, 2500, 3500);  // 25mm to 35mm range
  radiusSize = radiusSize / 100000.0;

  kalmanQ = inputToggleL;  // should be 1000, 1500 or 2000

  kalmanQ = constrain(kalmanQ, 1000, 2000);
  kalmanQ = map(kalmanQ, 1000, 2000, 0, 3);
  kalmanQ = 0.5 + (1.0 / pow(10, kalmanQ));
  kalmanFilter.setProcessNoise(kalmanQ);

  aux6 = crsf->rcToUs(crsf->getChannel(10));
  aux6 = constrain(aux6, 1000, 2000);
  aux6 = (aux6 - 950) / 500.0;  // should be 0.1-2.1 float value.
  aux7 = crsf->rcToUs(crsf->getChannel(11));
  aux7 = (aux7 - 1000) / 1000.0;  // 0-1.0 float value
}

void setPattern(uint16_t r, uint16_t g, uint16_t b) {
  redState = g;
  greenState = r;
  blueState = b;
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

float calculateW() {  // probably takes more than 310us to get sensor data
  // Get accelerometer data
  sensors_event_t s;
  lis.getEvent(&s);

  // 1. Validate raw Z-axis acceleration
  if (isnan(s.acceleration.z) || isinf(s.acceleration.z)) {
    if (DEBUG_HOTLOOP) Serial.printf(" Bad Z-accel: %.2f (X=%.2f, Y=%.2f)\n", s.acceleration.z, s.acceleration.x, s.acceleration.y);
    return RPS_THRESHOLD * 2 * PI;  // Return safe value in radians/sec
  }

  // 2. Apply calibration offsets
  float z = s.acceleration.z - accelOffsetZ;

  // 3a. Update Kalman filter with calibrated value
  //float spinAccel = kalmanFilter.updateEstimate(z);
  // 3b. plain sensor reading instead of kalman
  float spinAccel = z;
  estimated_accel = spinAccel;  // Store for telemetry


  // 4. Check accelerometer data after filtering
  if ((spinAccel <= 0) || isnan(spinAccel) || isinf(spinAccel)) {
    if (DEBUG_HOTLOOP) Serial.printf(" Invalid accel data: %1.2f\n", spinAccel);
    return RPS_THRESHOLD * 2 * PI;  // Fallback to safe value in radians/sec
  }

  // 5. Check radius size
  if (radiusSize <= 0 || isnan(radiusSize) || isinf(radiusSize)) {
    if (DEBUG_HOTLOOP) Serial.printf(" Invalid radius: %.4f\n", radiusSize);
    return RPS_THRESHOLD * 2 * PI;  // Fallback to safe value in radians/sec
  }

  // 6. Calculate w (angular velocity in radians/sec)
  float w = sqrt(spinAccel / radiusSize);

  // 7. Calculate RPS for compatibility with existing code
  float rps = w / (2 * PI);

  // 8. keep track of maximum rps
  if (rps > maxRPS) {
    maxRPS = rps;
  }

  lastRPS = rps;  // Store for telemetry and other uses

  // 9. Ensure w does not fall below threshold
  if (w < RPS_THRESHOLD * 2 * PI) {
    w = RPS_THRESHOLD * 2 * PI;
  }

  return w;
}

void updatePhaseTracking(float w) {  // whenever called it updates currentPhase by adding w*dT and keeping track of last update time.
  // Get current time
  unsigned long now = micros();

  // Calculate time since last update
  unsigned long deltaTime = now - lastPhaseUpdate;

  // Skip if this is the first call or after a long delay
  if (lastPhaseUpdate == 0 || deltaTime > 100000) {  // 100ms
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

void checkLoggingTrigger() {
  if (aux7 > 0.6 && !loggingActive && !dataReady && logIndex < LOG_BUFFER_SIZE) {
    loggingActive = true;
    logIndex = 0;
    if (DEBUG_HLTELEM) { Serial.println("HL Logging started"); }
  }
}

void logSample(uint32_t usec0, float sample1, float sample2, float sample3, float sample4, float sample5, float sample6, float sample7) {
  if (loggingActive && logIndex < LOG_BUFFER_SIZE) {
    logBuffer[logIndex].usec0 = usec0;
    logBuffer[logIndex].sample1 = sample1;
    logBuffer[logIndex].sample2 = sample2;
    logBuffer[logIndex].sample3 = sample3;
    logBuffer[logIndex].sample4 = sample4;
    logBuffer[logIndex].sample5 = sample5;
    logBuffer[logIndex].sample6 = sample6;
    logBuffer[logIndex].sample7 = sample7;

    logIndex++;

    if (logIndex >= LOG_BUFFER_SIZE) {
      loggingActive = false;
      dataReady = true;
      if (DEBUG_HLTELEM) { Serial.println("HL Buffer full - logging stopped"); }
    }
  }
}

void checkDumpTrigger() {
  if (throttle < ZERO_THROTTLE_THRESHOLD && dataReady && !dumpMode) {
    dumpMode = true;
    dumpIndex = 0;
    if (DEBUG_HLTELEM) { Serial.println("Entering HL dump mode"); }
  }
}

void dumpLoggedData() {
  if (dumpMode && dumpIndex < logIndex) {
    telemVal[0] = (float)logBuffer[dumpIndex].usec0;  // u32: microseconds
    telemVal[1] = logBuffer[dumpIndex].sample1;       // f: phase
    telemVal[2] = logBuffer[dumpIndex].sample2;       // f: throttle
    telemVal[3] = logBuffer[dumpIndex].sample3;       // f: cos(phase1)
    telemVal[4] = logBuffer[dumpIndex].sample4;       // f: loop count
    telemVal[5] = logBuffer[dumpIndex].sample5;       // f: unused (0.0)
    telemVal[6] = logBuffer[dumpIndex].sample6;       // f: unused (0.0)
    telemVal[7] = logBuffer[dumpIndex].sample7;       // f: magic number (17.1717)

    dumpIndex++;

    if (dumpIndex >= logIndex) {
      dumpMode = false;
      dataReady = false;
      logIndex = 0;
      if (DEBUG_HLTELEM) { Serial.println("HL Dump complete"); }
    }
  }
}

void MeltybrainDrive1() {
  // Calculate initial angular velocity (w)
  float W = calculateW();
  updatePhaseTracking(W);

  // Calculate expected revolution time based on angular velocity
  unsigned long exitDurationMicros = (unsigned long)((2.0 * PI / W) * 1000000);

  ledInterruptsEnabled = false;  // Bypass LED interrupt action

  // Track hot loop iterations for diagnostic purposes
  int32_t hotLoopCount = 0;
  // track loop entry time
  unsigned long usLoopStartTime = micros();

  // HOT LOOP
  while (true) {
    // Get current time
    unsigned long currentTimeMicros = micros();

    if (DEBUG_HOTHZ) {  // timing this loop
      hotHz = 1000000.0 / (currentTimeMicros - hotMicros);
      hotMicros = currentTimeMicros;
    }

    // Exit conditions
    if (throttle < ZERO_THROTTLE_THRESHOLD || (currentTimeMicros - usLoopStartTime) > 2000000) {
      if (DEBUG_HOTLOOP) Serial.println(" Throttle zero or Timeout");
      break;
    }

    if ((currentTimeMicros - usLoopStartTime) > exitDurationMicros) {  // one revolution time elapsed
      break;
    }

    // Update angular velocity and phase tracking
    W = calculateW();
    updatePhaseTracking(W);

    hotLoopCount++;  // Count iterations of the hot loop

    float forwardPhase = normalize(continuousPhase + stickAngle, -0.5, 0.5);
    float backwardPhase = normalize(continuousPhase + stickAngle + 0.5, -0.5, 0.5);
    //float ledPhase = normalize(continuousPhase + stickAngle + ledOffset, -0.5, 0.5);
    float ledPhase = normalize(continuousPhase + ledOffset, -0.5, 0.5);  // try fixed leds

    float cos_ph1 = cos(forwardPhase * 2 * PI);
    float cos_ph2 = cos(backwardPhase * 2 * PI);
    float cos_led = cos(ledPhase * 2 * PI);

    float widthScale = max(stickLength, throttle);
    // float th1 = max(0, (cos_ph1 * TRANSL_STRENGTH * stickLength) + throttle);
    //float th2 = max(0, (cos_ph2 * TRANSL_STRENGTH * stickLength) + throttle);
    float th1 = max(0, (cos_ph1 * aux6 * stickLength) + throttle);  // try with adjustable strength pulse
    float th2 = max(0, (cos_ph2 * aux6 * stickLength) + throttle);

    if (DEBUG_HOTLOOP) {
      Serial.print(" th1:");
      Serial.println(th1);
    }
    setThrottle(th1, -th2);  // -th2 because of CW spin direction

    logSample(
      currentTimeMicros,    // usec0: relative microseconds
      continuousPhase,      // sample1: phase
      th1,                  // sample2: motor1 throttle
      cos_ph1,              // sample3: cos(phase1)
      (float)hotLoopCount,  // sample4: loop count
      0.0,                  // sample5: unused
      0.0,                  // sample6: unused
      17.1717               // sample7: magic number
    );


    bool LEDOn = cos_led > 0.7071 * (1.4 - widthScale * 0.9);  // Magic numbers for ~45deg arc
    uint32_t COLOR = LEDOn * BLUE + LEDOn * GREEN;
    float currentRPS = W / (2 * PI);
    if (currentRPS <= RPS_THRESHOLD) { COLOR = LEDOn * BLUE; }  // just BLUE if below threshold
    for (int l = 0; l <= NUMPIXELS; l++) { leds.setPixelColor(l, COLOR); }
    leds.show();
  }

  ledInterruptsEnabled = true;  // Resume LED interrupt action
}

void loop() {
  static bool meltyMode = false;
  static bool tankMode = false;
  static int i, j = 0;
  static uint32_t lastTime = 0;
  j++;
  if (j % 250 == 0) { pulse = !pulse; }  //led blink every 250 reads (1/4 sec?)

  crsf->update();
  updateInputs();

  checkLoggingTrigger();  // ADD THIS LINE
  checkDumpTrigger();     // ADD THIS LINE

  if (FAILSAFE == true) {
    pixel.setPixelColor(0, 255, 0, 0);  // red
    setPattern(LED_PATTERN_RADIO_ERROR);
    setThrottle(0, 0);
  } else {
    pixel.setPixelColor(0, 0, 255, 0);  //green
    setPattern(LED_PATTERN_LOWRPS);
  }

  meltyMode = (throttle > ZERO_THROTTLE_THRESHOLD);
  tankMode = (throttle < ZERO_THROTTLE_THRESHOLD);

  if (tankMode && !FAILSAFE) {  // Tank mode
    setPattern(LED_PATTERN_CYCLE);
    TankDrive();
  } else if (meltyMode && !FAILSAFE) {  // Melty mode
    setPattern(LED_PATTERN_OFF);
    MeltybrainDrive1();
  } else {  //error shouldn't be here
  }

  delayMicroseconds(200);

  //rpm++;                            // eRPM = RPM * poles/2
  if (millis() - lastTime > 100) {  // run 10hz stuff, mainly display
    lastTime = millis();
    if (DEBUG_TIMERS) {  // serial stuff
      Serial.print(" sensHz: ");
      Serial.print(sensHz);
      Serial.print("\tledHz: ");
      Serial.print(ledHz);
      Serial.print("\tdshotHz: ");
      Serial.println(dshotHz);
    }
    if (DEBUG_SERIAL) {
      Serial.print(" FS: ");
      Serial.print(FAILSAFE);
      Serial.print(" sensHz: ");
      Serial.print(sensHz);
      Serial.print(" hotHz: ");
      Serial.print(hotHz);
      Serial.print(" T1: ");
      Serial.print(motor1Throttle);
      Serial.print(" T2: ");
      Serial.print(motor2Throttle);
      Serial.print(" lastRPS: ");
      Serial.print(lastRPS);
      Serial.printf(" Rmm: %.1f ", radiusSize * 1000);
      Serial.print(" Led: ");
      Serial.print(ledOffset);
      Serial.print(" Z: ");
      Serial.println(accel_event.acceleration.z - accelOffsetZ);
    }
    i = !i;

    if (TELEMETRY) {
      if (dumpMode) {
        // Populate telemVal with logged data
        dumpLoggedData();

        // Send telemetry
        for (int k = 0; k < 8; k++) {
          if (k == 0) {
            // Index 0: uint32_t format
            Serial2.printf("%s,%lu", telemDumpLbl[k], (uint32_t)telemVal[k]);
          } else {
            // Indices 1-7: float format
            Serial2.printf("%s,%f", telemDumpLbl[k], telemVal[k]);
          }
          if (k < 7) {
            Serial2.print(",");
          } else {
            Serial2.print("\n");
          }
        }

      } else {
        // Normal mode telemetry
        telemVal[0] = micros();                                     // u32: microsecond timestamp
        telemVal[1] = radiusSize * 1000.0;                          // f: radius in mm
        telemVal[2] = ledOffset;                                    // f: LED offset
        telemVal[3] = hotHz;                                        // f: hot loop frequency
        telemVal[4] = aux6;                                         // f: mix fraction (aux6)
        telemVal[5] = kalmanQ;                                      // f: kalman Q parameter
        telemVal[6] = lastRPS * 60.0;                               // f: RPM
        telemVal[7] = (accel_event.acceleration.x - accelOffsetX);  // f: accel X (up/down)

        // Send telemetry
        for (int k = 0; k < 8; k++) {
          if (k == 0) {
            // Index 0: uint32_t format
            Serial2.printf("%s,%lu", telemLbl[k], (uint32_t)telemVal[k]);
          } else {
            // Indices 1-7: float format
            Serial2.printf("%s,%f", telemLbl[k], telemVal[k]);
          }
          if (k < 7) {
            Serial2.print(",");
          } else {
            Serial2.print("\n");
          }
        }
      }
    }

    pixel.show();                  // display pixel
    digitalWrite(PIN_LED, pulse);  //send led pin
  }
}
