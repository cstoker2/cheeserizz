//cheeseRizz15_rp2350_03_with_logging.ino
#include <PIO_DShot.h>
#include "CRSFforArduino.hpp"
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DotStar.h>
#include <SimpleKalmanFilter.h>
#define TIMER_INTERRUPT_DEBUG 0      // defines add print debugs in isr
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
  Serial.println("cheeseRizz_rp2350_03_with_logging");
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

int pwm2dshot(int pwm_throttle) {
  // DShot wants throttle 0 to 1999
  // In DShot, any throttle value < 48 is a motor configuration command.
  // The throttle ranges are then 1000 steps wide, from 48 -> 1047 for direction 1 and 1049-2047 for direction 2
  // Yes, this means that we go from full forwards throttle at 1047 to minimum reverse throttle at 1049.
  // Also note, 1048 is not 0! We need to send an actual 0 for 0.

  const int DSHOT_DEADBAND = 3;  // pwm above which motors spin
  const int DSHOT_MIN = 48;
  const int DSHOT_MID = 1048;
  const int DSHOT_MAX = 2047;

  if (pwm_throttle >= (1500 + DSHOT_DEADBAND) && pwm_throttle <= 2000) {
    return map(pwm_throttle, 1500 + DSHOT_DEADBAND, 2000, DSHOT_MIN, DSHOT_MID - 1);
  } else if (pwm_throttle <= (1500 - DSHOT_DEADBAND) && pwm_throttle >= 1000) {
    return map(pwm_throttle, 1500 - DSHOT_DEADBAND, 1000, DSHOT_MID + 1, DSHOT_MAX);
  } else {
    return 0;  // if within deadband, send 0
  }
}

void setPattern(uint16_t r, uint16_t g, uint16_t b) {
  redState = r;
  greenState = g;
  blueState = b;
}

float normalize(float value, float min, float max) {
  float range = max - min;
  while (value < min) value += range;
  while (value >= max) value -= range;
  return value;
}

void TankDrive() {
  float leftSpeed = stickVert + rudderInput;
  float rightSpeed = stickVert - rudderInput;

  leftSpeed = constrain(leftSpeed, -1.0, 1.0);
  rightSpeed = constrain(rightSpeed, -1.0, 1.0);

  setThrottle(leftSpeed, rightSpeed);
}

void updateInputs() {
  stickVert = ((crsf->rcToUs(crsf->getChannel(2)) - 1500.0) / 500.0);
  stickHoriz = ((crsf->rcToUs(crsf->getChannel(1)) - 1500.0) / 500.0);
  throttle = ((crsf->rcToUs(crsf->getChannel(3)) - 1000.0) / 1000.0);
  rudderInput = ((crsf->rcToUs(crsf->getChannel(4)) - 1500.0) / 500.0);
  inputToggleL = ((crsf->rcToUs(crsf->getChannel(5)) - 1000.0) / 1000.0);
  inputToggleR = ((crsf->rcToUs(crsf->getChannel(6)) - 1000.0) / 1000.0);
  inputPot = ((crsf->rcToUs(crsf->getChannel(8)) - 1000.0) / 1000.0);
  aux6 = ((crsf->rcToUs(crsf->getChannel(10)) - 1000.0) / 1000.0);
  aux7 = ((crsf->rcToUs(crsf->getChannel(11)) - 1000.0) / 1000.0);

  stickLength = sqrt(stickVert * stickVert + stickHoriz * stickHoriz);
  stickLength = constrain(stickLength, 0.0, 1.0);
  stickAngle = atan2(stickVert, stickHoriz) / (2.0 * PI);
  stickAngle = normalize(stickAngle - 0.25, -0.5, 0.5);

  radiusInput = inputToggleL;
  ledOffset = inputToggleR;

  if (radiusInput < 0.33) {
    radiusSize = 0.025;
  } else if (radiusInput >= 0.33 && radiusInput < 0.66) {
    radiusSize = 0.030;
  } else {
    radiusSize = 0.035;
  }

  radiusSize += rudderInput * 0.003;
  ledOffset = ledOffset - 0.5;

  if (aux6 < 0.33) {
    aux6 = 0.5;
  } else if (aux6 >= 0.33 && aux6 < 0.66) {
    aux6 = 0.75;
  } else {
    aux6 = 1.0;
  }
}

float calculateW() {
  float accelX = accel_event.acceleration.x - accelOffsetX;
  float accelY = accel_event.acceleration.y - accelOffsetY;
  float accelZ = accel_event.acceleration.z - accelOffsetZ;

  if (isnan(accelX)) accelX = 0;
  if (isnan(accelY)) accelY = 0;
  if (isnan(accelZ)) accelZ = 0;

  float spinAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  kalmanInput = aux7;
  if (kalmanInput < 0.33) {
    kalmanQ = 0.01;
  } else if (kalmanInput >= 0.33 && kalmanInput < 0.66) {
    kalmanQ = 0.1;
  } else {
    kalmanQ = 1.0;
  }

  kalmanFilter.setProcessNoise(kalmanQ);
  estimated_accel = kalmanFilter.updateEstimate(spinAccel);

  spinAccel = estimated_accel;

  if (spinAccel <= 0 || isnan(spinAccel) || isinf(spinAccel)) {
    if (DEBUG_HOTLOOP) Serial.printf(" Invalid accel data: %1.2f\n", spinAccel);
    return RPS_THRESHOLD * 2 * PI;
  }

  if (radiusSize <= 0 || isnan(radiusSize) || isinf(radiusSize)) {
    if (DEBUG_HOTLOOP) Serial.printf(" Invalid radius: %.4f\n", radiusSize);
    return RPS_THRESHOLD * 2 * PI;
  }

  float w = sqrt(spinAccel / radiusSize);

  float rps = w / (2 * PI);

  if (rps > maxRPS) {
    maxRPS = rps;
  }

  lastRPS = rps;

  if (w < RPS_THRESHOLD * 2 * PI) {
    w = RPS_THRESHOLD * 2 * PI;
  }

  return w;
}

void updatePhaseTracking(float w) {
  unsigned long now = micros();

  unsigned long deltaTime = now - lastPhaseUpdate;

  if (lastPhaseUpdate == 0 || deltaTime > 100000) {
    lastPhaseUpdate = now;
    previousW = w;
    return;
  }

  float deltaAngle = (w + previousW) * 0.5f * deltaTime * 0.000001f;

  continuousPhase += deltaAngle / (2 * PI);

  continuousPhase = normalize(continuousPhase, 0, 1.0);

  previousW = w;
  lastPhaseUpdate = now;
}

void checkLoggingTrigger() {
  if (aux7 > 0.5 && !loggingActive && !dataReady && logIndex < LOG_BUFFER_SIZE) {
    loggingActive = true;
    logIndex = 0;
    //Serial.println("Logging started");
  }
}

void logSample(int32_t hotLoopCount, float cos_ph1) {
  if (loggingActive && logIndex < LOG_BUFFER_SIZE) {
    logBuffer[logIndex].timestamp_us = micros();
    logBuffer[logIndex].phase = continuousPhase;
    logBuffer[logIndex].m1_throttle = motor1Throttle;
    logBuffer[logIndex].m2_throttle = motor2Throttle;
    logBuffer[logIndex].cos_phase1 = cos_ph1;
    logBuffer[logIndex].hotloop_count = hotLoopCount;
    
    logIndex++;
    
    if (logIndex >= LOG_BUFFER_SIZE) {
      loggingActive = false;
      dataReady = true;
      //Serial.println("Buffer full - logging stopped");
    }
  }
}

void checkDumpTrigger() {
  if (throttle < ZERO_THROTTLE_THRESHOLD && dataReady && !dumpMode) {
    dumpMode = true;
    dumpIndex = 0;
    //Serial.println("Entering dump mode");
  }
}

void dumpLoggedData() {
  if (dumpMode && dumpIndex < logIndex) {
    telemVal[0] = (float)dumpIndex;
    telemVal[1] = (float)logBuffer[dumpIndex].timestamp_us;
    telemVal[2] = logBuffer[dumpIndex].phase;
    telemVal[3] = logBuffer[dumpIndex].m1_throttle;
    telemVal[4] = logBuffer[dumpIndex].m2_throttle;
    telemVal[5] = logBuffer[dumpIndex].cos_phase1;
    telemVal[6] = (float)logBuffer[dumpIndex].hotloop_count;
    telemVal[7] = 1717.1717;
    
    dumpIndex++;
    
    if (dumpIndex >= logIndex) {
      dumpMode = false;
      dataReady = false;
      logIndex = 0;
      //Serial.println("Dump complete");
    }
  }
}

void MeltybrainDrive1() {
  float W = calculateW();
  updatePhaseTracking(W);

  unsigned long exitDurationMicros = (unsigned long)((2.0 * PI / W) * 1000000);

  ledInterruptsEnabled = false;

  int32_t hotLoopCount = 0;
  unsigned long usLoopStartTime = micros();

  while (true) {
    unsigned long currentTimeMicros = micros();

    if (DEBUG_HOTHZ) {
      hotHz = 1000000.0 / (currentTimeMicros - hotMicros);
      hotMicros = currentTimeMicros;
    }

    if (throttle < ZERO_THROTTLE_THRESHOLD || (currentTimeMicros - usLoopStartTime) > 2000000) {
      if (DEBUG_HOTLOOP) Serial.println(" Throttle zero or Timeout");
      break;
    }

    if ((currentTimeMicros - usLoopStartTime) > exitDurationMicros) {
      break;
    }

    W = calculateW();
    updatePhaseTracking(W);

    hotLoopCount++;

    float forwardPhase = normalize(continuousPhase + stickAngle, -0.5, 0.5);
    float backwardPhase = normalize(continuousPhase + stickAngle + 0.5, -0.5, 0.5);
    float ledPhase = normalize(continuousPhase + ledOffset, -0.5, 0.5);

    float cos_ph1 = cos(forwardPhase * 2 * PI);
    float cos_ph2 = cos(backwardPhase * 2 * PI);
    float cos_led = cos(ledPhase * 2 * PI);

    float widthScale = max(stickLength, throttle);
    float th1 = max(0, (cos_ph1 * aux6 * stickLength) + throttle);
    float th2 = max(0, (cos_ph2 * aux6 * stickLength) + throttle);

    if (DEBUG_HOTLOOP) {
      Serial.print(" th1:");
      Serial.println(th1);
    }
    setThrottle(th1, -th2);

    logSample(hotLoopCount, cos_ph1);

    bool LEDOn = cos_led > 0.7071 * (1.4 - widthScale * 0.9);
    uint32_t COLOR = LEDOn * BLUE + LEDOn * GREEN;
    float currentRPS = W / (2 * PI);
    if (currentRPS <= RPS_THRESHOLD) { COLOR = LEDOn * BLUE; }
    for (int l = 0; l <= NUMPIXELS; l++) { leds.setPixelColor(l, COLOR); }
    leds.show();
  }

  ledInterruptsEnabled = true;
}

void loop() {
  static bool meltyMode = false;
  static bool tankMode = false;
  static int i, j = 0;
  static uint32_t lastTime = 0;
  j++;
  if (j % 250 == 0) { pulse = !pulse; }

  crsf->update();
  updateInputs();
  
  checkLoggingTrigger();
  checkDumpTrigger();

  if (FAILSAFE == true) {
    pixel.setPixelColor(0, 255, 0, 0);
    setPattern(LED_PATTERN_RADIO_ERROR);
    setThrottle(0, 0);
  } else {
    pixel.setPixelColor(0, 0, 255, 0);
    setPattern(LED_PATTERN_LOWRPS);
  }

  meltyMode = (throttle > ZERO_THROTTLE_THRESHOLD);
  tankMode = (throttle < ZERO_THROTTLE_THRESHOLD);

  if (tankMode && !FAILSAFE) {
    setPattern(LED_PATTERN_CYCLE);
    TankDrive();
  } else if (meltyMode && !FAILSAFE) {
    setPattern(LED_PATTERN_OFF);
    MeltybrainDrive1();
  } else {
  }

  delayMicroseconds(200);

  if (millis() - lastTime > 100) {
    lastTime = millis();
    if (DEBUG_TIMERS) {
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
    
    if (DEBUG_TELEMETRY) {
      const char **currentLabels;
      
      if (dumpMode) {
        dumpLoggedData();
        currentLabels = telemLblDump;
      } else {
        telemVal[0] = radiusSize * 1000.0;
        telemVal[1] = ledOffset;
        telemVal[2] = hotHz;
        telemVal[3] = motor1Throttle;
        telemVal[4] = aux6;
        telemVal[5] = FAILSAFE;
        telemVal[6] = lastRPS * 60.0;
        telemVal[7] = (accel_event.acceleration.x - accelOffsetX);
        currentLabels = telemLbl;
      }
      
      for (int k = 0; k < 8; k++) {
        Serial2.printf("%s,%f", currentLabels[k], telemVal[k]);
        if (k < 7) {
          Serial2.print(",");
        } else {
          Serial2.print("\n");
        }
      }
    }

    pixel.show();
    digitalWrite(PIN_LED, pulse);
  }
}
