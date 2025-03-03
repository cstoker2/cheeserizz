void initLEDs() {
  Serial.print("LEDs ");
  ledTimer.begin(updateLEDs, 50000);  // 50ms interval for LED updates
  leds.begin();
  leds.show();
  leds.setBrightness(BRIGHTNESS);
  Serial.println("initialized.");
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
  Wire.setClock(400000);  // Set I2C to fast mode (400 kHz)  sll s getevent() takes 310us at 400khz.
  accel.setDataRate(LIS331_DATARATE_1000_HZ);
  accel.setRange(H3LIS331_RANGE_400_G);
  //accel.setLPFCutoff(LIS331_LPF_292_HZ);
  Serial.print(" configured 1000hz, 400g ");
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