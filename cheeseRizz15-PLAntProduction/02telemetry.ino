
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
    Serial1.println("FORMAT,timestamp,magX,magY,heading");

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

      // Calculate normalized X and Y values
      float norm_x = (magBuffer[currentIndex].rawX - magXOffset) / 131072.0;
      float norm_y = (magBuffer[currentIndex].rawY - magYOffset) / 131072.0;
      
      float mag_heading = normalize((atan2(norm_y, norm_x) / (2 * PI)), 0.0, 1.0);

      // Send magnetometer data with calculated heading
      Serial1.printf("MAG,%lu,%lu,%lu,%.4f\n",
                     magBuffer[currentIndex].timestamp,
                     magBuffer[currentIndex].rawX,
                     magBuffer[currentIndex].rawY,
                     mag_heading);

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

void sendRPMOverBT() {
  static uint32_t lastRPMSendTime = 0;
  uint32_t currentTime = millis();
  
  // Send RPM data twice per second (every 500ms)
  if (currentTime - lastRPMSendTime >= 500) {
    // Calculate RPM from RPS (revolutions per second)
    float rpm = lastRPS * 60.0;
    
    // Send RPM data over Bluetooth (Serial1)
    //Serial1.print("rpm ");
    Serial1.println(rpm);
    
    // Update last send time
    lastRPMSendTime = currentTime;
  }
}

void sendRecentTelemetryOverBT() {
  static uint32_t lastTransmissionTime = 0;
  static bool headerSent = false;
  
  // Read channel 10 to check if recent telemetry is requested
  int telemetryTrigger = ibus.readChannel(9);

  // Only send data if trigger is activated and not too frequent (at least 500ms between updates)
  if (telemetryTrigger > 1300 && (millis() - lastTransmissionTime > 500)) {
    // Calculate the index of the most recent data point
    uint16_t recentIndex;
    if (telemetryBufferCount == 0) {
      // No data available yet
      return;
    } else {
      // Get most recent entry (one position before the current write position)
      recentIndex = (telemetryBufferIndex == 0) ? 
                    (TELEMETRY_BUFFER_SIZE - 1) : 
                    (telemetryBufferIndex - 1);
    }

    // Send header with only the fields we're using
    if (!headerSent || (millis() - lastTransmissionTime > 5000)) {
      Serial1.println("RECENT_TELEM_FORMAT,timestamp,rpm,radius,kalmanQ,stickAngle,stickLength,throttle,zAccelG,hotLoopCount");
      headerSent = true;
    }

    // Get the most recent telemetry data entry
    TelemetryData* data = &telemetryBuffer[recentIndex];

    // Send telemetry data record with only the fields we're using
    Serial1.print("RECENT_TELEM,");
    Serial1.print(data->timestamp); // timestamp
    Serial1.print(",");
    Serial1.print(data->fl1, 1);    // rpm
    Serial1.print(",");
    Serial1.print(data->fl2, 4);    // radius
    Serial1.print(",");
    Serial1.print(data->fl3, 4);    // kalmanQ
    Serial1.print(",");
    Serial1.print(data->fl4, 4);    // stickAngle
    Serial1.print(",");
    Serial1.print(stickLength, 4);  // stickLength (current value, not from telemetry)
    Serial1.print(",");
    Serial1.print(data->fl5, 4);    // throttle
    Serial1.print(",");
    Serial1.print(data->fl6, 4);    // zAccelG
    Serial1.print(",");
    Serial1.print(data->int1);      // hotLoopCount

    Serial1.println();

    // Update last transmission time
    lastTransmissionTime = millis();
  }
}