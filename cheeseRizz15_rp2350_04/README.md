/*
# CheeseRizz Robot Code Modifications

## Summary of Changes

Modified the robot code to work with the updated ESPNowLogger library that supports:
1. **10-character label limit** (increased from 8)
2. **Mixed data types**: Index 0 as `uint32_t`, indices 1-7 as `float`
3. **Generic LogSample structure** for flexible data logging

---

## Files Modified

### 1. header.h

#### LogSample Structure - Complete Redesign
**CHANGED FROM:**
```cpp
struct LogSample {
  float timestamp_us;
  float phase;
  float m1_throttle;
  float m2_throttle;
  float cos_phase1;
  float hotloop_count;
};
```

**CHANGED TO:**
```cpp
struct LogSample {
  uint32_t usec0;
  float sample1;
  float sample2;
  float sample3;
  float sample4;
  float sample5;
  float sample6;
  float sample7;
};
```

**Impact:**
- Structure size: 24 bytes → 32 bytes (+8 bytes per sample)
- Total buffer: 18KB → 24KB (+6KB)
- More flexible for different logging scenarios
- Aligns with telemetry data types (1 uint32_t + 7 floats)

#### Telemetry Label Arrays - Complete Update
**Normal Mode Labels (telemLbl):**
```cpp
const char *telemLbl[] = {
  "uSec    ",  // u32 0: unsigned int microseconds (NEW!)
  "radius",    //   f 1: radiusSize in mm (moved from index 0)
  "LEDoffst",  //   f 2: LED offset (renamed)
  "hotHZ",     //   f 3: hot loop frequency
  "mixFrac",   //   f 4: aux6 (renamed from "aux6")
  "kalmanQ",   //   f 5: kalman Q parameter (NEW!)
  "rpm",       //   f 6: RPM
  "accelX"     //   f 7: X-axis acceleration
};
```

**Dump Mode Labels (telemDumpLbl):**
```cpp
const char *telemDumpLbl[] = {
  "uSec",      // u32 0: microseconds (changed from "index")
  "phase",     // f 1: phase
  "Throttle",  // f 2: throttle (changed from "m1Th")
  "cosph1",    // f 3: cos(phase1)
  "LoopCnt",   // f 4: loop count
  "",          // f 5: unused
  "",          // f 6: unused
  "magicNum"   // f 7: magic number 17.1717 (changed from "svntn")
};
```

---

### 2. cheeseRizz15_rp2350_04.ino

#### logSample() Function - Complete Redesign

**Signature Changed:**
```cpp
// OLD:
void logSample(float hotLoopCount, float cos_ph1, unsigned long time)

// NEW:
void logSample(uint32_t usec0, float sample1, float sample2, float sample3, 
               float sample4, float sample5, float sample6, float sample7)
```

**Implementation:**
```cpp
void logSample(uint32_t usec0, float sample1, float sample2, float sample3, 
               float sample4, float sample5, float sample6, float sample7) {
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
      if(DEBUG_HLTELEM){Serial.println("HL Buffer full - logging stopped");}
    }
  }
}
```

#### logSample() Call in MeltybrainDrive1() - Updated

**Located around line 376 in the hot loop:**
```cpp
logSample(
  (uint32_t)(currentTimeMicros - usLoopStartTime),  // usec0: relative microseconds
  continuousPhase,                                   // sample1: phase
  th1,                                               // sample2: motor1 throttle
  cos_ph1,                                           // sample3: cos(phase1)
  (float)hotLoopCount,                               // sample4: loop count
  0.0,                                               // sample5: unused
  0.0,                                               // sample6: unused
  17.1717                                            // sample7: magic number
);
```

**Key Changes:**
- usec0 is now relative time (currentTimeMicros - usLoopStartTime)
- Magic number (17.1717) is stored in sample7 during logging
- Samples 5-6 are unused (set to 0.0) for future expansion

#### dumpLoggedData() Function - Updated

**Implementation:**
```cpp
void dumpLoggedData() {
  if (dumpMode && dumpIndex < logIndex) {
    telemVal[0] = (float)logBuffer[dumpIndex].usec0;   // u32: microseconds
    telemVal[1] = logBuffer[dumpIndex].sample1;        // f: phase
    telemVal[2] = logBuffer[dumpIndex].sample2;        // f: throttle
    telemVal[3] = logBuffer[dumpIndex].sample3;        // f: cos(phase1)
    telemVal[4] = logBuffer[dumpIndex].sample4;        // f: loop count
    telemVal[5] = logBuffer[dumpIndex].sample5;        // f: unused (0.0)
    telemVal[6] = logBuffer[dumpIndex].sample6;        // f: unused (0.0)
    telemVal[7] = logBuffer[dumpIndex].sample7;        // f: magic number (17.1717)

    dumpIndex++;

    if (dumpIndex >= logIndex) {
      dumpMode = false;
      dataReady = false;
      logIndex = 0;
      if(DEBUG_HLTELEM){Serial.println("HL Dump complete");}
    }
  }
}
```

**Key Change:**
- telemVal[7] now retrieves sample7 from buffer (containing 17.1717)
- Magic number is stored in buffer, not hardcoded during dump

#### Normal Mode Telemetry - Updated

**Located in loop() around line 581-588:**
```cpp
// Normal mode telemetry
telemVal[0] = (float)micros();           // u32: microsecond timestamp
telemVal[1] = radiusSize * 1000.0;       // f: radius in mm
telemVal[2] = ledOffset;                 // f: LED offset
telemVal[3] = hotHz;                     // f: hot loop frequency
telemVal[4] = aux6;                      // f: mix fraction (aux6)
telemVal[5] = kalmanQ;                   // f: kalman Q parameter
telemVal[6] = lastRPS * 60.0;            // f: RPM
telemVal[7] = (accel_event.acceleration.x - accelOffsetX);  // f: accel X
```

**Key Changes:**
- Index 0: Now sends micros() timestamp (was radiusSize)
- Index 1: Now sends radiusSize (moved from index 0)
- Index 5: Now sends kalmanQ (was FAILSAFE)
- FAILSAFE removed from telemetry

#### Serial2 Output Format - Updated (TWO LOCATIONS)

**Normal Mode Telemetry Output:**
```cpp
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
```

**Dump Mode Telemetry Output:**
```cpp
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
```

**Key Change:**
- Index 0 now uses `%lu` format (for uint32_t)
- Indices 1-7 use `%f` format (for float)
- Applied to both normal and dump modes

---

## Data Flow Summary

### Normal Mode:
```
Robot sensors/state → telemVal array → Serial2 → Master ESP32 → Slave ESP32 display

telemVal[0] = micros()        → uint32_t → Master parses with atol() → Slave displays as integer
telemVal[1-7] = float values  → float    → Master parses with atof() → Slave displays with decimals
```

### Dump Mode:
```
LogBuffer → dumpLoggedData() → telemVal array → Serial2 → Master ESP32 → Slave ESP32 display

telemVal[0] = usec0 (uint32_t cast to float) → uint32_t → Master/Slave handle as above
telemVal[1-7] = sample1-7 (floats)           → float    → Master/Slave handle as above
```

### Hot Loop Logging:
```
MeltybrainDrive1() → logSample() → LogBuffer[logIndex].usec0/sample1-7

usec0 = relative time (uint32_t)
sample1 = continuousPhase
sample2 = th1 (motor1 throttle)
sample3 = cos_ph1
sample4 = hotLoopCount
sample5 = 0.0 (unused)
sample6 = 0.0 (unused)
sample7 = 17.1717 (magic number for searching dumps)
```

---

## Example Serial2 Output

### Normal Mode:
```
uSec    ,123456789,radius,28.50,LEDoffst,0.35,hotHZ,1250.00,mixFrac,1.50,kalmanQ,0.501,rpm,450.00,accelX,2.34
```

### Dump Mode:
```
uSec,45678,phase,0.234,Throttle,1523.00,cosph1,0.987,LoopCnt,125.00,,,,,magicNum,17.1717
```

---

## Testing Checklist

- [ ] Robot code compiles without errors
- [ ] Normal mode telemetry sends micros() as index 0
- [ ] Normal mode telemetry sends all 7 float values correctly
- [ ] Serial2 output uses %lu for index 0, %f for indices 1-7
- [ ] Hot loop logging captures all 8 values correctly
- [ ] Magic number (17.1717) is stored in logBuffer.sample7
- [ ] Dump mode retrieves and sends all logged data correctly
- [ ] Dump mode includes magic number from buffer (not hardcoded)
- [ ] Master ESP32 receives and parses data correctly
- [ ] Slave ESP32 displays uint32_t without decimals
- [ ] Slave ESP32 displays floats with 2 decimal places
- [ ] SD card logging formats CSV correctly

---

## Memory Usage

**LogSample Structure:**
- Old: 6 floats = 24 bytes
- New: 1 uint32_t + 7 floats = 32 bytes
- Change: +8 bytes per sample

**Total Buffer (750 samples):**
- Old: 18,000 bytes (18KB)
- New: 24,000 bytes (24KB)
- Change: +6KB

**RP2350 RAM:**
- Total: 520KB
- Buffer usage: 24KB (~4.6%)
- Plenty of headroom remaining

---

## Future Expansion

The LogSample structure now has 2 unused slots (sample5 and sample6) that can be used for logging additional data in future updates:

**Possible additions:**
- sample5: motor2Throttle (for debugging differential behavior)
- sample6: stickLength or stickAngle (for input analysis)
- sample6: lastRPS or W (for velocity tracking)
- sample6: estimated_accel (for sensor analysis)

To add new logged data, simply modify the logSample() call in MeltybrainDrive1() and update telemDumpLbl accordingly.

---

## Notes

- **Index 0 is ALWAYS uint32_t** across the entire system
- **Labels can now be up to 10 characters** (was 8)
- **Magic number (17.1717) is stored in buffer**, not hardcoded during dump
- **Backwards compatibility broken** - all devices must be updated together
- **Serial link capacity:** Still only ~14% at 10Hz, plenty of headroom
*/