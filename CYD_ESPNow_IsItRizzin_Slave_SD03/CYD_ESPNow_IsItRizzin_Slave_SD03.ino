// Slave Example - ESP32-2432S028R (CYD) Display - Binary Buffer Version
// Program as esp32 Dev Module
// This will receive and display telemetry data via ESPnow from the robot
// Stores data in RAM as binary floats, writes to SD card as CSV after 5 minutes

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "ESPNowLogger.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// ----------------------------
// SD Reader pins (default VSPI pins)
// ----------------------------
//#define SD_SCK 18
//#define SD_MISO 19
//#define SD_MOSI 23
//#define SD_CS 5

// Create the TFT display object
TFT_eSPI tft = TFT_eSPI();

// Create the logger in slave mode
ESPNowLogger logger(ESPNowLogger::SLAVE);

// Layout constants
#define TEXT_SIZE 1     // Small text
#define VALUE_X 120     // Position for values
#define ROW_HEIGHT 20   // Height per data row
#define TITLE_Y 5       // Title position
#define FIRST_ROW_Y 30  // First data row position

// Colors
#define COLOR_TITLE TFT_WHITE
#define COLOR_LABEL TFT_YELLOW
#define COLOR_VALUE TFT_GREEN
#define COLOR_BACKGROUND TFT_NAVY
#define COLOR_TIMESTAMP TFT_SILVER
#define COLOR_CONNECTION TFT_RED

// For determining connection status
unsigned long lastUpdateTime = 0;
const unsigned long CONNECTION_TIMEOUT = 2000;  // 2 seconds without data means disconnected
bool initialDataReceived = false;

// Store current title to detect changes
char currentTitle[MAX_TITLE_LENGTH] = "";

#define SERIAL_LOG_TEST 1

int LOG_NUM = 1;
char numString[5];     // short string for logging fn
char robotLog[20];     // numbered log file path name

// ============================================================================
// BINARY DATA STORAGE STRUCTURES
// ============================================================================

// Binary data sample structure
struct DataSample {
  float timestamp;
  float values[8];
};

// Large RAM buffer for samples
#define MAX_SAMPLES 1500  // ~1,500 samples = 6.25 min at 4Hz, uses 54KB
DataSample sampleBuffer[MAX_SAMPLES];
int sampleCount = 0;
bool bufferFull = false;

// Timing for collection phases
#define COLLECTION_DURATION_MS 300000  // 5 minutes
unsigned long collectionStartTime = 0;
bool collectingData = true;

// ============================================================================
// ESP-NOW CONTROL FUNCTIONS
// ============================================================================

void pauseESPNow() {
  // Unregister receive callback to stop processing incoming data
  esp_now_unregister_recv_cb();
  Serial.println("ESP-NOW paused");
}

void resumeESPNow() {
  // Re-register receive callback
  esp_now_register_recv_cb(ESPNowLogger::onDataReceived);
  Serial.println("ESP-NOW resumed");
}

// ============================================================================
// SD CARD HELPER FUNCTIONS
// ============================================================================

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  if (SERIAL_LOG_TEST) Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    // Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  if (SERIAL_LOG_TEST) Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    if (SERIAL_LOG_TEST) Serial.print(".");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

int read3File(fs::FS &fs, const char *path) {
  if (SERIAL_LOG_TEST) Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return -1;
  }
  char fileContent[10];
  int i = 0;
  while (file.available() && i < 4) {  //reads first 4 characters of a textfile
    fileContent[i] = file.read();
    i++;
  }
  file.close();
  return atoi(fileContent);  // return file digits as integer
}

// ============================================================================
// BUFFER TO CSV CONVERSION AND SD WRITE
// ============================================================================

void writeBufferToSD(char labels[MAX_DATA_VALUES][MAX_LABEL_LENGTH + 1]) {
  if (sampleCount == 0) {
    Serial.println("No samples to write");
    return;
  }
  
  Serial.printf("Converting %d samples to CSV and writing to SD...\n", sampleCount);
  
  // Write CSV header line using passed labels
  char headerLine[100];
  sprintf(headerLine, "time,%s,%s,%s,%s,%s,%s,%s,%s\n",
          labels[0], labels[1], labels[2],
          labels[3], labels[4], labels[5],
          labels[6], labels[7]);
  appendFile(SD, robotLog, headerLine);
  
  // Convert and write samples line by line
  char lineBuffer[100];
  for (int i = 0; i < sampleCount; i++) {
    sprintf(lineBuffer, "%.3f,%.1f,%.1f,%.1f,%.0f,%.0f,%.2f,%.0f,%.1f\n",
            sampleBuffer[i].timestamp,
            sampleBuffer[i].values[0], sampleBuffer[i].values[1],
            sampleBuffer[i].values[2], sampleBuffer[i].values[3],
            sampleBuffer[i].values[4], sampleBuffer[i].values[5],
            sampleBuffer[i].values[6], sampleBuffer[i].values[7]);
    appendFile(SD, robotLog, lineBuffer);
    
    // Progress indicator every 100 samples
    if ((i + 1) % 100 == 0) {
      Serial.print(".");
    }
  }
  Serial.printf("\nWrote %d samples to %s\n", sampleCount, robotLog);
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  
  // Initialize display
  tft.init();
  tft.setRotation(0);  // Portrait mode
  tft.fillScreen(COLOR_BACKGROUND);
  tft.setTextSize(TEXT_SIZE);

  // Display initial title (will be updated when header is received)
  tft.setTextColor(COLOR_TITLE);
  tft.drawString("Waiting for Master...", 20, TITLE_Y, 2);

  // Initialize the logger
  if (!logger.begin()) {
    tft.setTextColor(COLOR_CONNECTION);
    tft.drawString("ESP-NOW init failed!", 20, FIRST_ROW_Y, 2);
    while (1) {
      delay(1000);
    }
  }

  // Draw initial layout with placeholder values
  drawLayout();

  delay(2000);

  // ============================================================================
  // Initialize SD Card with proper SPI configuration
  // ============================================================================
  
  SPIClass spi = SPIClass(VSPI);

  if (!SD.begin(SS, spi, 8000000)) {
    Serial.println("Card Mount Failed");
    tft.setTextColor(COLOR_CONNECTION);
    tft.drawString("SD Card Failed!", 20, FIRST_ROW_Y + 20, 2);
    // Continue without SD logging
  } else {
    Serial.println("SD Card Mounted Successfully");
    
    // Testing SD write / appends ....
    char hlotxt[20] = "/hello.txt";
    writeFile(SD, "/hello.txt", "Hello ");
    appendFile(SD, hlotxt, "World!\n");

    LOG_NUM = read3File(SD, "/lognum.txt");
    LOG_NUM++;

    if (SD.exists("/lognum.txt")) {
      SD.remove("/lognum.txt");
    }

    sprintf(numString, "%04d", LOG_NUM);
    writeFile(SD, "/lognum.txt", numString);
    sprintf(robotLog, "/FN%04drobot.txt", LOG_NUM);
    
    char logHeader[50];
    sprintf(logHeader, "LogNum: %d\n", LOG_NUM);

    if (SD.exists(robotLog)) {
      SD.remove(robotLog);
    }
    writeFile(SD, robotLog, logHeader);
  }
  
  // Initialize collection timing
  collectionStartTime = millis();
  collectingData = true;
  sampleCount = 0;
  bufferFull = false;
  
  Serial.println("Setup complete - starting data collection");
  Serial.printf("Buffer can hold %d samples (%d bytes)\n", MAX_SAMPLES, sizeof(sampleBuffer));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long currentTime = millis();
  
  // Check if 5-minute collection period is over
  if (collectingData && (currentTime - collectionStartTime >= COLLECTION_DURATION_MS)) {
    Serial.println("\n5 minutes elapsed - pausing and writing to SD...");
    
    // Pause ESP-NOW reception
    pauseESPNow();
    collectingData = false;
    
    // Capture current labels right before writing
    char currentLabels[MAX_DATA_VALUES][MAX_LABEL_LENGTH + 1];
    for (int i = 0; i < MAX_DATA_VALUES; i++) {
      strncpy(currentLabels[i], logger.getLabel(i), MAX_LABEL_LENGTH);
      currentLabels[i][MAX_LABEL_LENGTH] = '\0';
    }
    
    // Convert buffer to CSV and write to SD
    writeBufferToSD(currentLabels);
    
    // Clear buffer and restart
    sampleCount = 0;
    bufferFull = false;
    collectionStartTime = currentTime;
    
    // Resume ESP-NOW
    resumeESPNow();
    collectingData = true;
    Serial.println("Collection resumed");
  }
  
  // Check for new header (just update display, don't store labels)
  if (logger.hasNewHeader()) {
    updateTitle();
    drawLabels();
  }
  
  // Check for new data
  if (logger.hasNewData() && collectingData) {
    lastUpdateTime = millis();
    updateValues();
    
    // Store data in binary format
    if (sampleCount < MAX_SAMPLES) {
      sampleBuffer[sampleCount].timestamp = logger.getLastTimestamp() / 1000.0;
      for (int i = 0; i < 8; i++) {
        sampleBuffer[sampleCount].values[i] = logger.getValue(i);
      }
      sampleCount++;
      
      // Optional: periodic status update
      if (sampleCount % 100 == 0) {
        Serial.printf("Collected %d samples (%.1f%% full)\n", 
                     sampleCount, (sampleCount * 100.0) / MAX_SAMPLES);
      }
    } else {
      if (!bufferFull) {
        Serial.println("WARNING: Sample buffer full!");
        bufferFull = true;
      }
    }
    
    if (!initialDataReceived) {
      initialDataReceived = true;
      clearDataArea();
    }
  }
  
  // Check connection status
  updateConnectionStatus();

  delay(10);  // Short delay to prevent loop from running too fast
}

// ============================================================================
// DISPLAY UPDATE FUNCTIONS
// ============================================================================

// Update the title if it has changed
void updateTitle() {
  const char *newTitle = logger.getTitle();

  // Check if title has changed
  if (strcmp(currentTitle, newTitle) != 0) {
    // Title has changed, update it

    // Clear title area
    tft.fillRect(0, TITLE_Y, 240, 20, COLOR_BACKGROUND);

    // Display new title
    tft.setTextColor(COLOR_TITLE);
    tft.drawString(newTitle, 20, TITLE_Y, 2);

    // Save current title
    strncpy(currentTitle, newTitle, MAX_TITLE_LENGTH);
    currentTitle[MAX_TITLE_LENGTH - 1] = '\0';  // Ensure null termination
  }
}

// Clear the entire data display area
void clearDataArea() {
  tft.fillRect(0, FIRST_ROW_Y, 240, 160, COLOR_BACKGROUND);

  // Redraw labels after clearing
  drawLabels();
}

// Update just the values
void updateValues() {
  tft.setTextColor(COLOR_VALUE, COLOR_BACKGROUND);

  // Update each telemetry value
  for (int i = 0; i < MAX_DATA_VALUES; i++) {
    float value = logger.getValue(i);
    int y = FIRST_ROW_Y + (i * ROW_HEIGHT);

    // Clear previous value
    tft.fillRect(VALUE_X, y, 120, 20, COLOR_BACKGROUND);

    // Display new value with 1 decimal place
    char valueStr[10];
    dtostrf(value, 5, 1, valueStr);
    tft.drawString(valueStr, VALUE_X, y, 2);
  }

  // Update timestamp
  uint32_t timestamp = logger.getLastTimestamp();
  updateTimestamp(timestamp);
}

// Draw just the labels
void drawLabels() {
  tft.setTextColor(COLOR_LABEL, COLOR_BACKGROUND);

  for (int i = 0; i < MAX_DATA_VALUES; i++) {
    int y = FIRST_ROW_Y + (i * ROW_HEIGHT);

    // Get label
    const char *label = logger.getLabel(i);

    // Clear previous label
    tft.fillRect(0, y, VALUE_X - 10, 20, COLOR_BACKGROUND);

    // Display label
    tft.drawString(label, 10, y, 2);
  }
}

// Draw the initial layout
void drawLayout() {
  // Draw initial labels
  drawLabels();

  // Draw timestamp area
  tft.drawLine(0, 190, 240, 190, TFT_DARKGREY);
  updateTimestamp(0);

  // Draw initial connection status
  updateConnectionStatus();
}

// Update the timestamp display
void updateTimestamp(uint32_t timestamp) {
  // Clear timestamp area
  tft.fillRect(0, 200, 240, 20, COLOR_BACKGROUND);

  // Format timestamp as seconds.milliseconds
  float seconds = timestamp / 1000.0;
  char timeStr[16];
  dtostrf(seconds, 6, 2, timeStr);

  // Display timestamp
  tft.setTextColor(COLOR_TIMESTAMP, COLOR_BACKGROUND);
  tft.drawString("Time: ", 5, 200, 1);
  tft.drawString(timeStr, 50, 200, 1);
  tft.drawString("s", 120, 200, 1);
}

// Update the connection status indicator
void updateConnectionStatus() {
  bool connected = logger.isConnected() && (millis() - lastUpdateTime < CONNECTION_TIMEOUT);

  // Clear status area
  tft.fillRect(200, 5, 30, 15, COLOR_BACKGROUND);

  // Draw status indicator
  if (connected) {
    tft.fillCircle(215, 10, 5, TFT_GREEN);
  } else {
    tft.fillCircle(215, 10, 5, COLOR_CONNECTION);
  }
}
