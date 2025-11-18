// Slave Example - ESP32-2432S028R (CYD) Display
//program as esp32 Dev Module
// This will receive and display telemetry data via ESPnow from the robot
// logging to SD card, master is usually set to send values 4hz, labels 1/3hz
// buffer of 10000 writes approx every 45sec.

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

#define SERIAL_LOG_TEST 0  // output to usb serial for diagnostics
#define BUFFER_SIZE 10000

int LOG_NUM = 1;
char numString[4];     // short string for loggin fn
char robotLog[20];     // numbered log file path name
char dataString[100];  // 65 should cover all labels, plus extra for commas
char buffer[BUFFER_SIZE];
bool newLogLine = false;  // track when line needs appending

int i = 0;  //ram buffer length indicator

SPIClass spiSD = SPIClass(VSPI);

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
  Serial.printf("Writing file: %s\n", path);

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
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    //Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

int read3File(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return -1;
  }
  char fileContent[10];
  int i = 0;
  while (file.available() && i < 4) {  //reads first 4 characters of a textfile, contents need to be 4 ascii digits
    fileContent[i] = file.read();
    i++;
  }
  file.close();
  return atoi(fileContent);  // return file digits as integer
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

  delay(5000);

  if (!SD.begin(SS, spiSD, 8000000)) {
    Serial.println("Card Mount Failed");
    return;
  }

  writeFile(SD, "/hello.txt", "Hello ");
  appendFile(SD, "/hello.txt", "World!\n");

  LOG_NUM = read3File(SD, "/lognum.txt");  // desired release height in ft in text file
  LOG_NUM++;

  if (SD.exists("/lognum.txt")) {  // remove so we can overwrite with new value
    SD.remove("/lognum.txt");
  }

  sprintf(numString, "%04d", LOG_NUM);
  writeFile(SD, "/lognum.txt", numString);         // store logfile index number
  sprintf(robotLog, "/FN%04drobot.txt", LOG_NUM);  // robotlog is log pathname
  sprintf(dataString, "LogNum: %d\n", LOG_NUM);
  writeFile(SD, robotLog, dataString);  // header for file
}

void loop() {
  bool needsRefresh = false;

  // Check for new header
  if (logger.hasNewHeader()) {
    // Update title if changed
    updateTitle();

    // Update labels
    drawLabels();
    needsRefresh = true;
  }

  // Check for new data
  if (logger.hasNewData()) {
    lastUpdateTime = millis();
    updateValues();
    needsRefresh = true;
    sprintf(dataString, "%4.3f,%4.1f,%4.1f,%4.1f,%4.0f,%4.0f,%3.2f,%4.0f,%4.1f\n", logger.getLastTimestamp() / 1000.0, logger.getValue(0), logger.getValue(1), logger.getValue(2), logger.getValue(3), logger.getValue(4), logger.getValue(5), logger.getValue(6), logger.getValue(7));
    newLogLine = true;

    if (!initialDataReceived) {
      initialDataReceived = true;
      // We got our first data, clear the entire data area to be sure
      clearDataArea();
    }
  }
  if (newLogLine) {
    newLogLine = false;
    if (SERIAL_LOG_TEST) {
      //Serial.print(i);
      //Serial.print(" ");
      Serial.print(dataString);
      if (i <= 0.9 * BUFFER_SIZE) {  // add lines to buffer untill it's 90% full
        i += sprintf(buffer + i, dataString);
        dataString[0] = '\0';
      }

      if (i > 0.9 * BUFFER_SIZE) {  // write headers and buffer to SD card when more than 90% full
        //logger.pauseReception();  // not needed
        i += sprintf(buffer + i, dataString);  //
        sprintf(dataString, "time,%s,%s,%s,%s,%s,%s,%s,%s\n", logger.getLabel(0), logger.getLabel(1), logger.getLabel(2), logger.getLabel(3), logger.getLabel(4), logger.getLabel(5), logger.getLabel(6), logger.getLabel(7));
        i += sprintf(buffer + i, dataString);
        dataString[0] = '\0';
        appendFile(SD, robotLog, buffer);
        i = 0;
        buffer[0] = '\0';  // set terminator to beginning, wiping out buffer
        //logger.resumeReception();  // not needed
      }
    }

    // Check connection status
    updateConnectionStatus();

    delay(10);  // Short delay to prevent loop from running too fast
  }
}
