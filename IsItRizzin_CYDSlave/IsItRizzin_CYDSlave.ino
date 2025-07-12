// Slave Example - ESP32-2432S028R (CYD) Display 
// This will receive and display telemetry data via ESPnow from the robot

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "ESPNowLogger.h"

// Create the TFT display object
TFT_eSPI tft = TFT_eSPI();

// Create the logger in slave mode
ESPNowLogger logger(ESPNowLogger::SLAVE);

// Layout constants
#define TEXT_SIZE 1       // Small text
#define VALUE_X 120      // Position for values
#define ROW_HEIGHT 20    // Height per data row
#define TITLE_Y 5        // Title position
#define FIRST_ROW_Y 30   // First data row position

// Colors
#define COLOR_TITLE TFT_WHITE
#define COLOR_LABEL TFT_YELLOW
#define COLOR_VALUE TFT_GREEN
#define COLOR_BACKGROUND TFT_NAVY
#define COLOR_TIMESTAMP TFT_SILVER
#define COLOR_CONNECTION TFT_RED

// For determining connection status
unsigned long lastUpdateTime = 0;
const unsigned long CONNECTION_TIMEOUT = 2000; // 2 seconds without data means disconnected
bool initialDataReceived = false;

// Store current title to detect changes
char currentTitle[MAX_TITLE_LENGTH] = "";

void setup() {
  // Initialize display
  tft.init();
  tft.setRotation(0); // Portrait mode
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
    
    if (!initialDataReceived) {
      initialDataReceived = true;
      // We got our first data, clear the entire data area to be sure
      clearDataArea();
    }
  }
  
  // Check connection status
  updateConnectionStatus();
  
  delay(10); // Short delay to prevent loop from running too fast
}

// Update the title if it has changed
void updateTitle() {
  const char* newTitle = logger.getTitle();
  
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
    currentTitle[MAX_TITLE_LENGTH - 1] = '\0'; // Ensure null termination
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
    const char* label = logger.getLabel(i);
    
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
  bool connected = logger.isConnected() && 
                  (millis() - lastUpdateTime < CONNECTION_TIMEOUT);
  
  // Clear status area
  tft.fillRect(200, 5, 30, 15, COLOR_BACKGROUND);
  
  // Draw status indicator
  if (connected) {
    tft.fillCircle(215, 10, 5, TFT_GREEN);
  } else {
    tft.fillCircle(215, 10, 5, COLOR_CONNECTION);
  }
}