// Master Example - ESP32 QT PY Pico in Robot - With Serial1 Input from any other microcontroller
// This will receive data from another microcontroller and send it to the CYD display so you can check...
// Is It Rizzin ? (especially if your bot is named CheeseRizz)
// Cameron Stoker, Complified Creations, March 2025

//Data stream has 8 values (floats) and names (string) for those 8 floats on each line.

/*Serial1 data should be this format:
Label1,Value1,Label2,Value2,Label3,Value3,Label4,Value4,Label5,Value5,Label6,Value6,Label7,Value7,Label8,Value8\n
A QT-PY has Serial1 RX on pin 7, labeled RX

Example:
void setup() {
Serial1.begin(115200);
}

void loop() {
float Value1 = 1.0;
float Value2 = 2.0;
float Value3 = 3.0;
float Value4 = 4.0;
float Value5 = 5.0;
float Value6 = 6.0;
float Value7 = 7.0;
float Value8 = 8.0;

Serial1.printf("Label1,%f,Label2,%f,Label3,%f,Label4,%f,Label5,%f,Label6,%f,Label7,%f,Label8,%f\n",Value1,Value2,Value3,Value4,Value5,Value6,Value7,Value8);
delay(500); 
  }
*/

//

#include <Arduino.h>
#include "ESPNowLogger.h"
#include <Adafruit_NeoPixel.h>

// MAC address of the slave ESP32-2432S028R (CYD display) Adjust to match your hardware
uint8_t slaveAddress[] = { 0xF0, 0x24, 0xF9, 0x44, 0x6D, 0xF4 };

// Create the logger in master mode
ESPNowLogger logger(ESPNowLogger::MASTER);

// Set baud rate to match the sending microcontroller
#define SERIAL_BAUD_RATE 115200

// Buffer for receiving serial data
#define MAX_SERIAL_BUFFER 256
char serialBuffer[MAX_SERIAL_BUFFER];
int bufferPos = 0;

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
uint32_t color = pixels.Color(0, 0, 255); //blue


void setup() {
  // Initialize serial communication with USB port (for debugging, if needed)
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
  pixels.begin();                  // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setPixelColor(0, color);  // green when going,
  pixels.show();

  // Initialize Serial1 communication with the microcontroller
  Serial1.begin(SERIAL_BAUD_RATE);

  // Initialize the logger
  if (!logger.begin()) {
    while (1) {
      delay(1000);
    }
  }

  // Set the slave device
  if (!logger.setPeer(slaveAddress)) {
    while (1) {
      delay(1000);
    }
  }

  // Set custom data transmission intervals
  // Data every 250ms, header every 3 seconds
  logger.setIntervals(250, 3000);

  // Set default title for the display
  logger.setTitle("Robot Telemetry");

  // Set default labels (will be overwritten when received from Serial1)
  logger.setLabel(0, "Value1");
  logger.setLabel(1, "Value2");
  logger.setLabel(2, "Value3");
  logger.setLabel(3, "Value4");
  logger.setLabel(4, "Value5");
  logger.setLabel(5, "Value6");
  logger.setLabel(6, "Value7");
  logger.setLabel(7, "Value8");

  // Force an immediate header transmission
  logger.sendHeader();
  delay(100);

  // Force another header transmission after a delay to be sure
  delay(500);
  logger.sendHeader();
}

void loop() {
  // Check for serial data from microcontroller
  readSerialData();

  // Update the logger (sends data at configured intervals)
  logger.update();

  if (logger.isConnected() == 0) {
  color = pixels.Color(255, 0, 0); //red
  } else {
    color = pixels.Color(0, 255, 0); //green
  }
  pixels.setPixelColor(0, color);
  pixels.show();
  delay(10);  // Short delay to allow other processes
}

void readSerialData() {
  // Check for serial data from microcontroller
  while (Serial1.available() > 0) {
    char c = Serial1.read();

    // Line ending indicates complete data
    if (c == '\n' || c == '\r') {
      if (bufferPos > 0) {
        // Null terminate the string
        serialBuffer[bufferPos] = '\0';

        // Parse the data
        parseSerialData(serialBuffer);

        // Reset buffer position
        bufferPos = 0;
      }
    }
    // Add character to buffer if not full
    else if (bufferPos < MAX_SERIAL_BUFFER - 1) {
      serialBuffer[bufferPos++] = c;
    }
  }
}

void parseSerialData(char* data) {
  char* token;
  int index = 0;

  // Get first token (first label)
  token = strtok(data, ",");

  // Continue parsing label-value pairs
  while (token != NULL && index < MAX_DATA_VALUES) {
    // Set the label (trim whitespace if needed)
    char* trimmedLabel = token;
    while (*trimmedLabel == ' ') trimmedLabel++;  // Skip leading spaces

    logger.setLabel(index, trimmedLabel);

    // Get the value token
    token = strtok(NULL, ",");
    if (token != NULL) {
      // Trim whitespace if needed
      char* trimmedValue = token;
      while (*trimmedValue == ' ') trimmedValue++;  // Skip leading spaces

      // Convert to float and set the value
      float value = atof(trimmedValue);
      logger.setValue(index, value);

      // Move to next index
      index++;
    }

    // Get next label
    token = strtok(NULL, ",");
  }

  // Send header if labels have changed
  logger.sendHeader();
}