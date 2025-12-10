// ESP-NOW Logger for Telemetry - With Title Support
// A lightweight library for sending telemetry data from one ESP32 to another

#ifndef ESP_NOW_LOGGER_H
#define ESP_NOW_LOGGER_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Configuration constants
#define MAX_DATA_VALUES 8
#define MAX_LABEL_LENGTH 8
#define MAX_TITLE_LENGTH 32
#define DEFAULT_DATA_INTERVAL_MS 250
#define DEFAULT_HEADER_INTERVAL_MS 10000

// Message Type for clear identification
enum MessageType {
  DATA_MESSAGE = 0,
  HEADER_MESSAGE = 1
};

// Data structures for ESP-NOW communication
struct LogEntry {
  uint8_t msgType;               // Use first byte for message type (0 = data)
  uint32_t timestamp;            // Milliseconds since boot
  float values[MAX_DATA_VALUES]; // Telemetry data values
};

struct HeaderEntry {
  uint8_t msgType;               // Use first byte for message type (1 = header)
  uint32_t timestamp;            // Milliseconds since boot
  char title[MAX_TITLE_LENGTH];  // Title for the display
  char labels[MAX_DATA_VALUES][MAX_LABEL_LENGTH + 1]; // Labels for each value (+1 for null terminator)
};

class ESPNowLogger {
public:
  // Initialize as master (sender) or slave (receiver)
  enum Mode { MASTER, SLAVE };
  
  // Constructor
  ESPNowLogger(Mode mode = MASTER);
  
  // Initialize the ESP-NOW connection
  bool begin();
  
  // For master: Set the peer MAC address (slave device)
  bool setPeer(const uint8_t* mac);
  
  // For master: Set the transmission intervals
  void setIntervals(uint32_t dataIntervalMs = DEFAULT_DATA_INTERVAL_MS, 
                   uint32_t headerIntervalMs = DEFAULT_HEADER_INTERVAL_MS);
  
  // For master: Set the telemetry values
  void setValue(uint8_t index, float value);
  
  // For master: Set a label for a value
  void setLabel(uint8_t index, const char* label);
  
  // For master: Set the title for the display
  void setTitle(const char* title);
  
  // For master: Update and send data if needed (call in main loop)
  void update();
  
  // For master: Force sending a header packet
  void sendHeader();
  
  // For slave: Get the last received value
  float getValue(uint8_t index);
  
  // For slave: Get the label for a value
  const char* getLabel(uint8_t index);
  
  // For slave: Get the title
  const char* getTitle();
  
  // For slave: Get the timestamp of the last received data
  uint32_t getLastTimestamp();
  
  // For slave: Check if new data has been received since last check
  bool hasNewData();
  
  // For slave: Check if new header has been received
  bool hasNewHeader();
  
  // For both: Get connection status
  bool isConnected();
  
  // For slave: Check if header has ever been received
  bool headerReceived();
  
  // For slave: Pause reception (unregister callback)
  void pauseReception();
  
  // For slave: Resume reception (re-register callback)
  void resumeReception();
  
  // For slave: Check if reception is paused
  bool isReceptionPaused();

private:
  Mode _mode;
  uint8_t _peerMac[6];
  bool _peerConfigured;
  bool _connected;
  
  // Transmission timing
  uint32_t _dataIntervalMs;
  uint32_t _headerIntervalMs;
  uint32_t _lastDataSent;
  uint32_t _lastHeaderSent;
  
  // Data storage
  LogEntry _dataPacket;
  HeaderEntry _headerPacket;
  
  // For slave: Reception state
  bool _newDataReceived;
  bool _newHeaderReceived;
  uint32_t _lastReceivedTimestamp;
  bool _receptionPaused;
  
  // ESP-NOW callbacks
  static void onDataSent(const uint8_t* mac, esp_now_send_status_t status);
  static void onDataReceived(const uint8_t* mac, const uint8_t* data, int len);
  
  // Reference to the instance for callbacks
  static ESPNowLogger* _instance;
  static bool _headerReceived;
  
  // Internal methods
  void sendData();
};

#endif // ESP_NOW_LOGGER_H
