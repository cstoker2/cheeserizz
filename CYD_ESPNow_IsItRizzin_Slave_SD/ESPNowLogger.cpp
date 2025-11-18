// ESP-NOW Logger Implementation - With Title Support

#include "ESPNowLogger.h"

// Static instance pointer for callback functions
ESPNowLogger* ESPNowLogger::_instance = nullptr;
bool ESPNowLogger::_headerReceived = false;

// Constructor
ESPNowLogger::ESPNowLogger(Mode mode) {
  _mode = mode;
  _peerConfigured = false;
  _connected = false;
  _newDataReceived = false;
  _newHeaderReceived = false;
  _lastReceivedTimestamp = 0;
  _dataIntervalMs = DEFAULT_DATA_INTERVAL_MS;
  _headerIntervalMs = DEFAULT_HEADER_INTERVAL_MS;
  _lastDataSent = 0;
  _lastHeaderSent = 0;
  _receptionPaused = false;
  
  // Initialize data structures
  memset(&_dataPacket, 0, sizeof(LogEntry));
  memset(&_headerPacket, 0, sizeof(HeaderEntry));
  
  // Set message types
  _dataPacket.msgType = DATA_MESSAGE;
  _headerPacket.msgType = HEADER_MESSAGE;
  
  // Set default title
  strncpy(_headerPacket.title, "Telemetry Data", MAX_TITLE_LENGTH - 1);
  _headerPacket.title[MAX_TITLE_LENGTH - 1] = '\0';
  
  // Set default labels
  for (int i = 0; i < MAX_DATA_VALUES; i++) {
    snprintf(_headerPacket.labels[i], MAX_LABEL_LENGTH + 1, "Val%d", i);
  }
  
  // Set static instance for callbacks
  _instance = this;
}

// Initialize ESP-NOW
bool ESPNowLogger::begin() {
  // Set device as WiFi Station
  WiFi.mode(WIFI_STA);
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    return false;
  }
  
  // Register callbacks based on mode
  if (_mode == MASTER) {
    esp_now_register_send_cb(onDataSent);
  } else {
    esp_now_register_recv_cb(onDataReceived);
  }
  
  return true;
}

// Set peer MAC address (for master)
bool ESPNowLogger::setPeer(const uint8_t* mac) {
  memcpy(_peerMac, mac, 6);
  
  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, _peerMac, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    return false;
  }
  
  _peerConfigured = true;
  return true;
}

// Set transmission intervals
void ESPNowLogger::setIntervals(uint32_t dataIntervalMs, uint32_t headerIntervalMs) {
  _dataIntervalMs = dataIntervalMs;
  _headerIntervalMs = headerIntervalMs;
}

// Set a telemetry value
void ESPNowLogger::setValue(uint8_t index, float value) {
  if (index < MAX_DATA_VALUES) {
    _dataPacket.values[index] = value;
  }
}

// Set a label for a telemetry value
void ESPNowLogger::setLabel(uint8_t index, const char* label) {
  if (index < MAX_DATA_VALUES) {
    strncpy(_headerPacket.labels[index], label, MAX_LABEL_LENGTH);
    _headerPacket.labels[index][MAX_LABEL_LENGTH] = '\0';
  }
}

// Set the title for the display
void ESPNowLogger::setTitle(const char* title) {
  strncpy(_headerPacket.title, title, MAX_TITLE_LENGTH - 1);
  _headerPacket.title[MAX_TITLE_LENGTH - 1] = '\0';
}

// Update function - call in main loop
void ESPNowLogger::update() {
  if (_mode != MASTER || !_peerConfigured) {
    return;
  }
  
  uint32_t currentTime = millis();
  
  // Send header at regular intervals
  if (currentTime - _lastHeaderSent >= _headerIntervalMs) {
    sendHeader();
    _lastHeaderSent = currentTime;
  }
  
  // Send data at regular intervals
  if (currentTime - _lastDataSent >= _dataIntervalMs) {
    sendData();
    _lastDataSent = currentTime;
  }
}

// Send telemetry data
void ESPNowLogger::sendData() {
  _dataPacket.timestamp = millis();
  
  esp_err_t result = esp_now_send(_peerMac, (uint8_t*)&_dataPacket, sizeof(LogEntry));
  
  if (result != ESP_OK) {
    _connected = false;
  }
}

// Send header with labels
void ESPNowLogger::sendHeader() {
  _headerPacket.timestamp = millis();
  
  esp_err_t result = esp_now_send(_peerMac, (uint8_t*)&_headerPacket, sizeof(HeaderEntry));
  
  if (result != ESP_OK) {
    _connected = false;
  }
}

// Get a telemetry value (for slave)
float ESPNowLogger::getValue(uint8_t index) {
  if (index < MAX_DATA_VALUES) {
    return _dataPacket.values[index];
  }
  return 0.0f;
}

// Get a label (for slave)
const char* ESPNowLogger::getLabel(uint8_t index) {
  if (index < MAX_DATA_VALUES) {
    return _headerPacket.labels[index];
  }
  return "";
}

// Get the title (for slave)
const char* ESPNowLogger::getTitle() {
  return _headerPacket.title;
}

// Get the timestamp of the last received data
uint32_t ESPNowLogger::getLastTimestamp() {
  return _lastReceivedTimestamp;
}

// Check if new data has been received
bool ESPNowLogger::hasNewData() {
  bool result = _newDataReceived;
  _newDataReceived = false;
  return result;
}

// Check if new header has been received
bool ESPNowLogger::hasNewHeader() {
  bool result = _newHeaderReceived;
  _newHeaderReceived = false;
  return result;
}

// Get connection status
bool ESPNowLogger::isConnected() {
  return _connected;
}

// Check if we've ever received a header
bool ESPNowLogger::headerReceived() {
  return _headerReceived;
}

// Pause reception (for slave)
void ESPNowLogger::pauseReception() {
  if (_mode != SLAVE) {
    return;
  }
  
  if (_receptionPaused) {
    return;
  }
  
  esp_now_unregister_recv_cb();
  _receptionPaused = true;
  _connected = false;
}

// Resume reception (for slave)
void ESPNowLogger::resumeReception() {
  if (_mode != SLAVE) {
    return;
  }
  
  if (!_receptionPaused) {
    return;
  }
  
  esp_now_register_recv_cb(onDataReceived);
  _receptionPaused = false;
}

// Check if reception is paused
bool ESPNowLogger::isReceptionPaused() {
  return _receptionPaused;
}

// ESP-NOW callback when data is sent
void ESPNowLogger::onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  if (_instance) {
    _instance->_connected = (status == ESP_NOW_SEND_SUCCESS);
  }
}

// ESP-NOW callback when data is received
void ESPNowLogger::onDataReceived(const uint8_t* mac, const uint8_t* data, int len) {
  if (!_instance) return;
  
  // Check if we have at least one byte to read message type
  if (len < 1) {
    return;
  }
  
  // Get the message type
  uint8_t msgType = data[0];
  
  if (msgType == HEADER_MESSAGE) {
    // Process header message
    if (len == sizeof(HeaderEntry)) {
      memcpy(&_instance->_headerPacket, data, sizeof(HeaderEntry));
      _instance->_newHeaderReceived = true;
      _headerReceived = true;
    }
  } else if (msgType == DATA_MESSAGE) {
    // Process data message
    if (len == sizeof(LogEntry)) {
      memcpy(&_instance->_dataPacket, data, sizeof(LogEntry));
      _instance->_lastReceivedTimestamp = _instance->_dataPacket.timestamp;
      _instance->_newDataReceived = true;
    }
  }
  
  _instance->_connected = true;
}
