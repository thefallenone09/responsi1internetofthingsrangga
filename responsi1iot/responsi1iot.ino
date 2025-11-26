/* ============================================================
   PROJECT: IoT Flame + Rain Sensor (WeMos D1 ESP8266)
   Fixed Version - Correct sensor reading & no false detection
   ============================================================ */

#include <ESP8266WiFi.h>

// WiFi Configuration
const char* WIFI_SSID = "Infinix HOT 60 Pro";
const char* WIFI_PASS = "123456788";

// Sensor pins
const int PIN_FLAME = D1; // GPIO5
const int PIN_RAIN  = D2; // GPIO4

// Sensor configuration - SESUAIKAN DENGAN SENSOR ANDA
const bool FLAME_ACTIVE_LOW = true;  // true jika sensor LOW saat ada api
const bool RAIN_ACTIVE_LOW  = true;  // true jika sensor LOW saat hujan

// Sampling configuration
const int SAMPLES = 10;  // Increased for better stability
const int SAMPLE_DELAY_MS = 10;
const int STABLE_REQUIRED_LOOPS = 5;  // Increased for more stability
const unsigned long LOOP_DELAY_MS = 500;
const unsigned long FLAME_PRIORITY_MS = 5000;

// WiFi configuration
const unsigned long WIFI_TIMEOUT = 30000;
const unsigned long WIFI_RETRY_INTERVAL = 10000;
const unsigned long WIFI_STATUS_INTERVAL = 30000;

// Global variables
int flameCounter = 0;
int rainCounter  = 0;
bool stableFlame = false;
bool stableRain  = false;
bool lastFlameState = false;
bool lastRainState = false;
unsigned long flamePriorityUntil = 0;
unsigned long lastWifiRetry = 0;
unsigned long lastWifiStatus = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastRawPrint = 0;

/**
 * Improved digital read with majority voting and noise filtering
 */
bool majorityReadDigital(int pin) {
  int ones = 0;
  for (int i = 0; i < SAMPLES; i++) {
    ones += digitalRead(pin);
    delay(SAMPLE_DELAY_MS);
  }
  return (ones > SAMPLES / 2);
}

/**
 * Convert raw reading to logical state based on active low/high configuration
 */
bool interpret(bool raw, bool activeLow) {
  return activeLow ? (!raw) : raw;
}

/**
 * Print raw sensor values for debugging
 */
void printRawValues() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastRawPrint >= 3000) {
    bool rawFlame = digitalRead(PIN_FLAME);
    bool rawRain = digitalRead(PIN_RAIN);
    
    Serial.printf("[DEBUG] Raw - Flame: %d, Rain: %d | Interpreted - Flame: %d, Rain: %d\n",
                 rawFlame, rawRain,
                 interpret(rawFlame, FLAME_ACTIVE_LOW),
                 interpret(rawRain, RAIN_ACTIVE_LOW));
    lastRawPrint = currentMillis;
  }
}

/**
 * Non-blocking WiFi connection with timeout
 */
void wifiConnect() {
  if (WiFi.status() == WL_CONNECTED) return;
  
  Serial.printf("Connecting to WiFi '%s'...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWifiRetry = millis();
}

/**
 * Handle WiFi reconnection and status monitoring
 */
void handleWiFi() {
  unsigned long currentMillis = millis();
  
  switch (WiFi.status()) {
    case WL_CONNECTED:
      if (currentMillis - lastWifiStatus >= WIFI_STATUS_INTERVAL) {
        Serial.printf("[WiFi] Connected, IP: %s, RSSI: %d dBm\n", 
                     WiFi.localIP().toString().c_str(), WiFi.RSSI());
        lastWifiStatus = currentMillis;
      }
      break;
      
    case WL_CONNECTION_LOST:
    case WL_DISCONNECTED:
      if (currentMillis - lastWifiRetry >= WIFI_RETRY_INTERVAL) {
        Serial.println("[WiFi] Connection lost. Reconnecting...");
        WiFi.disconnect();
        delay(1000);
        wifiConnect();
      }
      break;
      
    default:
      if (currentMillis - lastWifiRetry >= WIFI_RETRY_INTERVAL) {
        Serial.println("[WiFi] Not connected. Retrying...");
        WiFi.disconnect();
        delay(1000);
        wifiConnect();
      }
      break;
  }
}

/**
 * Initialize sensors and read initial state
 */
void initializeSensors() {
  Serial.println("Calibrating sensors...");
  
  // Read multiple times to get stable initial state
  bool initialFlame = false;
  bool initialRain = false;
  
  for (int i = 0; i < 10; i++) {
    initialFlame = majorityReadDigital(PIN_FLAME);
    initialRain = majorityReadDigital(PIN_RAIN);
    delay(100);
  }
  
  stableFlame = interpret(initialFlame, FLAME_ACTIVE_LOW);
  stableRain = interpret(initialRain, RAIN_ACTIVE_LOW);
  
  // Reset counters based on initial state
  flameCounter = STABLE_REQUIRED_LOOPS;
  rainCounter = STABLE_REQUIRED_LOOPS;

  Serial.println("Sensor calibration complete:");
  Serial.printf("  Raw Flame: %d -> Interpreted: %s\n", initialFlame, stableFlame ? "FIRE" : "SAFE");
  Serial.printf("  Raw Rain:  %d -> Interpreted: %s\n", initialRain, stableRain ? "RAIN" : "DRY");
  Serial.printf("  Active Low - Flame: %s, Rain: %s\n", FLAME_ACTIVE_LOW ? "YES" : "NO", RAIN_ACTIVE_LOW ? "YES" : "NO");
}

/**
 * Handle flame sensor state changes with debouncing
 */
void handleFlameSensor(bool currentState) {
  // Debounce logic - hanya update jika state berbeda
  if (currentState == stableFlame) {
    flameCounter = min(flameCounter + 1, STABLE_REQUIRED_LOOPS * 2);
  } else {
    flameCounter = max(flameCounter - 1, 0);
  }

  // State change detection - hanya jika counter mencapai threshold
  if (flameCounter >= STABLE_REQUIRED_LOOPS && currentState != stableFlame) {
    stableFlame = currentState;
    
    if (stableFlame) {
      Serial.println("🚨🚨🚨 ALERT: FIRE DETECTED! 🚨🚨🚨");
      flamePriorityUntil = millis() + FLAME_PRIORITY_MS;
    } else {
      Serial.println("✅ Fire cleared - area SAFE");
    }
    
    // Reset counter setelah state change
    flameCounter = STABLE_REQUIRED_LOOPS;
  }
}

/**
 * Handle rain sensor state changes with flame priority
 */
void handleRainSensor(bool currentState) {
  // Debounce logic
  if (currentState == stableRain) {
    rainCounter = min(rainCounter + 1, STABLE_REQUIRED_LOOPS * 2);
  } else {
    rainCounter = max(rainCounter - 1, 0);
  }

  // State change detection with flame priority
  if (rainCounter >= STABLE_REQUIRED_LOOPS && currentState != stableRain) {
    bool flamePriorityActive = (millis() < flamePriorityUntil && stableFlame);
    
    if (currentState && flamePriorityActive) {
      Serial.println("[INFO] Rain detection suppressed (flame priority)");
      rainCounter = STABLE_REQUIRED_LOOPS; // Reset counter
    } else {
      stableRain = currentState;
      Serial.printf("[INFO] %s\n", stableRain ? "🌧️ RAIN detected" : "☀️ Rain stopped - DRY");
      rainCounter = STABLE_REQUIRED_LOOPS; // Reset counter
    }
  }
}

/**
 * Print status summary with rate limiting
 */
void printStatus() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastStatusPrint >= 5000) {
    Serial.printf("STATUS: Flame=%s | Rain=%s | WiFi=%s\n",
                 stableFlame ? "🔥 FIRE" : "✅ SAFE",
                 stableRain ? "🌧️ RAINING" : "☀️ DRY",
                 WiFi.status() == WL_CONNECTED ? "📶 Connected" : "❌ Disconnected");
    lastStatusPrint = currentMillis;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for stable boot
  
  pinMode(PIN_FLAME, INPUT);
  pinMode(PIN_RAIN, INPUT);

  Serial.println("\n=== IoT Flame & Rain Detection System ===");
  Serial.println("System starting...");
  
  initializeSensors();
  wifiConnect();
  
  Serial.println("System ready! Monitoring sensors...");
}

void loop() {
  handleWiFi();
  
  // Read sensors
  bool rawFlame = majorityReadDigital(PIN_FLAME);
  bool rawRain  = majorityReadDigital(PIN_RAIN);
  
  bool currentFlame = interpret(rawFlame, FLAME_ACTIVE_LOW);
  bool currentRain  = interpret(rawRain, RAIN_ACTIVE_LOW);
  
  // Process sensor data
  handleFlameSensor(currentFlame);
  handleRainSensor(currentRain);
  
  // Debug info
  printRawValues();
  printStatus();
  
  delay(LOOP_DELAY_MS);
}