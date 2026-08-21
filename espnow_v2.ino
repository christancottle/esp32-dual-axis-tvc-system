#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <LittleFS.h>

// --- HARDWARE PIN DEFINITIONS ---
#define SERVO_PITCH_PIN 18 // Header J2
#define SERVO_YAW_PIN   19 // Header J3
#define SERVO_CENTER    90
#define SERVO_MAX_DEF   30

// --- GROUND STATION MAC ADDRESS ---
uint8_t broadcastAddress[] = {0xD4, 0xE9, 0xF4, 0xE1, 0xEC, 0xC0};

typedef struct TelemetryPacket {
    float pitch;
    float yaw;
    float altitude;
    uint8_t flightState; // 0=IDLE, 1=ARMED, 2=FLIGHT, 3=DESCENT, 4=LANDED
} TelemetryPacket;

TelemetryPacket txData;
esp_now_peer_info_t peerInfo;

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
File logFile;

Servo pitchServo;
Servo yawServo;

unsigned long lastTxTime = 0;
unsigned long stateTimer = 0; 
float basePressure = 1013.25;
float maxAltitude = 0.0f;
float currentAltitude = 0.0f;
float filteredYaw = 0.0f; // EMA Filter memory for Yaw axis
bool bmpAvailable = false;

// Updated callback signature for ESP32 Core 3.3.8 / 3.x
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // ESP-NOW Send Callback
}

// Full System & Altitude Baseline Reset
void resetSystem() {
  if (bmpAvailable) {
    float totalPressure = 0;
    for (int i = 0; i < 10; i++) {
      totalPressure += bmp.readPressure() / 100.0F;
      delay(10);
    }
    basePressure = totalPressure / 10.0F;
  }
  
  maxAltitude = 0.0f;
  currentAltitude = 0.0f;
  filteredYaw = 0.0f;
  txData.altitude = 0.0f;
  txData.flightState = 1; // ARMED mode
  stateTimer = millis();

  // Attach and center BOTH servos on reset
  pitchServo.write(SERVO_CENTER);
  yawServo.write(SERVO_CENTER);

  // LittleFS Log Initialization
  logFile = LittleFS.open("/flight_log.csv", FILE_WRITE);
  if (logFile) {
    logFile.println("Time_ms,State,Pitch_deg,Yaw_deg,Alt_m,AccMag");
    logFile.flush(); // Ensure header is written immediately
  }

  Serial.println("\n*** BENCH RESET COMPLETE: SERVOS CENTERED & ALTITUDE ZEROED ***\n");
}

void dumpFlashData() {
  Serial.println("\n--- DUMPING TELEMETRY LOG FROM LITTLEFS FLASH ---");
  
  // Close write buffer before reading to prevent file locks
  if (logFile) {
    logFile.flush();
    logFile.close();
  }

  File file = LittleFS.open("/flight_log.csv", FILE_READ);
  if (!file) {
    Serial.println("❌ Failed to open /flight_log.csv");
    return;
  }
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
  Serial.println("--- END OF FLIGHT LOG DUMP ---\n");

  // Re-open in append mode for future operations
  logFile = LittleFS.open("/flight_log.csv", FILE_APPEND);
}

void setup() {
  Serial.begin(115200);

  // 1. Initialize LittleFS Flash Storage
  if (!LittleFS.begin(true)) {
    Serial.println("❌ LittleFS Storage Mount Failed!");
  } else {
    Serial.println("✅ LittleFS Storage Online.");
  }

  // 2. Optimized Servo Timer Allocation (Shared Timer 0 to eliminate jitter)
  ESP32PWM::allocateTimer(0);
  pitchServo.setPeriodHertz(50);
  yawServo.setPeriodHertz(50);
  
  pitchServo.attach(SERVO_PITCH_PIN, 500, 2400);
  yawServo.attach(SERVO_YAW_PIN, 500, 2400);

  pitchServo.write(SERVO_CENTER);
  yawServo.write(SERVO_CENTER);

  // 3. Initialize I2C Peripherals
  Wire.begin(21, 22);

  if (!mpu.begin()) {
    Serial.println("Warning: MPU6050 not detected!");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.println("✅ MPU6050 Online!");
  }

  if (bmp.begin(0x76) || bmp.begin(0x77)) {
    bmpAvailable = true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                    Adafruit_BMP280::SAMPLING_X2,     
                    Adafruit_BMP280::SAMPLING_X16,    
                    Adafruit_BMP280::FILTER_X16,      
                    Adafruit_BMP280::STANDBY_MS_1);   
    Serial.println("✅ BMP280 Online!");
  } else {
    Serial.println("Warning: BMP280 not detected!");
  }

  // 4. Initialize ESP-NOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  resetSystem();
}

void loop() {
  // Serial command parser
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("reset")) {
      resetSystem();
    } else if (input.equalsIgnoreCase("dump")) {
      dumpFlashData();
    }
  }

  // Read MPU-6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Pitch calculation
  txData.pitch = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / M_PI;

  // Yaw calculation with EMA Low-Pass Filter (0.15 alpha) to eliminate motor jitter
  float rawYaw = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / M_PI;
  filteredYaw = (0.15f * rawYaw) + (0.85f * filteredYaw);
  txData.yaw = filteredYaw;

  // Read BMP280
  if (bmpAvailable) {
    float rawAltitude = bmp.readAltitude(basePressure);

    if (rawAltitude > 2000.0 || isnan(rawAltitude) || rawAltitude < -50.0f) {
      Wire.begin(21, 22);
      bmp.begin(0x76);
      delay(5);
      rawAltitude = bmp.readAltitude(basePressure);
    }

    currentAltitude = (0.80f * currentAltitude) + (0.20f * rawAltitude);
    if (currentAltitude < 0.0f) currentAltitude = 0.0f;
    txData.altitude = currentAltitude;
  }

  // Calculate Total Acceleration Vector Magnitude
  float accelMag = sqrt(a.acceleration.x * a.acceleration.x + 
                        a.acceleration.y * a.acceleration.y + 
                        a.acceleration.z * a.acceleration.z);

  // --- DUAL-AXIS ACTIVE TVC SERVO CONTROL ---
  if (txData.flightState == 1 || txData.flightState == 2) { 
    // Active Stabilization during ARMED & FLIGHT
    int pCmd = constrain(SERVO_CENTER - (int)txData.pitch, SERVO_CENTER - SERVO_MAX_DEF, SERVO_CENTER + SERVO_MAX_DEF);
    int yCmd = constrain(SERVO_CENTER + (int)txData.yaw,   SERVO_CENTER - SERVO_MAX_DEF, SERVO_CENTER + SERVO_MAX_DEF);

    pitchServo.write(pCmd);
    yawServo.write(yCmd);
  } else {
    // Lock Servos Neutral in IDLE, DESCENT, or LANDED
    pitchServo.write(SERVO_CENTER);
    yawServo.write(SERVO_CENTER);
  }

  // --- REVISED FLIGHT STATE MACHINE ---
  switch (txData.flightState) {
    case 0: // IDLE
      break;

    case 1: // ARMED
      if (accelMag > 13.0 || txData.altitude > 0.30f) { 
        txData.flightState = 2; // FLIGHT
        maxAltitude = txData.altitude;
        stateTimer = millis();
        Serial.println(">>> LAUNCH DETECTED! SWITCHING TO FLIGHT MODE <<<");
      }
      break;

    case 2: // FLIGHT / ASCENT
      if (txData.altitude > maxAltitude) {
        maxAltitude = txData.altitude;
        stateTimer = millis(); // Reset timer while ascending
      }

      // Robust bench test apogee criteria: >1.8s peak dwell time AND >0.5m altitude reached
      if (millis() - stateTimer > 1800 && txData.altitude > 0.30f) {
        txData.flightState = 3; // DESCENT
        stateTimer = millis();
        Serial.println(">>> APOGEE DETECTED! ENTERING DESCENT MODE <<<");
      }
      break;

    case 3: // DESCENT
      if (txData.altitude < 0.20f) {
        if (millis() - stateTimer > 1000) {
          txData.flightState = 4; // LANDED
          stateTimer = millis();
          Serial.println(">>> TOUCHDOWN DETECTED! VEHICLE LANDED SAFELY. DATA READY FOR 'dump' <<<");
        }
      } else {
        stateTimer = millis();
      }
      break;

    case 4: // LANDED
      // Hold state safely without auto-resetting
      break;
  }

  // Telemetry Logging with Immediate Flash Flushing
  if (logFile && (txData.flightState == 2 || txData.flightState == 3)) {
    logFile.printf("%lu,%d,%.2f,%.2f,%.2f,%.2f\n", 
                    millis(), txData.flightState, txData.pitch, txData.yaw, txData.altitude, accelMag);
    logFile.flush(); // Critical: Commit buffer directly to physical flash
  }

  // Wireless ESP-NOW Telemetry Transmission (5 Hz)
  if (millis() - lastTxTime >= 200) {
    lastTxTime = millis();
    esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));
  }
}