#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// --- HARDWARE PIN DEFINITIONS ---
#define GREEN_LED_PIN 16
#define RED_LED_PIN   17
#define BUZZER_PIN    25
#define BUTTON_PIN    13

// LCD Initialization (Default Address: 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Telemetry Data Structure - MUST MATCH FLIGHT COMPUTER
typedef struct TelemetryPacket {
    float pitch;
    float yaw;
    float altitude;
    uint8_t flightState; // 0=IDLE, 1=ARMED, 2=FLIGHT, 3=DESCENT
} TelemetryPacket;

TelemetryPacket rxData;
const char* stateNames[] = {"IDLE", "ARM ", "FLY ", "DESC"};

unsigned long lastPacketTime = 0;
uint8_t lastState = 255;
bool greenLedState = false;
bool newPacketReceived = false;

// Audio alert helper function
void beep(int frequency, int durationMs) {
    tone(BUZZER_PIN, frequency, durationMs);
    delay(durationMs);
    noTone(BUZZER_PIN);
}

// ESP-NOW Receive Callback
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    memcpy(&rxData, incomingData, sizeof(rxData));
    lastPacketTime = millis();
    newPacketReceived = true;
}

void setup() {
    Serial.begin(115200);

    // Initialize Pins
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Startup Tones
    beep(1000, 100);
    delay(100);
    beep(1500, 150);

    // Initialize LCD
    Wire.begin(21, 22);
    lcd.init();
    lcd.backlight();
    
    lcd.setCursor(0, 0);
    lcd.print("GND STATION v3.0");
    lcd.setCursor(0, 1);
    lcd.print("AWAITING ESP-NOW");

    // Initialize ESP-NOW
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        lcd.setCursor(0, 1);
        lcd.print("ESP-NOW ERROR   ");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    delay(1500);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("LINK SEARCHING..");
}

void loop() {
    // Process Incoming Telemetry
    if (newPacketReceived) {
        newPacketReceived = false;

        // Heartbeat LED
        greenLedState = !greenLedState;
        digitalWrite(GREEN_LED_PIN, greenLedState ? HIGH : LOW);

        // CSV Serial Logging
        Serial.printf("TELEM,%.2f,%.2f,%.2f,%d\n", rxData.pitch, rxData.yaw, rxData.altitude, rxData.flightState);

        // Status LED
        digitalWrite(RED_LED_PIN, (rxData.flightState == 1 || rxData.flightState == 2) ? HIGH : LOW);

        // State Transition Audio Alerts
        if (rxData.flightState != lastState) {
            lastState = rxData.flightState;
            
            if (rxData.flightState == 1) { 
                beep(1200, 100); delay(50); beep(1200, 100); // Double Beep (ARMED)
            } 
            else if (rxData.flightState == 2) { 
                beep(1800, 300); // High Launch Tone (FLIGHT)
            } 
            else if (rxData.flightState == 0) { 
                beep(400, 200);  // Low Return Tone (IDLE)
            }
        }

        // Update LCD Display with Sub-Meter Decimal Precision
        lcd.setCursor(0, 0);
        lcd.printf("P:%5.1f Y:%5.1f", rxData.pitch, rxData.yaw);

        lcd.setCursor(0, 1);
        lcd.printf("A:%4.1fm [%s]", rxData.altitude, stateNames[rxData.flightState]);
    }

    // Link Timeout Detector (3 seconds)
    if (millis() - lastPacketTime > 3000 && lastPacketTime != 0) {
        lcd.setCursor(0, 0);
        lcd.print("  SIGNAL LOST!  ");
        lcd.setCursor(0, 1);
        lcd.print(" CHECK TRANSMIT ");
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, LOW);
    }

    // Ground Station Button Trigger
    if (digitalRead(BUTTON_PIN) == LOW) {
        beep(2000, 50);
        Serial.println("GND_CMD: USER_INTERRUPT_MARKER");
        delay(200);
    }

    delay(20);
}