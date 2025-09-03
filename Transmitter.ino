#include <SPI.h>
#include "RF24.h"

// Pin Definitions
#define CE_PIN 9
#define CSN_PIN 10
#define STATUS_LED_PIN 13

// Safety Parameters
#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000
#define THROTTLE_STEP 50    // How much throttle changes per key press
#define EMERGENCY_THROTTLE_STEP 50  // Throttle reduction during emergency stop
#define CONTROL_STEP 50       // How much pitch/roll/yaw changes per key press

// Radio Configuration
RF24 radio(CE_PIN, CSN_PIN);
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 1;

// Control Structure
struct ControlInputs {
    int16_t throttle;  // Base throttle (1000-2000)
    int16_t pitch;     // Forward/backward (-500 to 500)
    int16_t roll;      // Left/right (-500 to 500)
    int16_t yaw;       // Rotation (-500 to 500)
} controls;

// System State
bool armed = false;
bool emergencyStopActive = false;
bool transmissionNeeded = false;  // Flag to track when transmission is needed
unsigned long lastEmergencyStopTime = 0;
const unsigned long EMERGENCY_STOP_INTERVAL = 100;  // Throttle reduction interval

void setup() {
    Serial.begin(9600);
    pinMode(STATUS_LED_PIN, OUTPUT);
    
    initializeRadio();
    resetControls();
    
    Serial.println(F("Drone Transmitter Initialized"));
    printHelp();
}

void initializeRadio() {
    if (!radio.begin()) {
        Serial.println(F("Radio hardware not responding!"));
        while (1) {
            digitalWrite(STATUS_LED_PIN, HIGH);
            delay(100);
            digitalWrite(STATUS_LED_PIN, LOW);
            delay(100);
        }
    }
    
    radio.setPALevel(RF24_PA_LOW);
    radio.setPayloadSize(sizeof(ControlInputs));
    radio.setDataRate(RF24_250KBPS);
    radio.setRetries(15, 10);
    
    radio.openWritingPipe(address[radioNumber]);
    radio.openReadingPipe(1, address[!radioNumber]);
    radio.stopListening();
}

void resetControls() {
    controls.throttle = MIN_THROTTLE;
    controls.pitch = 0;
    controls.roll = 0;
    controls.yaw = 0;
    transmissionNeeded = true;  // Transmit initial control state
}

void loop() {
    // Handle serial commands
    handleSerialCommands();
    
    // Handle emergency stop throttle reduction
    if (emergencyStopActive) {
        unsigned long currentTime = millis();
        if (currentTime - lastEmergencyStopTime >= EMERGENCY_STOP_INTERVAL) {
            performEmergencyStop();
            lastEmergencyStopTime = currentTime;
        }
    }
    
    // Transmit only when needed
    if (transmissionNeeded) {
        transmitControls();
        updateStatus();  // Show status only when transmitting
        transmissionNeeded = false;  // Reset flag after transmission
    }
}

void handleSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        if (!armed && cmd != 'f' && cmd != '?') {
            Serial.println(F("System is DISARMED. Press 'f' to arm."));
            return;
        }
        
        switch (cmd) {
            // System commands
            case 'f':  // Arm (F for Fire/Flight)
                armed = true;
                emergencyStopActive = false;
                transmissionNeeded = true;  // Transmit state change
                Serial.println(F("System ARMED"));
                break;
                
            case 'g':  // Disarm
                armed = false;
                emergencyStopActive = false;
                resetControls();
                transmissionNeeded = true;  // Transmit state change
                Serial.println(F("System DISARMED"));
                break;
                
            case 'x':  // Emergency Stop
                armed = false;
                emergencyStopActive = true;
                controls.pitch = 0;
                controls.roll = 0;
                controls.yaw = 0;
                transmissionNeeded = true;  // Transmit state change
                Serial.println(F("EMERGENCY STOP INITIATED"));
                break;
                
            // Throttle control
            case '1':  // Throttle up
                if (!emergencyStopActive) {
                    controls.throttle = constrain(controls.throttle + THROTTLE_STEP, 
                                               MIN_THROTTLE, MAX_THROTTLE);
                    transmissionNeeded = true;  // Transmit control change
                }
                break;
                
            case '0':  // Throttle down
                if (!emergencyStopActive) {
                    controls.throttle = constrain(controls.throttle - THROTTLE_STEP, 
                                               MIN_THROTTLE, MAX_THROTTLE);
                    transmissionNeeded = true;  // Transmit control change
                }
                break;
                
            // Movement controls
            case 'w':  // Forward
                if (!emergencyStopActive) {
                    controls.pitch = constrain(controls.pitch + CONTROL_STEP, -500, 500);
                    transmissionNeeded = true;  // Transmit control change
                }
                break;
                
            case 's':  // Backward
                if (!emergencyStopActive) {
                    controls.pitch = constrain(controls.pitch - CONTROL_STEP, -500, 500);
                    transmissionNeeded = true;  // Transmit control change
                }
                break;
                
            case 'a':  // Left
                if (!emergencyStopActive) {
                    controls.roll = constrain(controls.roll - CONTROL_STEP, -500, 500);
                    transmissionNeeded = true;  // Transmit control change
                }
                break;
                
            case 'd':  // Right
                if (!emergencyStopActive) {
                    controls.roll = constrain(controls.roll + CONTROL_STEP, -500, 500);
                    transmissionNeeded = true;  // Transmit control change
                }
                break;
                
            case 'q':  // Rotate left
                if (!emergencyStopActive) {
                    controls.yaw = constrain(controls.yaw - CONTROL_STEP, -500, 500);
                    transmissionNeeded = true;  // Transmit control change
                }
                break;
                
            case 'e':  // Rotate right
                if (!emergencyStopActive) {
                    controls.yaw = constrain(controls.yaw + CONTROL_STEP, -500, 500);
                    transmissionNeeded = true;  // Transmit control change
                }
                break;
                
            // Reset position
            case 'c':  // Center
                if (!emergencyStopActive) {
                    controls.pitch = 0;
                    controls.roll = 0;
                    controls.yaw = 0;
                    transmissionNeeded = true;  // Transmit control change
                    Serial.println(F("Controls centered"));
                }
                break;
                
            case '?':  // Help
                printHelp();
                break;
        }
    }
}

void performEmergencyStop() {
    // Gradually reduce throttle
    if (controls.throttle > MIN_THROTTLE) {
        controls.throttle = max(controls.throttle - EMERGENCY_STOP_INTERVAL, MIN_THROTTLE);
        transmissionNeeded = true;  // Transmit throttle change
        Serial.print(F("Emergency Stop - Throttle: "));
        Serial.println(controls.throttle);
    } else {
        // When throttle reaches minimum, completely stop emergency mode
        emergencyStopActive = false;
        Serial.println(F("Emergency Stop COMPLETE"));
    }
}

void printHelp() {
    Serial.println(F("\n=== Drone Control Commands ==="));
    Serial.println(F("System Controls:"));
    Serial.println(F("  f: Arm system"));
    Serial.println(F("  g: Disarm system"));
    Serial.println(F("  x: Emergency stop"));
    
    Serial.println(F("\nThrottle Controls:"));
    Serial.println(F("  1: Increase throttle"));
    Serial.println(F("  0: Decrease throttle"));
    
    Serial.println(F("\nMovement Controls:"));
    Serial.println(F("  w: Forward"));
    Serial.println(F("  s: Backward"));
    Serial.println(F("  a: Left"));
    Serial.println(F("  d: Right"));
    Serial.println(F("  q: Rotate left"));
    Serial.println(F("  e: Rotate right"));
    
    Serial.println(F("\nOther Controls:"));
    Serial.println(F("  c: Center all controls"));
    Serial.println(F("  ?: Show this help"));
    Serial.println(F("==========================\n"));
}

void transmitControls() {
    if (!radio.write(&controls, sizeof(ControlInputs))) {
        Serial.println(F("Transmission failed"));
        digitalWrite(STATUS_LED_PIN, HIGH);
    } else {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    }
}

void updateStatus() {
    Serial.println(F("\n--- Transmitting Values ---"));
    Serial.print(F("System: "));
    Serial.println(armed ? F("ARMED") : F("DISARMED"));
    Serial.print(F("Throttle: "));
    Serial.println(controls.throttle);
    Serial.println(F("Control Values:"));
    Serial.print(F("Pitch: "));
    Serial.print(controls.pitch);
    Serial.print(F(" | Roll: "));
    Serial.print(controls.roll);
    Serial.print(F(" | Yaw: "));
    Serial.println(controls.yaw);
    Serial.println(F("------------------------\n"));
}