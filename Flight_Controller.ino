#include <SPI.h>
#include "RF24.h"
#include <Servo.h>
#include <Wire.h>
#include <math.h>

// Pin Definitions
#define CE_PIN 9 // red --- anticlockwise
#define CSN_PIN 53
#define MOTOR_PIN_1 6  // back left motor, white
#define MOTOR_PIN_2 8  // front right motor, white
#define MOTOR_PIN_3 3  // front left motor, red
#define MOTOR_PIN_4 5  // back right motor, red
#define STATUS_LED_PIN 13

// Safety parameters
#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000

// MPU9250 Register Map
#define MPU9250_ADDRESS             0x68
#define ACCEL_XOUT_H               0x3B
#define GYRO_XOUT_H                0x43
#define ACCEL_CONFIG               0x1C
#define GYRO_CONFIG                0x1B
#define CONFIG                     0x1A
#define SMPLRT_DIV                 0x19

// Scale Factors
#define GYRO_SCALE_FACTOR          131.0f    // For ±250deg/s range
#define ACCEL_SCALE_FACTOR         16384.0f  // For ±2g range

// Control Structure matching transmitter
struct ControlInputs {
    int16_t throttle;  // Range: 1000-2000
    int16_t pitch;     // Range: -500 to 500
    int16_t roll;      // Range: -500 to 500
    int16_t yaw;       // Range: -500 to 500
} receivedControls;

// System State
bool armed = false;
bool emergencyStop = false;

// Calibration offsets //---------------------- No error code-------------------//
float accelXoffset = 0.0f;
float accelYoffset = 0.0f;
float accelZoffset = 0.0f;
float gyroXoffset = 0.0f;
float gyroYoffset = 0.0f;
float gyroZoffset = 0.0f;

// Sensor measurements
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

// Orientation Variables
float pitch = 0.0, roll = 0.0, yaw = 0.0;
unsigned long lastTime = 0;

//------------------------------------UPTO here--------------------------------//

// PID Constants
float kp = 2.2, ki = 0.0001, kd = 0.001;

// PID Variables
float setpointPitch = 0.0, setpointRoll = 0.0, setpointYaw = 0.0;
float errorPitch, errorRoll, errorYaw;
float prevErrorPitch = 0, prevErrorRoll = 0, prevErrorYaw = 0;
float integralPitch = 0, integralRoll = 0, integralYaw = 0;
float motorOutputPitch, motorOutputRoll, motorOutputYaw;

// RF24 Configuration
RF24 radio(CE_PIN, CSN_PIN);
uint8_t address[][6] = { "1Node", "2Node" };
bool radioNumber = 0;

// Motor Configuration
Servo motor1, motor2, motor3, motor4;
int baseThrottle = MIN_THROTTLE;
unsigned long lastRadioReceiveTime = 0;
const unsigned long RADIO_TIMEOUT = 1000;

void setup() {
    Wire.begin();
    Serial.begin(9600);  // Match transmitter baud rate
    pinMode(STATUS_LED_PIN, OUTPUT);
    
    // Initialize all systems
    initMPU9250();
    initializeESCs();
    initializeRadio();
    
    // Perform sensor calibration
    Serial.println(F("Position MPU9250 flat and don't move it - calibrating..."));
    delay(1000);
    calibrateSensors();
    Serial.println(F("Calibration Done!"));
    
    lastTime = micros();
    Serial.println(F("Flight controller initialized and ready."));
}

void loop() {
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;
    
    // Read sensor data
    readAccelerometer();
    readGyroscope();
    calculateOrientation(dt);
    
    // Check for any new radio commands
    checkRadioCommands();
    
    // Use last received controls for flight control
    if (armed && !emergencyStop) {
        updateFlightControl(dt);
    } else {
        disableMotors();
    }
}

//------------------------------No flight controller error here as well------------------------------------//
void initMPU9250() {
    // Wake up MPU9250
    writeRegister(MPU9250_ADDRESS, 0x6B, 0x00);
    delay(100);
    
    // Set sample rate divider
    writeRegister(MPU9250_ADDRESS, SMPLRT_DIV, 5);
    
    // Configure Accelerometer
    writeRegister(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  // ±2g range
    
    // Configure Gyroscope
    writeRegister(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);   // ±250 degrees/s range
    
    // Enable DLPF
    writeRegister(MPU9250_ADDRESS, CONFIG, 0x06);        // 5Hz bandwidth
}

void initializeESCs() {
    // Attach motors to pins
    motor1.attach(MOTOR_PIN_1);
    motor2.attach(MOTOR_PIN_2);
    motor3.attach(MOTOR_PIN_3);
    motor4.attach(MOTOR_PIN_4);

    // Step 1: Send maximum throttle signal to all ESCs
    motor1.writeMicroseconds(MAX_THROTTLE);
    motor2.writeMicroseconds(MAX_THROTTLE);
    motor3.writeMicroseconds(MAX_THROTTLE);
    motor4.writeMicroseconds(MAX_THROTTLE);

    // Wait for ESCs to recognize the maximum throttle signal
    delay(2000); // 2 seconds (adjust based on ESC specifications)

    // Step 2: Send minimum throttle signal to all ESCs
    motor1.writeMicroseconds(MIN_THROTTLE);
    motor2.writeMicroseconds(MIN_THROTTLE);
    motor3.writeMicroseconds(MIN_THROTTLE);
    motor4.writeMicroseconds(MIN_THROTTLE);

    // Wait for ESCs to initialize and arm
    delay(5000); // 5 seconds (adjust based on ESC specifications)

    Serial.println(F("ESCs initialized and armed successfully."));
}

//---------------------------------------upto here--------------------------------------//
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
    radio.setRetries(5, 15);
    
    radio.openWritingPipe(address[radioNumber]);
    radio.openReadingPipe(1, address[!radioNumber]);
    radio.startListening();
    
    Serial.println(F("Radio initialized successfully"));
}

//------------------------------------------------------------------------Data from MPU Sensor ( No error here )-----------------------------------------------------------------//
void readAccelerometer() {
    uint8_t data[6];
    readRegisters(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, data);
    
    int16_t rawX = (data[0] << 8) | data[1];
    int16_t rawY = (data[2] << 8) | data[3];
    int16_t rawZ = (data[4] << 8) | data[5];
    
    accelX = (float)rawX / ACCEL_SCALE_FACTOR - accelXoffset;
    accelY = (float)rawY / ACCEL_SCALE_FACTOR - accelYoffset;
    accelZ = (float)rawZ / ACCEL_SCALE_FACTOR - accelZoffset;
}

void readGyroscope() {
    uint8_t data[6];
    readRegisters(MPU9250_ADDRESS, GYRO_XOUT_H, 6, data);
    
    int16_t rawX = (data[0] << 8) | data[1];
    int16_t rawY = (data[2] << 8) | data[3];
    int16_t rawZ = (data[4] << 8) | data[5];
    
    gyroX = ((float)rawX / GYRO_SCALE_FACTOR) - gyroXoffset;
    gyroY = ((float)rawY / GYRO_SCALE_FACTOR) - gyroYoffset;
    gyroZ = ((float)rawZ / GYRO_SCALE_FACTOR) - gyroZoffset;
}

void writeRegister(uint8_t addr, uint8_t reg, uint8_t data) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void readRegisters(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* data) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    
    Wire.requestFrom(addr, count);
    for(uint8_t i = 0; i < count; i++) {
        data[i] = Wire.read();
    }
}

//----------------------------------------------------------------------------UPTO here----------------------------------------------------------------------//

//---------------------------------- [START OF CODE FROM FIRST SKETCH] , No need to change here if any error occurs , this code is already tested-----------------------------------------//

void calibrateSensors() {
    float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
    float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
    const int samples = 1000;
    
    for(int i = 0; i < samples; i++) {
        uint8_t data[6];
        
        // Read accelerometer
        readRegisters(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, data);
        sumAccelX += (int16_t)((data[0] << 8) | data[1]);
        sumAccelY += (int16_t)((data[2] << 8) | data[3]);
        sumAccelZ += (int16_t)((data[4] << 8) | data[5]) - ACCEL_SCALE_FACTOR;
        
        // Read gyroscope
        readRegisters(MPU9250_ADDRESS, GYRO_XOUT_H, 6, data);
        sumGyroX += (int16_t)((data[0] << 8) | data[1]);
        sumGyroY += (int16_t)((data[2] << 8) | data[3]);
        sumGyroZ += (int16_t)((data[4] << 8) | data[5]);
        
        delay(1);
    }
    
    // Calculate offsets
    accelXoffset = sumAccelX / (samples * ACCEL_SCALE_FACTOR);
    accelYoffset = sumAccelY / (samples * ACCEL_SCALE_FACTOR);
    accelZoffset = sumAccelZ / (samples * ACCEL_SCALE_FACTOR);
    
    gyroXoffset = sumGyroX / (samples * GYRO_SCALE_FACTOR);
    gyroYoffset = sumGyroY / (samples * GYRO_SCALE_FACTOR);
    gyroZoffset = sumGyroZ / (samples * GYRO_SCALE_FACTOR);
}

void calculateOrientation(float dt) {
    // Calculate total acceleration vector magnitude
    float accelTotal = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    
    // Only calculate pitch and roll if we're not in free fall and acceleration is close to 1g
    if (abs(accelTotal - 1.0) < 0.3) {
        // Calculate pitch (x-axis rotation)
        pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
        
        // Calculate roll (y-axis rotation)
        roll = atan2(-accelX, accelZ) * 180.0 / PI;
    }
    
    // Calculate yaw from gyroscope with gimbal lock compensation
    float cosRoll = cos(roll * PI / 180.0);
    float sinRoll = sin(roll * PI / 180.0);
    float cosPitch = cos(pitch * PI / 180.0);
    float sinPitch = sin(pitch * PI / 180.0);
    
    // Convert gyro rates to angular motion with gimbal lock compensation
    float yawRate = (gyroZ * cosPitch + gyroY * sinPitch) * cosRoll + 
                    gyroX * sinRoll;
    
    // Integrate yaw rate to get yaw angle
    yaw += yawRate * dt;
    
    // Normalize yaw to -180 to +180 degrees
    if (yaw > 180.0) yaw -= 360.0;
    else if (yaw < -180.0) yaw += 360.0;
}

//-----------------------------------------------------------------------[END OF CODE FROM FIRST SKETCH]--------------------------------------------------------------------//

void checkRadioCommands() {
    if (radio.available()) {
        // Read the control inputs
        radio.read(&receivedControls, sizeof(ControlInputs));
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));  // Toggle LED to show reception
        
        // Process controls
        if (receivedControls.throttle <= MIN_THROTTLE) {
            armed = false;
            emergencyStop = false;
        } else {
            armed = true;
        }
        
        // Map received controls to flight parameters
        if (armed && !emergencyStop) {
            baseThrottle = receivedControls.throttle;
            setpointPitch = map(receivedControls.pitch, -500, 500, -60, 60);
            setpointRoll = map(receivedControls.roll, -500, 500, -60, 60);
            setpointYaw = map(receivedControls.yaw, -500, 500, -180, 180);
        }
    }
}

void updateFlightControl(float dt) {
    if (!armed || emergencyStop) {
        disableMotors();
        return;
    }
    
    applyPID(dt);
    
    // Motor mixing algorithm
    int m1 = constrain(baseThrottle - motorOutputPitch + motorOutputRoll - motorOutputYaw, MIN_THROTTLE, MAX_THROTTLE);
    int m2 = constrain(baseThrottle + motorOutputPitch - motorOutputRoll - motorOutputYaw, MIN_THROTTLE, MAX_THROTTLE);
    int m3 = constrain(baseThrottle + motorOutputPitch + motorOutputRoll + motorOutputYaw, MIN_THROTTLE, MAX_THROTTLE);
    int m4 = constrain(baseThrottle - motorOutputPitch - motorOutputRoll + motorOutputYaw, MIN_THROTTLE, MAX_THROTTLE);
    
    // Apply motor speeds
    motor1.writeMicroseconds(m1);
    motor2.writeMicroseconds(m2);
    motor3.writeMicroseconds(m3);
    motor4.writeMicroseconds(m4);
    
    // Debug output
        printDebugInfo(m1, m2, m3, m4);
}

void disableMotors() {
    motor1.writeMicroseconds(MIN_THROTTLE);
    motor2.writeMicroseconds(MIN_THROTTLE);
    motor3.writeMicroseconds(MIN_THROTTLE);
    motor4.writeMicroseconds(MIN_THROTTLE);
    Serial.println("Motor are disabled");
    
}

void printDebugInfo(int m1, int m2, int m3, int m4) {
    Serial.print(F("Armed: ")); Serial.print(armed);
    Serial.print(F(" Emergency: ")); Serial.print(emergencyStop);
    Serial.print(F(" Throttle: ")); Serial.print(baseThrottle);
    Serial.print(F(" P/R/Y: "));
    Serial.print(pitch); Serial.print(F("/"));
    Serial.print(roll); Serial.print(F("/"));
    Serial.print(yaw);
    Serial.print(F(" Motors: "));
    Serial.print(m1); Serial.print(F("/"));
    Serial.print(m2); Serial.print(F("/"));
    Serial.print(m3); Serial.print(F("/"));
    Serial.println(m4);
}

//---------------------------------------------------------------Only modify constant value here after testing----------------------------------------------------//

void applyPID(float dt) {
    float deadbandPitch = 7;
    float deadbandRoll = 5;
    float deadbandYaw = 10;

    errorPitch = setpointPitch - pitch;
    errorRoll = setpointRoll - roll;
    errorYaw = setpointYaw - yaw;

    errorPitch = (fabs(errorPitch) < deadbandPitch) ? 0.0 : errorPitch;
    errorRoll = (fabs(errorRoll) < deadbandRoll) ? 0.0 : errorRoll;
    errorYaw = (fabs(errorYaw) < deadbandYaw) ? 0.0 : errorYaw;
    
    integralPitch += errorPitch * dt;
    integralRoll += errorRoll * dt;
    integralYaw += errorYaw * dt;
    
    float derivativePitch = (errorPitch - prevErrorPitch) / dt;
    float derivativeRoll = (errorRoll - prevErrorRoll) / dt;
    float derivativeYaw = (errorYaw - prevErrorYaw) / dt;
    
    motorOutputPitch = kp * errorPitch + ki * integralPitch + kd * derivativePitch;
    motorOutputRoll = kp * errorRoll + ki * integralRoll + kd * derivativeRoll;
    motorOutputYaw = kp * errorYaw + ki * integralYaw + kd * derivativeYaw;
    
    prevErrorPitch = errorPitch;
    prevErrorRoll = errorRoll;
    prevErrorYaw = errorYaw;
    
    // Anti-windup: limit integral term
    integralPitch = constrain(integralPitch, -400, 400);
    integralRoll = constrain(integralRoll, -400, 400);
    integralYaw = constrain(integralYaw, -400, 400);
    
    // Limit motor output values
    motorOutputPitch = constrain(motorOutputPitch, -400, 400);
    motorOutputRoll = constrain(motorOutputRoll, -400, 400);
    motorOutputYaw = constrain(motorOutputYaw, -400, 400);
}

//--------------------------------------------------------------------------------------UPTO here---------------------------------------------------------------------//