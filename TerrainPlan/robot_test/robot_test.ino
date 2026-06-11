/*
  ESP8266 Motor Test Program
  
  This is a standalone test program to verify that the robot drives correctly
  using the Cytron Maker Drive dual-channel motor driver connected to an ESP8266.
  
  It:
  1. Drives both motors forward at medium speed for 4 seconds.
  2. Halts for 2 seconds.
  3. Drives both motors backward at medium speed for 4 seconds.
  4. Halts for 2 seconds.
  5. Repeats indefinitely.
*/

// Pin Definitions for Cytron Maker Drive on ESP8266
#define PIN_L_M1A 4    // Left Motor M1A Input - GPIO4 (D2)
#define PIN_L_M1B 5    // Left Motor M1B Input - GPIO5 (D1)

#define PIN_R_M2A 12   // Right Motor M2A Input - GPIO12 (D6)
#define PIN_R_M2B 13   // Right Motor M2B Input - GPIO13 (D7)

// Clamp utility
int const_rain(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

// Motor Control functions
void setMotorLeft(int speed) {
  speed = const_rain(speed, -255, 255);

  // Invert left motor speed so positive speed drives forward
  speed = -speed;

  if (speed > 0) {
    // Forward: M1A = PWM, M1B = LOW
    analogWrite(PIN_L_M1A, speed);
    digitalWrite(PIN_L_M1B, LOW);
  } else if (speed < 0) {
    // Reverse: M1A = LOW, M1B = PWM
    digitalWrite(PIN_L_M1A, LOW);
    analogWrite(PIN_L_M1B, abs(speed));
  } else {
    // Active Brake: Both HIGH
    digitalWrite(PIN_L_M1A, HIGH);
    digitalWrite(PIN_L_M1B, HIGH);
  }
}

void setMotorRight(int speed) {
  speed = const_rain(speed, -255, 255);

  // Invert right motor speed so positive speed drives forward
  speed = -speed;

  if (speed > 0) {
    // Forward: M2A = PWM, M2B = LOW
    analogWrite(PIN_R_M2A, speed);
    digitalWrite(PIN_R_M2B, LOW);
  } else if (speed < 0) {
    // Reverse: M2A = LOW, M2B = PWM
    digitalWrite(PIN_R_M2A, LOW);
    analogWrite(PIN_R_M2B, abs(speed));
  } else {
    // Active Brake: Both HIGH
    digitalWrite(PIN_R_M2A, HIGH);
    digitalWrite(PIN_R_M2B, HIGH);
  }
}

void stopRobot() {
  setMotorLeft(0);
  setMotorRight(0);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- ESP8266 Motor Test Program Starting ---");

  // Configure control pins as outputs
  pinMode(PIN_L_M1A, OUTPUT);
  pinMode(PIN_L_M1B, OUTPUT);
  pinMode(PIN_R_M2A, OUTPUT);
  pinMode(PIN_R_M2B, OUTPUT);

  // Initialize stopped
  stopRobot();
  Serial.println("System initialized. Starting drive loop in 2 seconds...");
  delay(2000);
}

void loop() {
  // 1. Forward
  Serial.println("Driving FORWARD (speed: 150) for 4 seconds...");
  setMotorLeft(150);
  setMotorRight(150);
  delay(4000);

  // 2. Stop
  Serial.println("Halting motors for 2 seconds...");
  stopRobot();
  delay(2000);

  // 3. Backward
  Serial.println("Driving BACKWARD (speed: -150) for 4 seconds...");
  setMotorLeft(-150);
  setMotorRight(-150);
  delay(4000);

  // 4. Stop
  Serial.println("Halting motors for 2 seconds...");
  stopRobot();
  delay(20000);
}