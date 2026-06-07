/*
  ESP32 Robot Controller - MQTT Motor Subscriber
  
  This code runs on the robot's ESP32. It:
  1. Connects to Wi-Fi.
  2. Connects to the public MQTT broker (broker.hivemq.com).
  3. Subscribes to the topic "rover/motors/commands".
  4. Parses commands in the format "left_speed,right_speed" (e.g. "180,180").
  5. Controls a Cytron Maker Drive motor driver using dual PWM.

  Dependencies:
  - PubSubClient library (by Nick O'Leary) - install via Arduino Library Manager.
*/

#include <WiFi.h>
#include <PubSubClient.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

// Wi-Fi Credentials
const char* ssid = "YOUR_WIFI_SSID";             // Replace with your Wi-Fi SSID
const char* password = "YOUR_WIFI_PASSWORD";     // Replace with your Wi-Fi password

// MQTT Configuration
const char* mqtt_broker = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "rover/motors/commands";
const char* client_id = "ESP32_Rover_Client";

// Pin Definitions for Cytron Maker Drive dual-channel motor driver
// (Change these pins to match your physical wiring)
#define PIN_L_M1A 12   // Left Motor M1A Input (PWM/DIR)
#define PIN_L_M1B 14   // Left Motor M1B Input (PWM/DIR)

#define PIN_R_M2A 13   // Right Motor M2A Input (PWM/DIR)
#define PIN_R_M2B 25   // Right Motor M2B Input (PWM/DIR)

// PWM parameters (analogWrite handles these automatically on ESP32 Core v2.0+)
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8  // 8-bit resolution (0-255)

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastReconnectAttempt = 0;

// =============================================================================
// MOTOR CONTROL FUNCTIONS
// =============================================================================

void setupMotors() {
  // Configure control pins as outputs
  pinMode(PIN_L_M1A, OUTPUT);
  pinMode(PIN_L_M1B, OUTPUT);

  pinMode(PIN_R_M2A, OUTPUT);
  pinMode(PIN_R_M2B, OUTPUT);

  // Initialize motors to stopped state
  stopRobot();
}

void setMotorLeft(int speed) {
  // Clamp speed to standard 8-bit range
  speed = const_rain(speed, -255, 255);

  if (speed > 0) {
    // Forward: M1A = PWM, M1B = LOW
    analogWrite(PIN_L_M1A, speed);
    digitalWrite(PIN_L_M1B, LOW);
  } else if (speed < 0) {
    // Reverse: M1A = LOW, M1B = PWM
    digitalWrite(PIN_L_M1A, LOW);
    analogWrite(PIN_L_M1B, abs(speed));
  } else {
    // Active Brake: Both HIGH (Cytron Maker Drive supports active braking on both HIGH)
    digitalWrite(PIN_L_M1A, HIGH);
    digitalWrite(PIN_L_M1B, HIGH);
  }
}

void setMotorRight(int speed) {
  // Clamp speed to standard 8-bit range
  speed = const_rain(speed, -255, 255);

  if (speed > 0) {
    // Forward: M2A = PWM, M2B = LOW
    analogWrite(PIN_R_M2A, speed);
    digitalWrite(PIN_R_M2B, LOW);
  } else if (speed < 0) {
    // Reverse: M2A = LOW, M2B = PWM
    digitalWrite(PIN_R_M2A, LOW);
    analogWrite(PIN_R_M2B, abs(speed));
  } else {
    // Active Brake: Both HIGH (Cytron Maker Drive supports active braking on both HIGH)
    digitalWrite(PIN_R_M2A, HIGH);
    digitalWrite(PIN_R_M2B, HIGH);
  }
}

// Utility function to clamp a value (re-implementing constrain)
int const_rain(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

void stopRobot() {
  setMotorLeft(0);
  setMotorRight(0);
  Serial.println("Motors Halted.");
}

// =============================================================================
// WI-FI & MQTT COMMUNICATIONS
// =============================================================================

void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to Wi-Fi SSID: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ Wi-Fi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ Wi-Fi Connection Failed. Will retry in loop.");
  }
}

// Callback executed when an MQTT message is received
void callback(char* topic, byte* payload, unsigned int length) {
  // Copy payload into a null-terminated char array
  char message[32];
  unsigned int i;
  for (i = 0; i < length && i < sizeof(message) - 1; i++) {
    message[i] = (char)payload[i];
  }
  message[i] = '\0';

  Serial.print("Received MQTT Message [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  // Parse ASCII CSV tokens: left_speed,right_speed
  int leftSpeed = 0;
  int rightSpeed = 0;
  
  if (sscanf(message, "%d,%d", &leftSpeed, &rightSpeed) == 2) {
    Serial.print("Parsed Speeds -> Left: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right: ");
    Serial.println(rightSpeed);

    // Apply motor commands
    setMotorLeft(leftSpeed);
    setMotorRight(rightSpeed);
  } else {
    Serial.println("❌ Parsing failed. Invalid command format.");
  }
}

boolean connectMQTT() {
  Serial.print("Connecting to MQTT broker: ");
  Serial.println(mqtt_broker);
  
  if (client.connect(client_id)) {
    Serial.println("✓ Connected to MQTT Broker!");
    // Subscribe to motors commands topic
    client.subscribe(mqtt_topic);
    Serial.print("Subscribed to topic: ");
    Serial.println(mqtt_topic);
    return true;
  }
  
  Serial.print("❌ Connection failed, rc=");
  Serial.println(client.state());
  return false;
}

// =============================================================================
// MAIN ARDUINO ENTRY POINTS
// =============================================================================

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("\n--- ESP32 Rover Motor Controller Starting ---");

  // Setup motor pins
  setupMotors();

  // Setup WiFi
  setupWiFi();

  // Setup MQTT client
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  // Check Wi-Fi Connection
  if (WiFi.status() != WL_CONNECTED) {
    setupWiFi();
  }

  // Check MQTT Connection and handle reconnect (non-blocking)
  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) { // Retry every 5 seconds
      lastReconnectAttempt = now;
      if (connectMQTT()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Process incoming messages and keep connection alive
    client.loop();
  }
}
