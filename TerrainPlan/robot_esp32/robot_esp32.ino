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
const char* mqtt_uwb_topic = "rover/uwb/raw";
const char* client_id = "ESP32_Rover_Client";

// UWB Configuration
#define PIN_UWB_RX 16
#define PIN_UWB_TX 17

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
// UWB TELEMETRY PROCESSING
// =============================================================================

uint8_t uwb_buf[128];
int uwb_buf_len = 0;
const char* UWB_HEADER = "CmdM:4[";
const int UWB_HEADER_LEN = 7;

void processUWB() {
  while (Serial2.available() > 0) {
    uint8_t c = Serial2.read();
    
    // Add byte to buffer if space is available
    if (uwb_buf_len < sizeof(uwb_buf)) {
      uwb_buf[uwb_buf_len++] = c;
    } else {
      // Buffer overflow, shift left to make room
      memmove(uwb_buf, uwb_buf + 1, uwb_buf_len - 1);
      uwb_buf[uwb_buf_len - 1] = c;
    }
    
    // Try to find header in the buffer
    int header_idx = -1;
    for (int i = 0; i <= uwb_buf_len - UWB_HEADER_LEN; i++) {
      if (memcmp(uwb_buf + i, UWB_HEADER, UWB_HEADER_LEN) == 0) {
        header_idx = i;
        break;
      }
    }
    
    if (header_idx > 0) {
      // If header is found but not at start, discard everything before header
      memmove(uwb_buf, uwb_buf + header_idx, uwb_buf_len - header_idx);
      uwb_buf_len -= header_idx;
    } else if (header_idx == -1 && uwb_buf_len >= UWB_HEADER_LEN) {
      // No header found and buffer has at least UWB_HEADER_LEN bytes.
      // Keep only the last UWB_HEADER_LEN - 1 bytes in case the header is partially read.
      memmove(uwb_buf, uwb_buf + uwb_buf_len - (UWB_HEADER_LEN - 1), UWB_HEADER_LEN - 1);
      uwb_buf_len = UWB_HEADER_LEN - 1;
    }
    
    // Check if we have a complete packet (starting with header and ending with \r\n)
    if (uwb_buf_len >= UWB_HEADER_LEN + 2) {
      // Look for \r\n after the header
      int end_idx = -1;
      for (int i = UWB_HEADER_LEN; i < uwb_buf_len - 1; i++) {
        if (uwb_buf[i] == '\r' && uwb_buf[i+1] == '\n') {
          end_idx = i;
          break;
        }
      }
      
      if (end_idx != -1) {
        int packet_len = end_idx + 2;
        
        // We have a full packet! Publish it to MQTT if connected
        if (client.connected()) {
          if (client.publish(mqtt_uwb_topic, uwb_buf, packet_len)) {
            Serial.print("UWB Telemetry: Published packet of ");
            Serial.print(packet_len);
            Serial.println(" bytes.");
          } else {
            Serial.println("❌ UWB Telemetry: MQTT Publish failed.");
          }
        } else {
          Serial.println("⚠️ UWB Telemetry: Skipped publish (MQTT disconnected).");
        }
        
        // Shift remaining bytes in buffer
        memmove(uwb_buf, uwb_buf + packet_len, uwb_buf_len - packet_len);
        uwb_buf_len -= packet_len;
      }
    }
  }
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

  // Initialize Serial2 for UWB BU03 communication
  Serial2.begin(115200, SERIAL_8N1, PIN_UWB_RX, PIN_UWB_TX);
  Serial.println("✓ Serial2 initialized for UWB at 115200 baud.");

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

  // Process UWB ranging telemetry
  processUWB();
}
