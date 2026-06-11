/*
  ESP8266 Robot Controller - MQTT Motor Subscriber & UWB Telemetry
  
  This code runs on the robot's ESP8266. It:
  1. Connects to Wi-Fi using ESP8266WiFi.
  2. Connects to the public MQTT broker (broker.hivemq.com).
  3. Subscribes to the topic "rover/motors/commands".
  4. Parses commands in the format "left_speed,right_speed" (e.g. "180,180").
  5. Controls a Cytron Maker Drive motor driver using PWM.
  6. Reads UWB ranging data from BU03 via SoftwareSerial and publishes to "rover/uwb/raw".

  Dependencies:
  - PubSubClient library (by Nick O'Leary) - install via Arduino Library Manager.
  - SoftwareSerial library (built-in for ESP8266).
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

// Wi-Fi Credentials
const char* ssid = "HUAWEI nova 9";             // Replace with your Wi-Fi SSID
const char* password = "TESTpassword99";     // Replace with your Wi-Fi password

// MQTT Configuration
const char* mqtt_broker = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "rover/motors/commands";
const char* mqtt_uwb_topic = "rover/uwb/raw";
const char* client_id = "ESP8266_Rover_Client";

// Pin Definitions for Cytron Maker Drive on ESP8266
#define PIN_L_M1A 4    // Left Motor M1A Input - GPIO4 (D2)
#define PIN_L_M1B 5    // Left Motor M1B Input - GPIO5 (D1)
#define PIN_R_M2A 12   // Right Motor M2A Input - GPIO12 (D6)
#define PIN_R_M2B 13   // Right Motor M2B Input - GPIO13 (D7)

// UWB Configuration (SoftwareSerial pins)
#define PIN_UWB_RX 14  // GPIO14 (D5) - Connect to UWB TX
#define PIN_UWB_TX 2   // GPIO2  (D4) - Connect to UWB RX

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================

WiFiClient espClient;
PubSubClient client(espClient);
SoftwareSerial uwbSerial(PIN_UWB_RX, PIN_UWB_TX);
unsigned long lastReconnectAttempt = 0;
unsigned long lastMotorCommandTime = 0;

// =============================================================================
// MOTOR CONTROL FUNCTIONS
// =============================================================================

int const_rain(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

void setupMotors() {
  pinMode(PIN_L_M1A, OUTPUT);
  pinMode(PIN_L_M1B, OUTPUT);
  pinMode(PIN_R_M2A, OUTPUT);
  pinMode(PIN_R_M2B, OUTPUT);
  
  stopRobot();
}

void setMotorLeft(int speed) {
  speed = const_rain(speed, -255, 255);

  // Invert left motor speed so positive speed drives forward
  speed = -speed;

  if (speed > 0) {
    analogWrite(PIN_L_M1A, speed);
    digitalWrite(PIN_L_M1B, LOW);
  } else if (speed < 0) {
    digitalWrite(PIN_L_M1A, LOW);
    analogWrite(PIN_L_M1B, abs(speed));
  } else {
    digitalWrite(PIN_L_M1A, HIGH);
    digitalWrite(PIN_L_M1B, HIGH);
  }
}

void setMotorRight(int speed) {
  speed = const_rain(speed, -255, 255);

  // Invert right motor speed so positive speed drives forward
  speed = -speed;

  if (speed > 0) {
    analogWrite(PIN_R_M2A, speed);
    digitalWrite(PIN_R_M2B, LOW);
  } else if (speed < 0) {
    digitalWrite(PIN_R_M2A, LOW);
    analogWrite(PIN_R_M2B, abs(speed));
  } else {
    digitalWrite(PIN_R_M2A, HIGH);
    digitalWrite(PIN_R_M2B, HIGH);
  }
}

void stopRobot() {
  setMotorLeft(0);
  setMotorRight(0);
  Serial.println("Motors Halted.");
}

// =============================================================================
// UWB TELEMETRY PROCESSING
// =============================================================================

unsigned long uwb_bytes_received = 0;
unsigned long uwb_last_debug_print = 0;

void processUWB() {
  uint8_t batch_buf[128];
  int batch_len = 0;

  // Read available bytes into a batch buffer
  while (uwbSerial.available() > 0 && batch_len < 128) {
    batch_buf[batch_len++] = uwbSerial.read();
    uwb_bytes_received++;
  }

  // Publish the raw batch to MQTT
  if (batch_len > 0) {
    if (client.connected()) {
      client.publish(mqtt_uwb_topic, batch_buf, batch_len);
    }
  }

  // Print byte count every 2 seconds for diagnostics
  unsigned long now = millis();
  if (now - uwb_last_debug_print > 2000) {
    uwb_last_debug_print = now;
    Serial.print("[DEBUG] UWB total bytes received: ");
    Serial.println(uwb_bytes_received);
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

  WiFi.mode(WIFI_STA);
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

void callback(char* topic, byte* payload, unsigned int length) {
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

  int leftSpeed = 0;
  int rightSpeed = 0;
  
  if (sscanf(message, "%d,%d", &leftSpeed, &rightSpeed) == 2) {
    Serial.print("Parsed Speeds -> Left: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right: ");
    Serial.println(rightSpeed);

    setMotorLeft(leftSpeed);
    setMotorRight(rightSpeed);
    lastMotorCommandTime = millis();
  } else {
    Serial.println("❌ Parsing failed. Invalid command format.");
  }
}

boolean connectMQTT() {
  Serial.print("Connecting to MQTT broker: ");
  Serial.println(mqtt_broker);
  
  if (client.connect(client_id)) {
    Serial.println("✓ Connected to MQTT Broker!");
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
// MAIN ENTRY POINTS
// =============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- ESP8266 Rover Motor Controller Starting ---");

  // Initialize SoftwareSerial for UWB BU03 communication
  uwbSerial.begin(115200);
  Serial.println("✓ SoftwareSerial initialized for UWB at 115200 baud.");

  setupMotors();
  setupWiFi();

  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    setupWiFi();
  }

  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (connectMQTT()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client.loop();
  }

  processUWB();

  // Safety Watchdog: Stop motors if no MQTT command received in the last 1.5 seconds
  if (millis() - lastMotorCommandTime > 1500) {
    setMotorLeft(0);
    setMotorRight(0);
  }
}
