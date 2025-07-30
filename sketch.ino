#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPL3gFfsWAA8"
#define BLYNK_TEMPLATE_NAME "Obstacle Avoiding Bot"
#define BLYNK_AUTH_TOKEN "MmvjtxO_mUz_arWnOQMK0AcXtAKOfnjB"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <ESP32Servo.h>

// Your WiFi credentials
char ssid[] = "Wokwi-GUEST";
char pass[] = "";

// Blynk Auth Token
char auth[] = BLYNK_AUTH_TOKEN;

// Pin Definitions
#define TRIG_PIN 5
#define ECHO_PIN 18
#define DHT_PIN 4
#define DHT_TYPE DHT22
#define MQ135_PIN 34
#define SERVO_PIN 2

// Direct Motor Control Pins (No L298N needed in Wokwi)
#define LEFT_MOTOR_PIN1 19
#define LEFT_MOTOR_PIN2 21
#define RIGHT_MOTOR_PIN1 22
#define RIGHT_MOTOR_PIN2 23

// LED indicators
#define STATUS_LED 25
#define OBSTACLE_LED 26
#define DANGER_LED 27

// Objects
DHT dht(DHT_PIN, DHT_TYPE);
Servo radarServo;
BlynkTimer timer;

// Variables
float temperature = 0;
float humidity = 0;
int airQuality = 0;
int distance = 0;
bool obstacleDetected = false;
bool isMoving = false;
String robotStatus = "Idle";

// Movement parameters
const int SAFE_DISTANCE = 20;  // cm
const int TURN_DELAY = 1000;   // ms
const int SCAN_DELAY = 500;    // ms

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(OBSTACLE_LED, OUTPUT);
  pinMode(DANGER_LED, OUTPUT);
  
  // Motor pins (Direct control)
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  
  // Initialize components
  dht.begin();
  radarServo.attach(SERVO_PIN);
  radarServo.write(90); // Center position
  
  // Initialize Blynk
  Blynk.begin(auth, ssid, pass);
  
  // Setup a function to be called every 2 seconds
  timer.setInterval(2000L, sendSensorData);
  
  digitalWrite(STATUS_LED, HIGH);
  
  Serial.println("🤖 OBSTACLE AVOIDING ROBOT WITH TELEMATICS 🤖");
  Serial.println("==============================================");
  Serial.println("✅ System Initialized Successfully!");
  Serial.println("📡 Sensors: Ultrasonic, DHT22, MQ135");
  Serial.println("🚗 Motors: Direct ESP32 Control");
  Serial.println("🌐 Blynk: Connected for Remote Monitoring");
  Serial.println("🎯 Mission: Autonomous Navigation Started");
  Serial.println("==============================================\n");
}

void loop() {
  Blynk.run();
  timer.run();
  
  // Read sensors
  readSensors();
  
  // Main navigation logic
  performNavigation();
  
  delay(100);
}

void readSensors() {
  // Read temperature and humidity
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  
  // Handle sensor errors
  if (isnan(temperature)) temperature = 25.0;
  if (isnan(humidity)) humidity = 60.0;
  
  // Read air quality
  airQuality = analogRead(MQ135_PIN);
  airQuality = map(airQuality, 0, 4095, 50, 400);
  
  // Read distance
  distance = getDistance();
  
  // Check for obstacles
  obstacleDetected = (distance < SAFE_DISTANCE && distance > 0);
  
  // Update LEDs
  digitalWrite(OBSTACLE_LED, obstacleDetected);
  digitalWrite(DANGER_LED, (airQuality > 300 || temperature > 35));
}

int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  
  int dist = duration * 0.034 / 2;
  return (dist > 0 && dist < 400) ? dist : 999;
}

void performNavigation() {
  if (obstacleDetected) {
    robotStatus = "🚫 Avoiding Obstacle";
    Serial.println("⚠️  OBSTACLE! Distance: " + String(distance) + "cm");
    
    stopMotors();
    delay(500);
    
    // Scan for best direction
    Serial.println("🔍 Scanning...");
    int leftDistance = scanDirection(45);
    delay(SCAN_DELAY);
    int rightDistance = scanDirection(135);
    delay(SCAN_DELAY);
    radarServo.write(90);
    
    Serial.println("📊 Left: " + String(leftDistance) + "cm | Right: " + String(rightDistance) + "cm");
    
    if (leftDistance > rightDistance && leftDistance > SAFE_DISTANCE) {
      Serial.println("⬅️  Turning LEFT");
      turnLeft();
      robotStatus = "⬅️ Turning Left";
    } else if (rightDistance > SAFE_DISTANCE) {
      Serial.println("➡️  Turning RIGHT");
      turnRight();
      robotStatus = "➡️ Turning Right";
    } else {
      Serial.println("🔄 Reversing");
      moveBackward();
      delay(1000);
      turnRight();
      robotStatus = "🔄 Reversing";
    }
  } else {
    moveForward();
    robotStatus = "🚀 Moving Forward";
    isMoving = true;
  }
}

int scanDirection(int angle) {
  radarServo.write(angle);
  delay(500);
  return getDistance();
}

// Direct Motor Control Functions (No L298N needed)
void moveForward() {
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  isMoving = true;
}

void moveBackward() {
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
  isMoving = true;
}

void turnLeft() {
  // Left motor backward, right motor forward
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  delay(TURN_DELAY);
  stopMotors();
}

void turnRight() {
  // Left motor forward, right motor backward
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
  delay(TURN_DELAY);
  stopMotors();
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  isMoving = false;
}

// This function sends sensor data to Blynk
void sendSensorData() {
  Blynk.virtualWrite(V0, temperature);      // Temperature
  Blynk.virtualWrite(V1, humidity);         // Humidity
  Blynk.virtualWrite(V2, airQuality);       // Air Quality
  Blynk.virtualWrite(V3, distance);         // Distance
  Blynk.virtualWrite(V4, obstacleDetected); // Obstacle status
  Blynk.virtualWrite(V5, robotStatus);      // Robot status
  Blynk.virtualWrite(V6, isMoving);         // Movement status
  
  // Print status to serial
  Serial.println("\n📊 ===== SENSOR DATA SENT TO BLYNK =====");
  Serial.println("🌡️  Temperature: " + String(temperature, 1) + "°C");
  Serial.println("💧 Humidity: " + String(humidity, 1) + "%");
  Serial.println("🌪️  Air Quality: " + String(airQuality) + " PPM");
  Serial.println("📏 Distance: " + String(distance) + " cm");
  Serial.println("🚨 Obstacle: " + String(obstacleDetected ? "⚠️ YES" : "✅ NO"));
  Serial.println("🤖 Status: " + robotStatus);
  Serial.println("=======================================\n");
  
  // Environmental alerts
  if (temperature > 35) {
    Blynk.logEvent("high_temp", "High temperature: " + String(temperature) + "°C");
  }
  if (airQuality > 300) {
    Blynk.logEvent("poor_air", "Poor air quality: " + String(airQuality) + " PPM");
  }
}

// Blynk virtual pin handlers
BLYNK_WRITE(V7) { // Manual control override
  int value = param.asInt();
  if (value == 1) {
    robotStatus = "🔧 Manual Override";
    stopMotors();
    Serial.println("🔧 Manual Override Activated!");
  }
}

BLYNK_WRITE(V8) { // Emergency stop
  int value = param.asInt();
  if (value == 1) {
    robotStatus = "🛑 Emergency Stop";
    stopMotors();
    digitalWrite(DANGER_LED, HIGH);
    Serial.println("🛑 EMERGENCY STOP ACTIVATED!");
  }
}

BLYNK_CONNECTED() {
  Serial.println("🌐 Connected to Blynk Cloud!");
}

BLYNK_DISCONNECTED() {
  Serial.println("❌ Disconnected from Blynk!");
}