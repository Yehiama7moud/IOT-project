#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//#include <ESP32Servo.h>
#include <WiFi.h>
#include<WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESPSupabase.h>
#include <Arduino.h>


//Database info 
Supabase db;
String gateTable = "gate_status";
String streetLight_table = "street_lights";
String trafficLights_table = "traffic_lights";
String violations_table = "violations";

// WiFi credentials
const char* ssid = "HK-Home-908";
const char* password = "KE5925mt";

// Supabase credentials
const char* supabaseUrl ="https://ufrtmaqbbssewzieonnq.supabase.co";
const char* supabaseKey ="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InVmcnRtYXFiYnNzZXd6aWVvbm5xIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTU2MjcyNjAsImV4cCI6MjA3MTIwMzI2MH0.m1Z5cLZlAau1VYpEgcAsBVnyMbDqBUkI0M-CyTW0ODk";

// HiveMQ credentials
const char* mqtt_server = "db214e9cf2184882a22e1639d81e429f.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "Hania";
const char* mqtt_pass = "5566mLKs";
const char* client_id = "esp32Client";

WiFiClientSecure wifiClient;
PubSubClient client(wifiClient);


// Pin Definitions
#define IR_SENSOR_PIN 35   // FC-51 IR Sensor OUT pin
#define SERVO_PIN 18     // Servo Motor
#define RED_LED   27     // Red Traffic Light
#define YELLOW_LED 32     // Yellow LED
#define GREEN_LED  19      // Green Traffic Light
#define BUZZER_PIN 26      // Buzzer
#define LDR_PIN 34    // LDR on Analog GPIO34
#define BLUE_LED 25     // Blue LED output pin



// LCD Setup (I2C Address: 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

//Servo gateServo;
// PWM settings for servo
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 50;       // 50Hz for servo
const int PWM_RESOLUTION = 16; // 16-bit

// Function to write angle to servo (0–180°)
void writeServo(int angle) {
  // Limit angle between 0–180 (since typical hobby servos don't do -90)
  angle = constrain(angle, 0, 180);
  int duty = map(angle, 0, 180, 1638, 7864); // 16-bit duty cycle
  ledcWrite(PWM_CHANNEL, duty);
}
// Traffic Light States
enum TrafficState { STOP, READY, GO };
TrafficState currentState = STOP;



String currentTrafficId = "";
String currentGateId = "";



// Timing Variables
unsigned long previousMillis = 0;
const long stopDuration = 10000;    // 10s stop
const long readyDuration = 3000;    // 3s ready
const long goDuration = 10000;      // 10s go
int countdown = 10;

// IR Sensor Variables
bool carDetected = false;
bool violationDetected = false;
bool manualOverride = false;


// Function Prototypes
void setTrafficState(TrafficState newState);
void checkCarMovement();
void handleViolation();
void updateCountdown();
// Supabase functions

int sendViolation(String nightMode) {
  String Json = "{\"car_detected\":true";

 if (nightMode != "") {
    Json += ",\"night_mode\":\"" + nightMode + "\"";
  }

  Json += "}";
  Serial.println("Sending JSON: " + Json);
  int responseCode = db.insert("violations", Json, false); // false = no upsert
  Serial.print("Supabase response: ");
  Serial.println(responseCode);
  return responseCode;
}


int logTrafficLight(String status, int countdown) {
  String Json = "{\"status\":\"" + status + "\",\"countdown\":" + String(countdown) + "}";
  int responseCode = db.insert("traffic_lights", Json, false); // false = no upsert
  Serial.print("HTTP Response Code: ");
  Serial.println(responseCode);
  return responseCode;

}

int logGateStatus(String status) {
  String Json = "{\"status\":\"" + status + "\"}";
  int responseCode = db.insert("gate_status", Json, false); // false = no upsert
  Serial.print("HTTP Response Code: ");
  Serial.println(responseCode);
  return responseCode;

}

int logStreetLight(String status) {
  String Json = "{\"status\":\"" + status + "\"}";
  int responseCode = db.insert("street_lights", Json, false); // false = no upsert
  Serial.print("HTTP Response Code: ");
  Serial.println(responseCode);
  return responseCode;
}


//MQTT Functions 
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message received on topic ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);

 if (String(topic) == "trafficApp/gate") {
  manualOverride = true;// Pause automatic mode
  countdown = 5;
  if (message == "OPEN") {
    Serial.println("Gate  open");
    writeServo(90); // Open gate
    lcd.setCursor(0, 1);
    lcd.print("Manual: OPEN  ");
    logGateStatus("OPEN");

  } else if (message == "CLOSE") {
    Serial.println("Gate close");
    writeServo(0); // Close gate
    lcd.setCursor(0, 1);
    lcd.print("Manual: CLOSE ");
    logGateStatus("CLOSED");
  }
}
if (String(topic) == "trafficApp/light") {
    manualOverride = true;// Pause automatic mode
    countdown = 5;
    if (message == "NIGHT") {
      Serial.println(" Night mode ON");
      digitalWrite(BLUE_LED, HIGH);
      lcd.setCursor(0, 1);
      lcd.print("Night Mode ON ");
      logStreetLight("ON");
      

    } else if (message == "DAY") {
      Serial.println(" Night mode OFF");
      digitalWrite(BLUE_LED, LOW);
      logStreetLight("OFF");

    }
  }

  if (String(topic) == "trafficApp/traffic") {
  manualOverride = true; // Pause automatic mode
  countdown = 5;
  if (message == "RED") setTrafficState(STOP);
  else if (message == "YELLOW") setTrafficState(READY);
  else if (message == "GREEN") setTrafficState(GO);
}
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Reconnecting to MQTT...");
    if (client.connect(client_id, mqtt_user, mqtt_pass)) {
      Serial.println("connected!");
      client.subscribe("trafficApp/gate");
      client.subscribe("trafficApp/light");
      client.subscribe("trafficApp/traffic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}





void setup() {
  Serial.begin(115200);

//WIFI CONNECTION
  WiFi.begin(ssid, password);
  wifiClient.setInsecure(); // Skip certificate validation
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n WiFi connected");
   
  //MQTT CONNECTION
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  //SUPABASE CONNECTION
  db.begin(supabaseUrl, supabaseKey);
 
  while (!client.connected()) {// check if connected and reconnect to mqtt server
    Serial.print("Connecting to MQTT...");
    if (client.connect(client_id, mqtt_user, mqtt_pass)) {
      Serial.println("connected!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }




  // Initialize Servo
 /*ESP32PWM::allocateTimer(0);
  gateServo.setPeriodHertz(50);
  gateServo.attach(SERVO_PIN, 500, 2400);
  */ 
  // Setup PWM for servo
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN, PWM_CHANNEL);
  writeServo(90);


  
// Subscribe to MQTT Server
  client.subscribe("trafficApp/gate");
  client.subscribe("trafficApp/light");
  client.subscribe("trafficApp/traffic");


  // Initialize Pins
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);  // Blue LED
  pinMode(LDR_PIN, INPUT);

  // Initialize LEDs to OFF (active-high logic)
  digitalWrite(RED_LED, HIGH);
  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(BLUE_LED, HIGH);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Traffic System");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // LED Test Sequence
  digitalWrite(RED_LED, LOW);
  lcd.setCursor(0, 0);
  lcd.print("STOP TEST     ");
  delay(500);
  digitalWrite(RED_LED, HIGH);

  digitalWrite(YELLOW_LED, LOW);
  lcd.setCursor(0, 0);
  lcd.print("READY TEST    ");
  delay(500);
  digitalWrite(YELLOW_LED, HIGH);

  digitalWrite(GREEN_LED, LOW);
  lcd.setCursor(0, 0);
  lcd.print("GO TEST       ");
  delay(500);
  digitalWrite(GREEN_LED, HIGH);

  lcd.clear();
  setTrafficState(STOP); // Start in STOP state*/

}
void loop() {
   client.loop(); // Handle MQTT messages
  if (!client.connected()) {
  reconnectMQTT();
}

  unsigned long currentMillis = millis();

  checkCarMovement(); // Check PIR sensor

  // Handle violation buzzer and lockout
  if (violationDetected) {
    handleViolation();
    return; // Skip rest of loop during violation
  }
   // Update countdown every second
  static unsigned long lastCountUpdate = 0;
  if (currentMillis - lastCountUpdate >= 1000) {
    lastCountUpdate = currentMillis;
    if (countdown > 0) {
      countdown--;
    }
    updateCountdown();
  }
  // Manual override logic
  if (manualOverride) {
    if (countdown == 0) {
      manualOverride = false;
      setTrafficState(STOP); // Resume from STOP or last known state
      previousMillis = currentMillis;
      countdown = stopDuration / 1000;
    }
  } else {
    // Automatic traffic light cycling
    switch (currentState) {
      case STOP:
        if (currentMillis - previousMillis >= stopDuration) {
          setTrafficState(READY);
          previousMillis = currentMillis;
          countdown = readyDuration / 1000;
        }
        break;

      case READY:
        if (currentMillis - previousMillis >= readyDuration) {
          setTrafficState(GO);
          previousMillis = currentMillis;
          countdown = goDuration / 1000;
        }
        break;

      case GO:
        if (currentMillis - previousMillis >= goDuration) {
          setTrafficState(STOP);
          previousMillis = currentMillis;
          countdown = stopDuration / 1000;
        }
        break;
    }
     // ---- LDR Night Detection ----
 
  int ldrValue = analogRead(LDR_PIN);  // Read LDR value (0 - 4095 on ESP32)
  Serial.println(ldrValue);
static bool nightMode = false;
if (ldrValue > 500 && !nightMode) {
  nightMode = true;
  digitalWrite(BLUE_LED, LOW);
  lcd.setCursor(0, 1);
  lcd.print("Night Mode ON ");
  logStreetLight("ON");
} else if (ldrValue <= 500 && nightMode) {
  nightMode = false;
  digitalWrite(BLUE_LED, HIGH);
  lcd.setCursor(0, 1);
  lcd.print("Day Mode       ");
  logStreetLight("OFF");
}


 

 
}
}

//Traffic Logic (FUNCTIONS)
void setTrafficState(TrafficState newState) {
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  currentState = newState;
  
  switch(currentState) {
  case STOP:
      digitalWrite(RED_LED, HIGH);
      writeServo(0);
      lcd.setCursor(0, 0);
      lcd.print("STOP        "); // Pad to avoid overwriting countdown
      currentTrafficId = logTrafficLight("RED", countdown);
      currentGateId = logGateStatus("CLOSED");
      break;

    case READY:
      digitalWrite(YELLOW_LED, HIGH);
      writeServo(0);
      lcd.setCursor(0, 0);
      lcd.print("READY       ");
      currentTrafficId = logTrafficLight("YELLOW", countdown);
      currentGateId = logGateStatus("CLOSED");
      break;

    case GO:
      digitalWrite(GREEN_LED, HIGH);
      writeServo(90);
      lcd.setCursor(0, 0);
      lcd.print("GO          ");
      currentTrafficId = logTrafficLight("GREEN", countdown);
      currentGateId = logGateStatus("OPEN");
      break;
  }

  
  // Clear any previous messages if no violation
  if (!violationDetected) {
    lcd.setCursor(0, 1);
    lcd.print("              ");
  }
}

void checkCarMovement() {
  int irValue = analogRead(IR_SENSOR_PIN);
 

  if (analogRead(IR_SENSOR_PIN) < 500) {
    if (!carDetected) {
      carDetected = true;
      Serial.println("Car detected!");
      
      if (currentState == STOP || currentState == READY) {
        violationDetected = true;
        digitalWrite(BUZZER_PIN, HIGH);
        writeServo(0);
        lcd.setCursor(0, 1);
        lcd.print("WAIT!!");

       int lightLevel = analogRead(LDR_PIN);
       String nightMode = (lightLevel > 500) ? "ON" : "OFF";
       sendViolation(nightMode);  
}

      else if (currentState == GO) {
        lcd.setCursor(0, 1);
        lcd.print("SAFE TO PASS  ");
      }
    }
  } else {
    if (carDetected) {
      carDetected = false;
      if (!violationDetected) {
        lcd.setCursor(0, 1);
        lcd.print("              ");
      }
    }
  }
}

void handleViolation() {
  static unsigned long violationStart = millis();
  
  if (millis() - violationStart >= 2000) {
    digitalWrite(BUZZER_PIN, LOW);
    violationDetected = false;
    lcd.setCursor(0, 1);
    lcd.print("              ");
    violationStart = millis(); // Reset timer
  }
}

void updateCountdown() {
  lcd.setCursor(12, 0); // Display countdown on top row, right side
  lcd.print("    ");     // Clear previous digits
  lcd.setCursor(12, 0);
  if (countdown < 10) lcd.print(" "); // Align single digits
  lcd.print(countdown);
  lcd.print("s");
}

