#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include <MQUnifiedsensor.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


Servo pan;
Servo tilt;

#define panPin 18
#define tiltPin 15

#define STARTANGLEPAN 90
#define STARTANGLETILT 90

#define Board ("ESP-32")
#define PinMQ (36)
#define Type ("MQ-2")
#define Voltage_Resolution (3.3)
#define ADC_Bit_Resolution (12)
#define RatioMQ2CleanAir (9.83)

MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, PinMQ, Type);

#define DHTPIN 32

#define DHTTYPE DHT11  // DHT 11


DHT dht(DHTPIN, DHTTYPE);


int dircam = 0;

DynamicJsonDocument doc(1024);


unsigned long previousMillis = 0;  // will store last time LED was updated

// constants won't change:
const long interval = 1000;

// Update these with values suitable for your network.

const char* ssid = "KOWI";
const char* password = "kremi201";
const char* mqtt_server = "103.84.207.210";


// Pin untuk mengontrol motor kiri
int enA = 13;  // Pin PWM
int in1 = 12;
int in2 = 14;

// Pin untuk mengontrol motor kanan
int enB = 25;  // Pin PWM
int in3 = 27;
int in4 = 26;

int speedTank = 200;

int speed = 15;

void maju(int kecepatan) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  ledcWrite(6, kecepatan);  // PWM untuk motor kiri
  ledcWrite(7, kecepatan);  // PWM untuk motor kanan
}

void mundur(int kecepatan) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  ledcWrite(6, kecepatan);  // PWM untuk motor kiri
  ledcWrite(7, kecepatan);  // PWM untuk motor kanan
}

void belokKanan(int kecepatan) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  ledcWrite(6, kecepatan - 25);  // PWM untuk motor kiri
  ledcWrite(7, kecepatan - 25);  // PWM untuk motor kanan
}

void belokKiri(int kecepatan) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  ledcWrite(6, kecepatan - 25);  // PWM untuk motor kiri
  ledcWrite(7, kecepatan - 25);  // PWM untuk motor kanan
}

void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  ledcWrite(6, 0);  // PWM untuk motor kiri
  ledcWrite(7, 0);  // PWM untuk motor kanan
}


WiFiClient espclient;
PubSubClient client(espclient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to String
  String payloadStr;
  for (int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  // Convert topic to String
  String topicStr = String(topic);

  Serial.print("Message arrived [");
  Serial.print(topicStr);
  Serial.print("] ");
  Serial.print(payloadStr);
  Serial.println();

  if (topicStr == "/tankctl") {
    // Parse the payload for this specific topic
    StaticJsonDocument<200> doctank;
    DeserializationError error = deserializeJson(doctank, payloadStr);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    int x = doctank["x"];
    int y = doctank["y"];

    if (y < 0) {
      maju(speedTank);
      Serial.println("Maju");
    } else if (y > 0) {
      mundur(speedTank);
      Serial.println("Mundur");
    } else if (x > 0) {
      belokKanan(speedTank);
      Serial.println("Belok Kanan");
    } else if (x < 0) {
      belokKiri(speedTank);
      Serial.println("Belok Kiri");
    } else {
      stop();
      Serial.println("STOP");
    }
  } else if (topicStr == "/camctl") {
    StaticJsonDocument<200> camctl;
    DeserializationError error = deserializeJson(camctl, payloadStr);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    int xcam = camctl["x"];
    int ycam = camctl["y"];

    if (ycam < 0) {
      dircam = 1;
      Serial.println("Maju");
    } else if (ycam > 0) {
      dircam = 2;
      Serial.println("Mundur");
    } else if (xcam > 0) {
      dircam = 3;
      Serial.println("Belok Kanan");
    } else if (xcam < 0) {
      dircam = 4;
      Serial.println("Belok Kiri");
    } else {
      dircam = 0;
      Serial.println("STOP");
    }
  } else if (topicStr == "/camspeedctl") {
    StaticJsonDocument<200> camspeedctl;
    DeserializationError error = deserializeJson(camspeedctl, payloadStr);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    int camspeed = int(camspeedctl["camspeed"]);
    speed = map(camspeed, 0, 100, 100, 0);

  }
  else if (topicStr == "/tankspeedctl") {
    StaticJsonDocument<200> tankspeedctl;
    DeserializationError error = deserializeJson(tankspeedctl, payloadStr);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    int tankspeed = int(tankspeedctl["tankspeed"]);
    speedTank = map(tankspeed, 0, 100, 255, 200);

  }

  else {
    Serial.println("Unknown topic");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP#Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("/tankctl");
      client.subscribe("/camctl");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

int pos1 = 0;  // variable to store the servo position
int pos2 = 0;  // variable to store the servo position

               // adjust this value to control the speed of the servo
unsigned long lastUpdate1 = 0;  // when the servo position was last updated
unsigned long lastUpdate2 = 0;  // when the servo position was last updated




void setup() {
  Serial.begin(115200);
  setup_wifi();
  delay(500);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);


  // Konfigurasi PWM untuk motor kiri dan kanan
  ledcSetup(6, 5000, 8);  // Kanal 0, frekuensi 5000Hz, resolusi 8-bit
  ledcSetup(7, 5000, 8);  // Kanal 1, frekuensi 5000Hz, resolusi 8-bit
  ledcAttachPin(enA, 6);  // Hubungkan PWM ke pin enA
  ledcAttachPin(enB, 7);  // Hubungkan PWM ke pin enB


  stop();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  pan.setPeriodHertz(50);  // Standard 50hz servo
  tilt.setPeriodHertz(50);
  pan.attach(panPin, 500, 2400);
  tilt.attach(tiltPin, 500, 2400);

  pan.write(STARTANGLEPAN);
  delay(1000);
  tilt.write(STARTANGLETILT);
  delay(1000);

  dht.begin();
  MQ2.setRegressionMethod(1);  //_PPM =  a*ratio^b
  MQ2.setA(987.99);
  MQ2.setB(-2.162);
  MQ2.init();

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ2.update();  // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0)) {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1)
      ;
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1)
      ;
  }


  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("Robot Tank");
  display.display(); 
}

void loop() {

  if (!client.connected()) {
    reconnect();
  } else {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      float t = dht.readTemperature();

      if (isnan(t)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
      }
      Serial.print(t);
      Serial.print(F(" °C "));

      Serial.print(" | ");

      MQ2.update();

      int gass = MQ2.readSensor();

      Serial.print(gass);
      Serial.println(" PPM");

      doc["temperature"] = t;
      doc["gass"] = gass;


      String jsonStr;
      serializeJson(doc, jsonStr);

      client.publish("/sensor", jsonStr.c_str());
      display.setCursor(0, 0);
      display.println("Temperature: " + String(t));
      display.setCursor(0, 1);
      display.println("Gass: " + String(gass));

      display.display();

      previousMillis = currentMillis;
    }
  }

  if (dircam != 0) {
    if (dircam == 1 && millis() - lastUpdate1 > speed) {
      pos1 = min(180, pos1 + 1);  // increase the servo1 position
      pan.write(pos1);
      lastUpdate1 = millis();  // update the time
      Serial.println("DIR CAM UP");
      dircam = 0;
    } else if (dircam == 2 && millis() - lastUpdate1 > speed) {
      pos1 = max(0, pos1 - 1);  // decrease the servo1 position
      pan.write(pos1);
      lastUpdate1 = millis();  // update the time
      Serial.println("DIR CAM DOWN");
      dircam = 0;
    } else if (dircam == 3 && millis() - lastUpdate2 > speed) {
      pos2 = max(0, pos2 - 1);  // decrease the servo2 position
      tilt.write(pos2);
      lastUpdate2 = millis();  // update the time
      Serial.println("DIR CAM LEFT");
      dircam = 0;
    } else if (dircam == 4 && millis() - lastUpdate2 > speed) {
      pos2 = min(180, pos2 + 1);  // increase the servo2 position
      tilt.write(pos2);
      lastUpdate2 = millis();  // update the time
      Serial.println("DIR CAM RIGHT");
      dircam = 0;
    }
  }
  client.loop();
}
