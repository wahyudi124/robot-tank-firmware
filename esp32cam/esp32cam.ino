#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Update these with values suitable for your network.

const char* ssid = "KOWI";
const char* password = "kremi201";
const char* mqtt_server = "103.84.207.210";


// Pin untuk mengontrol motor kiri
int enA = 12;  // Pin PWM
int in1 = 13;
int in2 = 15;

// Pin untuk mengontrol motor kanan
int enB = 0;  // Pin PWM
int in3 = 14;
int in4 = 2;

int speedTank = 200;

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
  ledcWrite(7, kecepatan - 25 );  // PWM untuk motor kanan
}

void belokKiri(int kecepatan) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  ledcWrite(6, kecepatan -25 );  // PWM untuk motor kiri
  ledcWrite(7, kecepatan -25 );  // PWM untuk motor kanan
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
#define MSG_BUFFER_SIZE	(50)
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

  if (topicStr == "/tank") {
    // Parse the payload for this specific topic
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payloadStr);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    int x = doc["x"];
    int y = doc["y"];

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
  } else if (topicStr == "/cam") {
    // Handle another specific topic
    // ...
  } else {
    Serial.println("Unknown topic");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("/tank");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

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

}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
