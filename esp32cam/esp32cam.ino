#include <WiFi.h>
#include <MQTTClient.h>
#include "esp_camera.h"
#include <ArduinoJson.h>

// Update these with values suitable for your network.

const char* ssid = "........";
const char* password = "........";
const char* mqtt_server = "103.84.207.210";


// Pin untuk mengontrol motor kiri
int enA = 12;  // Pin PWM
int in1 = 13;
int in2 = 15;

// Pin untuk mengontrol motor kanan
int enB = 12;  // Pin PWM
int in3 = 14;
int in4 = 2;

int speedTank = 200;

void maju(int kecepatan) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  ledcWrite(6, kecepatan);  // PWM untuk motor kiri
}

void mundur(int kecepatan) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  ledcWrite(6, kecepatan);  // PWM untuk motor kiri
}

void belokKanan(int kecepatan) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  ledcWrite(6, kecepatan - 25);  // PWM untuk motor kiri
}

void belokKiri(int kecepatan) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  ledcWrite(6, kecepatan -25 );  // PWM untuk motor kiri
}

void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  ledcWrite(6, 0);  // PWM untuk motor kiri
}


#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define ESP32CAM_PUBLISH_TOPIC   "esp32/cam_0"

const int bufferSize = 1024 * 23; // 23552 bytes

WiFiClient espClient;
MQTTClient client = MQTTClient(bufferSize);


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  
}

void callback(String& topic, String& payload) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  if (topic == "/tank") {
    // Parse the payload for this specific topic
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload);

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
  } else if (topic == "/cam") {
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
    String clientId = "ESPCAMClient-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("/tank");
    } else {
      Serial.print("failed, rc=");
      //Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void cameraInit(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA; // 640x480
  config.jpeg_quality = 10;
  config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
    return;
  }
}


void grabImage(){
  camera_fb_t * fb = esp_camera_fb_get();
  if(fb != NULL && fb->format == PIXFORMAT_JPEG && fb->len < bufferSize){
    Serial.print("Image Length: ");
    Serial.print(fb->len);
    Serial.print("\t Publish Image: ");
    bool result = client.publish(ESP32CAM_PUBLISH_TOPIC, (const char*)fb->buf, fb->len);
    Serial.println(result);

    if(!result){
      ESP.restart();
    }
  }
  esp_camera_fb_return(fb);
  delay(1);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setup_wifi();
  cameraInit();
  client.begin("103.84.207.210", 1883, espClient);
  client.setCleanSession(true);
  client.onMessage(callback);


  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  

  // Konfigurasi PWM untuk motor kiri dan kanan
  ledcSetup(6, 5000, 8);  // Kanal 0, frekuensi 5000Hz, resolusi 8-bit
  ledcAttachPin(enA, 6);  // Hubungkan PWM ke pin enA

  stop();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if(client.connected()) grabImage();
}
