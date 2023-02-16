//****************************************//
//* ESP32 MQTT EEEBot Template           *//
//* Modified from Rui Santos'            *//
//* https://randomnerdtutorials.com      *//
//*                                      *//
//* ESP32 Code                           *//
//*                                      *//
//* UoN 2022 - ND                        *//
//****************************************//

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// Add your required sensor/component libraries here
// --
#include <HCSR04.h>
#include <MPU6050_tockn.h>
// --

// Replace the next variables with your SSID/Password combination
const char* ssid = "kennedyb19";       //CHANGE ME
const char* password = "purplereign";  //CHANGE ME

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.137.18";  //CHANGE ME

UltraSonicDistanceSensor distanceSensor(13, 12);
MPU6050 mpu6050(Wire, 1, 0.98);

String newmessage;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
float olddistance = -999;
const int ledPin = 14;

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Wire.begin();  // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(ledPin, OUTPUT);
}

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

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Add your subscribed topics here i.e. statements to control GPIOs with MQTT
  // --
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
  // --
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");

      // Add your subscribe topics here
      // --
      client.subscribe("esp32/output");
      
      // --

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void encoder(){  
  Wire.requestFrom(0x04, 4);  // request 6 bytes from slave device #8
    while (Wire.available()) {  // loop whilst slave sends data
                                // i.e. whilst bus is busy
      char c = Wire.read();     // receive data byte by byte
      if (c == '#') break;
      newmessage += c;  // form complete string
    }
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 500) {
    lastMsg = now;

    // Add your own code here i.e. sensor measurements, publish topics & subscribe topics for GPIO control
    // --
    Serial.println("Distance from obstacle(cm):");
    Serial.println(distanceSensor.measureDistanceCm());
    delay(500);
    char ultrasonicString[8];
    dtostrf(distanceSensor.measureDistanceCm(), 1, 2, ultrasonicString);
    client.publish("esp32/ultrasonic", ultrasonicString);

    mpu6050.update();
    Serial.print("gyroAngleZ : ");
    Serial.println(mpu6050.getGyroAngleZ());
    char orientationString[8];
    dtostrf(mpu6050.getGyroAngleZ(), 1, 2, orientationString);
    client.publish("esp32/orientation", orientationString);
  
    encoder();
    float distance = newmessage.toFloat();
    Serial.println("Distance travelled(cm):");
    Serial.println(distance);
    char encoderString[8];
    dtostrf(distance, 1, 2, encoderString);
    client.publish("esp32/encoder", encoderString); 
    Serial.println("Encoder count:");
    Serial.println(distance,0);
    char countString[8];
    dtostrf(distance, 1, 2, countString);
    client.publish("esp32/count", countString);
    delay(1000);
    
  }
}


