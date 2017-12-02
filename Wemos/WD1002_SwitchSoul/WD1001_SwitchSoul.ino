#include<WiFiClient.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

void callback(char* topic, byte* payload, unsigned int length);

#define SWITCHPIN1 4      //Relay Pin D2
#define SWITCHPIN2 5      //Relay Pin D3
#define SWITCHPIN3 14     //Relay Pin D4
#define SWITCHPIN4 16     //Relay Pin D5

#define SWITCHCOMMAND1 "/home/switch1/"
#define SWITCHCOMMAND2 "/home/switch2/"
#define SWITCHCOMMAND3 "/home/switch3/"
#define SWITCHCOMMAND4 "/home/switch4/"

#define SWITCHSTATE1 "/home/state1/"
#define SWITCHSTATE2 "/home/state2/"
#define SWITCHSTATE3 "/home/state3/"
#define SWITCHSTATE4 "/home/state4/"

#define WIFI_SSID "<wifi ssid>"
#define WIFI_PASSWORD "<wifi password>"
#define BUFFER_SIZE 300

#define MQTT_SERVER "192.168.1.15"
#define MQTT_USERNAME "<mosquitto usernames>"
#define MQTT_PASSWORD "<mosquitto password>"

WiFiClient wifiClient;
PubSubClient pubSubClient(MQTT_SERVER, 1883, callback, wifiClient);

void setup() {
  Serial.begin(115200);
  delay(2);

  Serial.print("Starting...");
  //Give your ESP8266 module a name when discovering it as a port in ARDUINO IDE, the initialize OTA
  ArduinoOTA.setHostname("SLIPSTREAM-01");
  ArduinoOTA.begin();

  //Initialize the relay switches pin
  pinMode(SWITCHPIN1, OUTPUT);
  pinMode(SWITCHPIN2, OUTPUT);
  pinMode(SWITCHPIN3, OUTPUT);
  pinMode(SWITCHPIN4, OUTPUT);

  //Turn the relay switches off
  digitalWrite(SWITCHPIN1, HIGH);
  digitalWrite(SWITCHPIN2, HIGH);
  digitalWrite(SWITCHPIN3, HIGH);
  digitalWrite(SWITCHPIN4, HIGH);

  //Attempt to connect to Wifi and MQTT, then wait a bit
  Wifi_Connect();
  MQTT_Connect();
  delay(500);
}

void loop() {
  //Connect/reconnect to WiFi and MQTT if connecgtion is lost
  Reconnect();

  //Maintain MQTT connection, then wait to allow ESP8266 WIFI functions to run
  pubSubClient.loop();
  delay(10);
  //ArduinoOTA.handle();
}
//
void Reconnect() {
  if (!pubSubClient.connected() || WiFi.status() == 3) {
    Wifi_Connect();
    MQTT_Connect();
  }
}

void Wifi_Connect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.println();
    Serial.print("Connecting to WIFI (SSID): ");
    Serial.print(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("\tWiFi Connected: ");
    Serial.println(WiFi.localIP());
  }
}


void MQTT_Connect() {
  //Make sure we are connected to WIFI before attemping to reconnect to MQTT
  if (WiFi.status() == WL_CONNECTED) {
    // Loop until we're reconnected to the MQTT server
    while (!pubSubClient.connected()) {
      Serial.println("Attempting MQTT connection...");
      // Attempt to connect
      if (pubSubClient.connect("SLIPSTREAM-01", MQTT_USERNAME, MQTT_PASSWORD)) {
        Serial.println("\tConnected");
        pubSubClient.subscribe(SWITCHCOMMAND1);
        Serial.println("\Listening");
        pubSubClient.subscribe(SWITCHCOMMAND2);
        pubSubClient.subscribe(SWITCHCOMMAND3);
        pubSubClient.subscribe(SWITCHCOMMAND4);
      } else {
        Serial.print("\tFailed, rc=");
        Serial.print(pubSubClient.state());
        Serial.println(" Try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  //Convert topic to string to make it easier to work with
  String topicStr = topic;
  //Note:  the "topic" value gets overwritten everytime it receives confirmation (callback) message from MQTT

  //Print out some debugging info
  Serial.println("Callback update.");
  Serial.print("Topic: ");
  Serial.println(topicStr);

  if (topicStr == SWITCHCOMMAND1)
  {
    //Turn the switch on if the payload is '1' and publish to the MQTT server a confirmation message
    if (payload[0] == '1') {
      digitalWrite(SWITCHPIN1, LOW);
      pubSubClient.publish(SWITCHSTATE1, "1");
    }

    //Turn the switch off if the payload is '0' and publish to the MQTT server a confirmation message
    else if (payload[0] == '0') {
      digitalWrite(SWITCHPIN1, HIGH);
      pubSubClient.publish(SWITCHSTATE1, "0");
    }
  }
  else if (topicStr == SWITCHCOMMAND2)
  {
    if (payload[0] == '1') {
      digitalWrite(SWITCHPIN2, LOW);
      pubSubClient.publish(SWITCHSTATE2, "1");
    }
    else if (payload[0] == '0') {
      digitalWrite(SWITCHPIN2, HIGH);
      pubSubClient.publish(SWITCHSTATE2, "0");
    }
  }
  else if (topicStr == SWITCHCOMMAND3)
  {
    if (payload[0] == '1') {
      digitalWrite(SWITCHPIN3, LOW);
      pubSubClient.publish(SWITCHSTATE3, "1");
    }
    else if (payload[0] == '0') {
      digitalWrite(SWITCHPIN3, HIGH);
      pubSubClient.publish(SWITCHSTATE3, "0");
    }
  }
  else if (topicStr == SWITCHCOMMAND4)
  {
    if (payload[0] == '1') {
      digitalWrite(SWITCHPIN4, LOW);
      pubSubClient.publish(SWITCHSTATE4, "1");
    }
    else if (payload[0] == '0') {
      digitalWrite(SWITCHPIN4, HIGH);
      pubSubClient.publish(SWITCHSTATE4, "0");
    }
  }
}
