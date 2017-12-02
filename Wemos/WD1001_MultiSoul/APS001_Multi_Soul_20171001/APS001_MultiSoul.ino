#include <DHT.h>
#include<WiFiClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h> //FOR HTTPClient Object
#include <PubSubClient.h>
#include <ArduinoJson.h>


#define ON 1
#define OFF 0

#define DHTPin 4                //DHT VCC pin is connected to WEMOS PIN 4
#define DHTType DHT11           //Type of DHT used
#define SSPin A0                //Smoke Sensor
#define FSPin D5              //Flame Sensor
#define PIRPin 5
#define LEDPin 16
#define BuzzerPin 0

#define mqtt_server "192.168.1.15"
#define mqtt_username "<mosquitto username>"
#define mqtt_password "<mosquitto password>"

#define multisensor_topic "home/multisensor"
#define humidity_subtopic "humidity"
#define temperature_subtopic "temperature"
#define heatindex_subtopic "heatIndex"
#define smoke_subtopic "smoke"
#define flame_subtopic "flame"
#define motion_subtopic "motion"

#define ssid "<ssid>"
#define password "<password>"
const int BUFFER_SIZE = 300;

IPAddress server(192, 168, 0, 27);
WiFiClient client;
PubSubClient pubSubClient(client);


int interval = 180000;

DHT dht(DHTPin, DHTType);       //Declare and initialize DHT object
float humidityReading;          //Humidity reading percentage
float temperatureFReading;      //Temperature reading in Fahrenheit
float temperatureCReading;      //Temperature reading in Celsius
float heatIndexReading;
float smokeSensorReading = 0.0;
float flameSensorReading = 0.0;

int pirValue;                   //PIR reading
int pirStatus;
String motionStatus;
int buzzerSwitch = OFF;

long duration = -180000;


void setup() {
  Serial.begin(115200);
  delay(2);

  // Connect to WiFi network
  Wifi_Connect();

  pubSubClient.setServer(mqtt_server, 1883);

  Sensors_Initialize();
}

void loop() {
  if (!pubSubClient.connected()) {
    MQTT_Connect();
  }
  pubSubClient.loop();

  Sensors_Read();

  //delay(600000);
  //delay(180000);
  delay(300);




}

void Connect() {
  Wifi_Connect();
}

void Wifi_Connect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.println();
    Serial.print("Connecting to WIFI (SSID): ");
    Serial.print(ssid);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("  WiFi Connected: ");
    Serial.println(WiFi.localIP());
  }
}


void MQTT_Connect() {
  // Loop until we're reconnected
  while (!pubSubClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (pubSubClient.connect("SAWTOOTH-1", mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(pubSubClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void Sensors_Initialize() {
  Serial.println("Initializing Sensors");
  dht.begin();

  pinMode(LEDPin, OUTPUT);
  pinMode(PIRPin, INPUT);
  pinMode(FSPin, INPUT);
  //pinMode(BuzzerPin, OUTPUT);

  digitalWrite(LEDPin, LOW);
  digitalWrite(BuzzerPin, HIGH);
}

void Sensors_Read() {
  float newHumidityReading;          //Humidity reading percentage
  float newTemperatureFReading;      //Temperature reading in Fahrenheit
  float newTemperatureCReading;      //Temperature reading in Celsius
  float newHeatIndexReading;
  float newSmokeSensorReading = 0.0;
  float newFlameSensorReading = 0.0;
  int newPirReading = 0;
  int newDuration = millis();

  pirValue = digitalRead(PIRPin);
  digitalWrite(LEDPin, pirValue);

  if (pirValue == LOW && pirStatus != 0) {
    motionStatus = "Standby";
    Publish(motion_subtopic, (String)motionStatus);
    pirStatus = 0;
  }
  else if (pirValue == HIGH && pirStatus != 1) {
    motionStatus = "Motion In-Progress";
    Publish(motion_subtopic, (String)motionStatus);
    pirStatus = 1;
  }

  //  if (buzzerSwitch == ON) {
  //      if(pirValue == HIGH) {
  //        digitalWrite(BuzzerPin, LOW);
  //        tone(BuzzerPin, 400);
  //        delay(500);
  //        tone(BuzzerPin, 800); // play 800Hz tone for 500ms
  //        delay(500);
  //      }
  //      else {
  //        digitalWrite(BuzzerPin, HIGH);
  //      }
  //  }
  delay(100);

  newFlameSensorReading = digitalRead(FSPin);                     //Read flame sensor
  if (checkBound(newFlameSensorReading, flameSensorReading, 0.5)) {
    flameSensorReading = newFlameSensorReading;
    Serial.print("New Flame Sensor Reading:");
    Serial.println(String(flameSensorReading).c_str());
    Publish(flame_subtopic, (String)flameSensorReading);
  }

  Serial.println(newDuration);
  if (checkBound(newDuration, duration, interval)) {
    duration = newDuration;

    Serial.println("Analyzing Environment...");
    newHumidityReading = dht.readHumidity();                       //Read humidity percentage
    newTemperatureFReading = dht.readTemperature(true);            //Read temperature in Fahenheit (isFahrenheit = true)
    newTemperatureCReading = dht.readTemperature();                //Read temperature in Celsius (isFahrenheit = false)
    newHeatIndexReading = calculateHeatIndex(newHumidityReading, newTemperatureFReading);
    newSmokeSensorReading = analogRead(SSPin);                   //Read smoke sensor
    //newFlameSensorReading = digitalRead(FSPin);                     //Read flame sensor

    //Check if any reads failed and exit early (to try again)
    if (isnan(humidityReading) ||  isnan(temperatureFReading) || isnan(temperatureCReading)) {
      Serial.println("  Failed to read from DHT sensor!");
      return;
    }

    if (checkBound(newTemperatureCReading, temperatureCReading, 1)) {
      temperatureCReading = newTemperatureCReading;
      Serial.print("New Temperature:");
      Serial.println(String(temperatureCReading).c_str());
      Publish(temperature_subtopic, (String)temperatureCReading);
    }

    if (checkBound(newHumidityReading, humidityReading, 1)) {
      humidityReading = newHumidityReading;
      Serial.print("New Humidity:");
      Serial.println(String(humidityReading).c_str());
      Publish(humidity_subtopic, (String)humidityReading);
    }

    if (checkBound(newHeatIndexReading, heatIndexReading, 1)) {
      heatIndexReading = newHeatIndexReading;
      Serial.print("New Heat Index:");
      Serial.println(String(heatIndexReading).c_str());
      Publish(heatindex_subtopic, (String)heatIndexReading);
    }

    if (checkBound(newSmokeSensorReading, smokeSensorReading, 1)) {
      smokeSensorReading = newSmokeSensorReading;
      Serial.print("New Smoke Reading:");
      Serial.println(String(smokeSensorReading).c_str());
      Publish(smoke_subtopic, (String)smokeSensorReading);
    }
  }
}

void Publish() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  root["temperature"] = (String)temperatureCReading;
  root["humidity"] = (String)humidityReading;
  root["heatIndex"] = (String)heatIndexReading;
  root["smoke"] = (String)smokeSensorReading;
  //root["motion"] = (String)motionStatus;

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  pubSubClient.publish(multisensor_topic, buffer, true);
}

void Publish(String p_strNode, String p_strValue) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  root[p_strNode] = p_strValue;

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  pubSubClient.publish(multisensor_topic, buffer, true);
}


bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

float calculateHeatIndex(float p_fltHumidity, float p_fltTemperature) {
  float l_fltHeatIndex = 0;
  if (p_fltTemperature >= 80) {
    l_fltHeatIndex = -42.379 + 2.04901523 * p_fltTemperature + 10.14333127 * p_fltHumidity;
    l_fltHeatIndex = l_fltHeatIndex - .22475541 * p_fltTemperature * p_fltHumidity - .00683783 * p_fltTemperature * p_fltTemperature;
    l_fltHeatIndex = l_fltHeatIndex - .05481717 * p_fltHumidity * p_fltHumidity + .00122874 * p_fltTemperature * p_fltTemperature * p_fltHumidity;
    l_fltHeatIndex = l_fltHeatIndex + .00085282 * p_fltTemperature * p_fltHumidity * p_fltHumidity - .00000199 * p_fltTemperature * p_fltTemperature * p_fltHumidity * p_fltHumidity;
  } else {
    l_fltHeatIndex = 0.5 * (p_fltTemperature + 61.0 + ((p_fltTemperature - 68.0) * 1.2) + (p_fltHumidity * 0.094));
  }

  if (p_fltHumidity < 13 && 80 <= p_fltTemperature <= 112) {
    float adjustment = ((13 - p_fltHumidity) / 4) * sqrt((17 - abs(p_fltTemperature - 95.)) / 17);
    l_fltHeatIndex = l_fltHeatIndex - adjustment;
  }

  return l_fltHeatIndex;
}
