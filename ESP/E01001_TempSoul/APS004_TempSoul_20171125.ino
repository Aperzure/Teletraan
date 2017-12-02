/*
    This sketch sends a message to a TCP server

*/

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define DHTPin 2                //DHT VCC pin is connected to WEMOS PIN 4
#define DHTType DHT22           //Type of DHT used

#define mqtt_server "192.168.1.15"
#define mqtt_port "1883"
#define mqtt_username "mosquitto"
#define mqtt_password "mosquitto"


#define multisensor_topic "home/masterbed/dht"
#define humidity_subtopic "humidity"
#define temperature_subtopic "temperature"
#define heatindex_subtopic "heatIndex"

#define ssid "TELATRAAN2G"
#define password "$192168-X2412-2S"
const int BUFFER_SIZE = 300;

ESP8266WiFiMulti WiFiMulti;
WiFiClient client;
PubSubClient pubSubClient(client);

DHT dht(DHTPin, DHTType);       //Declare and initialize DHT object
float humidityReading;          //Humidity reading percentage
float temperatureFReading;      //Temperature reading in Fahrenheit
float temperatureCReading;      //Temperature reading in Celsius
float heatIndexReading;

int interval = 10000;
long duration = -180000;

void setup() {
  Serial.begin(115200);
  delay(10);

  WIFI_Connect();
  MQTT_Connect();
  Sensors_Initialize();


  delay(500);
}


void loop() {
  if (WiFiMulti.run() == WL_CONNECTED) {
    if (pubSubClient.connected()) {
      Sensors_Read();
    }
    else {
      MQTT_Connect();
    }
  }
  else {
    WIFI_Connect();
  }






  delay(6000);

}


void WIFI_Connect() {
  Serial.print("Connecting to WIFI (SSID): ");
  Serial.print(ssid);
  WiFiMulti.addAP(ssid, password);
  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

}


void MQTT_Connect() {

  pubSubClient.setServer(mqtt_server, 1883);
  // Loop until we're reconnected
  while (!pubSubClient.connected()) {
    Serial.print("Connecting to MQTT Server...");
    // Attempt to connect
    if (pubSubClient.connect("NOJI-1", mqtt_username, mqtt_password)) {
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
}


void Sensors_Read() {
  float newHumidityReading;          //Humidity reading percentage
  float newTemperatureFReading;      //Temperature reading in Fahrenheit
  float newTemperatureCReading;      //Temperature reading in Celsius
  float newHeatIndexReading;

  delay(100);
  int newDuration = millis();
  if (checkBound(newDuration, duration, interval)) {
    duration = newDuration;

    Serial.println("Analyzing Environment...");
    newHumidityReading = dht.readHumidity();                       //Read humidity percentage
    newTemperatureFReading = dht.readTemperature(true);            //Read temperature in Fahenheit (isFahrenheit = true)
    newTemperatureCReading = dht.readTemperature();                //Read temperature in Celsius (isFahrenheit = false)
    newHeatIndexReading = calculateHeatIndex(newHumidityReading, newTemperatureFReading);


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
  }
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
