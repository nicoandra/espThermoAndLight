/**

Read the docs of Server for a TCP one here:

https://github.com/esp8266/Arduino/blob/master/doc/esp8266wifi/server-class.md
https://github.com/esp8266/Arduino/blob/master/doc/esp8266wifi/server-examples.md
https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WebServer/examples/WebUpdate/WebUpdate.ino

**/

#include <FS.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "Arduino.h"
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include "../lib/PirHcSr501/PirHcSr501.h"


// Thermostat functionality
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "../lib/ThermoLogic/ThermoLogic.h"


// MQTT settings
char mqtt_server[40] = "192.168.1.106";
char mqtt_port[6] = "1883";
char device_name[32] = "ESP Dimmer";

WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
WiFiManagerParameter custom_device_name("device_name", "device name", device_name, 32);

const int SUB_BUFFER_SIZE = JSON_OBJECT_SIZE(4) + JSON_ARRAY_SIZE(10);
bool shouldSaveConfig = false;  //flag for saving data

// Used later on for dynamically creating a Json representation of an object
DynamicJsonBuffer _buffer;

// Setup DHT22 instances (sensor data pin, sensor type, relay pin) in a fancy array
ThermoLogic thermos[3] = {
  ThermoLogic(D1, DHT22, D2),
  ThermoLogic(D3, DHT22, D4),
  ThermoLogic(D5, DHT22, D6)
};

PinHcSr501 Sensor1(D7);
PinHcSr501 Sensor2(D8);

WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;


/* Internal variables */
long lastAnnounceTime = 0;
int allowAnnounce = 0;

void resetOnDemand(){

  if(!digitalRead(D0)){
    return ;
  }


  if(millis() > 5000){
    // Reset will be taken into account during the first 5 seconds
    return ;
  }

  Serial.println("Resetting everything");
  wifiManager.resetSettings();
  SPIFFS.format();
  Serial.println("Reboot in 2 seconds");
  delay(2000);
  ESP.restart();
}

bool doReadConfig(){
  if (!SPIFFS.exists("/config.json")) {
    // If Config file does not exist Force the Config page
    Serial.println("doReadConfig: config file does not exist");
    return false;
  }

  Serial.println("Reading config file");
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to load json config file");
    // Force the Config page
    return false;
  }

  Serial.println("Opened config file");
  size_t size = configFile.size();
  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  configFile.readBytes(buf.get(), size);
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());
  json.printTo(Serial);
  Serial.println(" ...");
  if (!json.success()) {
    Serial.println("Failed to parse json config file");
    // Force the Config page
    return false;
  }

  strcpy(mqtt_server, json["mqtt_server"]);
  strcpy(mqtt_port, json["mqtt_port"]);
  strcpy(device_name, json["device_name"]);
  Serial.println("Json parsed and configuration loaded from file.");
}

void doSaveConfig(){
  //save the custom parameters to FS
  if (!shouldSaveConfig) {
    return ;
  }
  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(device_name, custom_device_name.getValue());

  Serial.println("Saving config");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["device_name"] = device_name;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
    return ;
  }

  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();
  //end save
  Serial.println("Config Saved! Wait 5 seconds");
  delay(5000);
  ESP.restart();
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
  doSaveConfig();
}

String macAddress() {
  String formatted = "";
  char mac_address[20];
  WiFi.macAddress().toCharArray(mac_address, 20);
  for(int i = 0; i < 17; i++){
    if(i == 2 || i == 5 || i == 8 || i == 11 || i == 14){
      continue;
    }
    formatted = formatted + mac_address[i];
  }
  return formatted;
}

void announce(){
  char* key = (char*) _buffer.alloc(4);

  if( allowAnnounce == 0){
    return ;
  }
  allowAnnounce = 0;

  String mac_address = macAddress();
  char message[4096];
  DynamicJsonBuffer jsonBufferPub;

  JsonObject& json = jsonBufferPub.createObject();

  Serial.println(mac_address);
  json["mac_address"] = mac_address;
  json["device_name"] = device_name;

  JsonObject& heaters = json.createNestedObject('h');

  for(int i = 0; i < sizeof(thermos); i++){
    // As per https://github.com/bblanchon/ArduinoJson/issues/87
    sprintf(key, "%d", i);
    JsonObject& thisThermo = heaters.createNestedObject(key);
    thisThermo["actualTemperature"] = thermos[i].getTemperature();
    thisThermo["humidity"] = thermos[i].getHumidity();
    thisThermo["power"] = thermos[i].getPower();
  }

  json.printTo(message);
  Serial.print("Publish message: ");
  Serial.println(message);
  client.publish("/device/announcement", message);
  allowAnnounce = 1;
}


void reportMovement(int sensorId){
  Serial.println("A MOVOEMENT WILL BE REPORTED. TODO.");
}


void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());

  Serial.println(myWiFiManager->getConfigPortalSSID());
}



void handleMovementDetection(){

  if(Sensor1.movementDetected()){
     reportMovement(1);
  }

  if(Sensor2.movementDetected()){
    reportMovement(2);
  }

  return ;

}





void mqttMessageCallback(char* topicParam, byte* payloadParam, unsigned int length) {
  allowAnnounce = 0;
  /*char topic[255];
  strcpy(topic, topicParam);
  */
  if( length > 1024 ){
    Serial.println("Message received, but longer than the max allowed length. Exiting.");
    return ;
  }

  char payload[1024] = "";
  strncpy(payload, (char *)payloadParam, length);
  Serial.print("Payload: ");
  Serial.print(payload);


  StaticJsonBuffer<SUB_BUFFER_SIZE> jsonBufferSub;
  JsonObject& jsonPayload = jsonBufferSub.parseObject(payload);
  Serial.print(" - Parsed: ");
  jsonPayload.printTo(Serial);
  Serial.println("***");

  // TODO:: READ WHO SENT THE MESSAGE. IF IT'S MYSELF, IGNORE IT.
  if( ((String) topicParam).startsWith("/heaters/")) {
    JsonVariant sender = jsonPayload["mac_address"];
    if(!sender.success()){
      Serial.print("No sender. Reject.");
      return ;
    }
    /*
    Serial.print("My MAC: ");
    Serial.println(macAddress());
    Serial.print("Sender: ");
    Serial.println(sender.as<char*>());
    */

    if(macAddress() == sender.as<char*>()){
      Serial.print("My own message. Reject.");
      return ;
    }

    Serial.print("Good message from ");
    Serial.println(sender.as<char*>());
    JsonArray& values = jsonPayload["heaters"];

    if(!values.success()){
      Serial.println("No lights section.");
      return ;
    }

    /*
    for(int i = 0; i < values.size(); i++){
      channels[i] = values[i];
    }
    */
    return ;
  }

  Serial.print("Not a /heaters/ topic. Received in topic ");
  Serial.print(topicParam);

  allowAnnounce = 1;
  return ;
}


void reconnect() {
  // Loop until we're reconnected
  // String topic = "/controllers/" + WiFi.macAddress();

  char mac_address[20];
  WiFi.macAddress().toCharArray(mac_address, 20);


  int tryes = 0;
  while (!client.connected() && tryes < 10) {
    Serial.print("Attempting MQTT connection...");
    tryes++;
    // Attempt to connect
    if (client.connect( mac_address )) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("/device/announcement", "ESPHeater");
      // ... and resubscribe
      String topic = "/heaters/";
      char topicBuffer[255];
      topic.toCharArray(topicBuffer, 255);
      client.subscribe(topicBuffer);

      topic = "/heaters/" + (String) macAddress();
      topic.toCharArray(topicBuffer, 255);
      client.subscribe(topicBuffer);

    } else {

      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try #" + String(tryes) + " again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1600);
    }

  }
}




void cycle(){
  resetOnDemand();

  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  long now = millis();
  if (now - lastAnnounceTime > 10000 || now < lastAnnounceTime) {
    lastAnnounceTime = now;
    announce();
  }

}

void loop(){
  cycle();

  bool doPrint = millis() % 5000 == 0;

  for(int i = 0; i < sizeof(thermos); i++){
    thermos[i].readSensorValues();
    thermos[i].calculatePower();
    thermos[i].writePwmValues();
    if(doPrint){
      thermos[i].printValues();
    }

  }

  handleMovementDetection();

}

void setup() {
  Serial.begin(115200);
  Serial.print("ESP WiFi Heater controller and movement sensor. MAC: ");
  Serial.println(WiFi.macAddress());

  pinMode(D0, INPUT);
  digitalWrite(D0, LOW);

  if (!SPIFFS.begin()) {
    Serial.println("Failed to mount FS in Setup.");
    delay(10000);
    ESP.restart();
    return ;
  }
  Serial.println("Mounted file system.");

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_device_name);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setAPCallback(configModeCallback);

  resetOnDemand();

  while(!wifiManager.autoConnect("AutoConnectAP", "password")){
    Serial.println("Can not connect to WiFi.");
    delay(2000);
  }

  Serial.println("Wifi up. Try to load device settings from JSON");

  if(!doReadConfig()){
    Serial.println("Either can not read config, or can not connect. Loading portal!");
    wifiManager.startConfigPortal("OnDemandAP");
    Serial.println("Portal loaded");
    return;
  }

  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttMessageCallback);

  Sensor1.setDelay(2000);
  Sensor2.setDelay(2000);

}

/*

void setup()
{
  // Open Serial for Output
  Serial.begin(115200);
  Serial.println("Booting...");

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  ledOff(LED_BUILTIN);

  pinMode(D2, INPUT); // For the button
  digitalWrite(D2, HIGH);

  pinMode(movementSensorDataPin, INPUT); // For the movement detector sensor
  digitalWrite(movementSensorDataPin, LOW);

  pinMode(ledOnOfPin, OUTPUT);

  // Network settings
  WiFiManager wifiManager;
  // wifiManager.autoConnect("AP-NAME", "AP-PASSWORD");
  wifiManager.autoConnect();
  Serial.println("Booted");

  if(connectUDP()) {
    Serial.println("Listening UDP");
    listenUdp();
  }
}
*/
