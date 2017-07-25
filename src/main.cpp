/**

Read the docs of Server for a TCP one here:

https://github.com/esp8266/Arduino/blob/master/doc/esp8266wifi/server-class.md
https://github.com/esp8266/Arduino/blob/master/doc/esp8266wifi/server-examples.md
https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WebServer/examples/WebUpdate/WebUpdate.ino

**/


#include "Arduino.h"
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

// Thermostat functionality
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "../lib/ThermoLogic/ThermoLogic.h" // To read temperatures
#include "../lib/PWMPin/PWMPin.h" // To read write analog values

unsigned int localPort = 8888;
WiFiUDP UDP;
boolean udpConnected = false;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
// char ReplyBuffer[] = "Acknowledged";
char replyBuffer[32];


// Setup a DHT22 instance
ThermoLogic livingThermoLogic(D6, DHT22, D3);

// Setup the button reader
int ButtonState      = 0;
int PrevButtonState = 0;
unsigned long TimeWhenButtonWasPressed = millis();
bool IsALongPress = false;

uint32_t timeOfLastStream = 0;
uint32_t timeOfLastMovement = 0;

int movementSensorDataPin = D8;


int ledOnOfPin = D7;
int ledPower = 0;

boolean connectUDP(){
  boolean state = false;
  Serial.println("");
  Serial.println("Connecting to UDP");

  if(UDP.begin(localPort) == 1){
    Serial.println("Connection successful");
    state = true;
  } else {
    Serial.println("Connection failed");
  }

  return state;
}

void sendUdpBuffer(IPAddress host, int port, char buffer[], int length){
    UDP.beginPacket(host, port);
    UDP.write(buffer, length);
    UDP.endPacket();
}

void sendUdpBufferResponse(char buffer[], int length){
  sendUdpBuffer(UDP.remoteIP(), UDP.remotePort(), buffer, length);
}

void sendStatusResponse(IPAddress ip, int port){
  char responseReplyBuffer[256];

  responseReplyBuffer[0] = 0x31;
  responseReplyBuffer[1] = 0xFF;
  responseReplyBuffer[2] = 0x01;  // Set real count of heaters here. Hardcoded to 1 now.

  responseReplyBuffer[3] = (int) livingThermoLogic.getTemperature();
  responseReplyBuffer[4] = (int) ((livingThermoLogic.getTemperature() - (int) livingThermoLogic.getTemperature()) * 256.0);  // Set real temperature here. Hardcoded for now.

  Serial.print("Response: desired temperature is ");
  Serial.println(livingThermoLogic.getDesiredTemperature());

  responseReplyBuffer[5] = (int) livingThermoLogic.getDesiredTemperature();  // Set real temperature here. Hardcoded for now.
  responseReplyBuffer[6] = (int) ((livingThermoLogic.getDesiredTemperature() - (int) livingThermoLogic.getDesiredTemperature()) * 256.0);  // Set real temperature here. Hardcoded for now.

  responseReplyBuffer[7] = (int) 1 + livingThermoLogic.getPower() - 1;  // Set real power level of the heater here. Hardcoded for now.

  responseReplyBuffer[8] = (int) livingThermoLogic.getHumidity();  // Set real humidity here. Hardcoded for now.
  responseReplyBuffer[9] = (int) ((livingThermoLogic.getHumidity() - (int) livingThermoLogic.getHumidity()) * 256.0);

  responseReplyBuffer[10] = 0x01;  // Set real amount of 110v outlets here
  responseReplyBuffer[11] = (int) ledPower / 256;  // Set real status of outlet here. There might be more than one
  responseReplyBuffer[12] = (int) (ledPower - (256 * responseReplyBuffer[11])) % 256;  // Set real status of outlet here. There might be more than one

  // sendUdpBuffer(UDP.remoteIP(), remotePort, replyBuffer);
  sendUdpBuffer(ip, port, responseReplyBuffer, 12);
}

void setLedPower(int value){
  analogWrite(ledOnOfPin, constrain(value, 0, 1023));
  ledPower = value;
  Serial.print("LedPower set to");
  Serial.println(ledPower);
}

void listenUdp(){
  // if there’s data available, read a packet
  int packetSize = UDP.parsePacket();

  char response[32];

  if(packetSize){
    // Serial.println("");
    // Serial.print("Received packet of size ");
    // Serial.print(packetSize);
    // Serial.print("From ");
    IPAddress remote = UDP.remoteIP();
    for (int i =0; i < 4; i++){
      // Serial.print(remote[i], DEC);
      if (i < 3) {
        // Serial.print(".");
      }
    }
    // Serial.print(", port ");
    // Serial.println(UDP.remotePort());

    // read the packet into packetBufffer
    UDP.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);

    if(packetSize != 4){
      sendUdpBufferResponse((char *)"Unknown instruction", 20);
      return ;
    }

    if(packetBuffer[0] == 0xFF && packetBuffer[1] == 0x00){
        // Report myself to the requester IP and to the specified port
		// Payload:	FF 00 XX YY => Ask heaters to report themselves to port XX * 256 + YY
		// Response:	FF 00 XX YY => Sent to the port specified before. This reports how many temperature sensors and heater has XX, how many 110v outlets has YY
        int remotePort = packetBuffer[2] * 256 + packetBuffer[3];
        char buffer[4] = {0xFF, 0x00, 0x01, 0x01};
        sendUdpBuffer(UDP.remoteIP(), remotePort, buffer, 4);    // Send real values here, with counts and everything
        Serial.print("DiscoveryRequest: Sent reply [0xFF, 0x00, 0x01, 0x01] to port ");
        Serial.println(remotePort);
        return ;
    }

    if(packetBuffer[0] == 0x30 && packetBuffer[1] == 0xFF){
        // Payload      30 FF XX YY =>  Requests for status to be sen back to port
        // Response	    30 FF AA (Name BB CC DD) EE (FF) =>
        int remotePort = packetBuffer[2] * 256 + packetBuffer[3];
        sendStatusResponse(UDP.remoteIP(), remotePort);
        // Serial.print("StatusRequest: Sent reply ");
        // Serial.print(" to port ");
        // Serial.println(remotePort);
        return ;
    }

    if(packetBuffer[0] == 0x10){
      float desiredTemperature = (float) (packetBuffer[2] / 1.0) + ((float) packetBuffer[3] / 0xFF);
      livingThermoLogic.setDesiredTemperature(desiredTemperature);
      Serial.print("Setting desired temperature to ");
      Serial.println(desiredTemperature);
      sendStatusResponse(UDP.remoteIP(), UDP.remotePort());
      // Serial.print("SetTemperature: Sent reply to port ");
      // Serial.println(UDP.remotePort());
      return ;
    }

    if(packetBuffer[0] == 0x20){
      setLedPower(packetBuffer[2] * 256 + packetBuffer[3]);
      sendStatusResponse(UDP.remoteIP(), UDP.remotePort());
      return ;
    }

    return ;
  } // End of if(packetSize);


} // End of void listenUdp();


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


void broadcastMovementDetectedAction(){
  char buffer[4] = { 0x11, 0x00, 0x00, 0x01 };
  sendUdpBuffer( ~WiFi.subnetMask() | WiFi.gatewayIP(), 8888, buffer, 4);
}

void broadcastCurrentStatusPeriodically(){

  if(millis() < timeOfLastStream){
    // Reset the timer. It happens once every 4 or 5 days
    timeOfLastStream = 0;
  }

  if(timeOfLastStream + 5000 > millis()){
    // Not read. Read has been done already
    return;
  }

  timeOfLastStream = millis();
  broadcastCurrentStatus();

}

void handleMovementDetection(){

  if(!digitalRead(movementSensorDataPin)){
    return ;
  }

  if(millis() < 65000){
    Serial.println("Sensor's still Initializing ...");
    return ;
  }

  if(
    timeOfLastMovement + 10000 < millis()
     ||
    timeOfLastMovement > millis() // To handle rollover after 72 minutes
  ){
    timeOfLastMovement = millis();
    Serial.print("Last:");
    Serial.print(timeOfLastMovement);
    Serial.print("- Now:");
    Serial.print(millis());
    Serial.println("MOVIMIENTO!");
    broadcastMovementDetectedAction();
  }
}


void blinkLedAccordingToPower(){
  int power = (int) 1 + livingThermoLogic.getPower() - 1;

  if(power == 0){
    // When power is off, the led will turn ON once every 5 seconds, for 1/10th of a second
    if(millis() % 5000 == 0 || (millis() - 100) % 5000 == 0){
      if(millis() % 5000 == 0){
        ledOn(LED_BUILTIN);
      } else {
        ledOff(LED_BUILTIN);
      }
    }
    return;
  }

  if(power == 5){
    if(
      millis() % 5000 == 0 ||
      (millis() - 100) % 5000 == 0 ||
      (millis() - 500) % 5000 == 0 ||
      (millis() - 600) % 5000 == 0){

      if(millis() % 5000 == 0 || (millis() - 500) % 5000 == 0){
          ledOn(LED_BUILTIN);
        } else {
          ledOff(LED_BUILTIN);
        }
        return;
    }
  }


  if(power == 10){
    if(
      millis() % 5000 == 0 ||
      (millis() - 100) % 5000 == 0 ||
      (millis() - 500) % 5000 == 0 ||
      (millis() - 600) % 5000 == 0 ||
      (millis() - 1000) % 5000 == 0 ||
      (millis() - 1100) % 5000 == 0
    ){

      if(millis() % 5000 == 0 || (millis() - 500) % 5000 == 0 || (millis() - 1000) % 5000 == 0){
          ledOn(LED_BUILTIN);
        } else {
          ledOff(LED_BUILTIN);
        }
        return;
    }
  }
}

void loop(){

  listenUdp();

  if(livingThermoLogic.readSensorValues()){
    Serial.print("LivingThermoLogic values: Temp: ");
    Serial.print(livingThermoLogic.getTemperature());
    Serial.print("*C , Humid: ");
    Serial.println(livingThermoLogic.getHumidity());
  }

  livingThermoLogic.calculatePower();
  livingThermoLogic.writePwmValues();

  blinkLedAccordingToPower();

  monitorButton();
  broadcastCurrentStatusPeriodically();
  handleMovementDetection();
}
