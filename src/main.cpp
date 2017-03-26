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

#include "../lib/PirHcSr501/PirHcSr501.h"


// Thermostat functionality
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "../lib/ThermoLogic/ThermoLogic.h"

unsigned int localPort = 8888;
WiFiUDP UDP;
boolean udpConnected = false;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
// char ReplyBuffer[] = "Acknowledged";
char replyBuffer[32];


// Setup a DHT22 instance
ThermoLogic port1ThermoLogic(D6, DHT22, D3);
ThermoLogic port2ThermoLogic(D4, DHT22, D7);  // 2nd Thermo

PinHcSr501 Sensor1(D5);
PinHcSr501 Sensor2(D6);

// Setup the button reader
int ButtonState      = 0;
int PrevButtonState = 0;
unsigned long TimeWhenButtonWasPressed = millis();
bool IsALongPress = false;

uint32_t timeOfLastStream = 0;
uint32_t timeOfLastMovement = 0;

int movementSensorDataPin = D8;


int ledOnOfPin = D0; // NOT IN USE
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

  responseReplyBuffer[3] = (int) port1ThermoLogic.getTemperature();
  responseReplyBuffer[4] = (int) ((port1ThermoLogic.getTemperature() - (int) port1ThermoLogic.getTemperature()) * 256.0);  // Set real temperature here. Hardcoded for now.

  Serial.print("Response: desired temperature [1] is ");
  Serial.println(port1ThermoLogic.getDesiredTemperature());

  responseReplyBuffer[5] = (int) port1ThermoLogic.getDesiredTemperature();  // Set real temperature here. Hardcoded for now.
  responseReplyBuffer[6] = (int) ((port1ThermoLogic.getDesiredTemperature() - (int) port1ThermoLogic.getDesiredTemperature()) * 256.0);  // Set real temperature here. Hardcoded for now.

  responseReplyBuffer[7] = (int) 1 + port1ThermoLogic.getPower() - 1;  // Set real power level of the heater here. Hardcoded for now.

  responseReplyBuffer[8] = (int) port1ThermoLogic.getHumidity();  // Set real humidity here. Hardcoded for now.
  responseReplyBuffer[9] = (int) ((port1ThermoLogic.getHumidity() - (int) port1ThermoLogic.getHumidity()) * 256.0);


  Serial.print("Response: desired temperature [2] is ");
  Serial.println(port2ThermoLogic.getDesiredTemperature());

  responseReplyBuffer[10] = (int) port2ThermoLogic.getTemperature();
  responseReplyBuffer[11] = (int) ((port2ThermoLogic.getTemperature() - (int) port2ThermoLogic.getTemperature()) * 256.0);  // Set real temperature here. Hardcoded for now.

  responseReplyBuffer[12] = (int) port2ThermoLogic.getDesiredTemperature();  // Set real temperature here. Hardcoded for now.
  responseReplyBuffer[13] = (int) ((port2ThermoLogic.getDesiredTemperature() - (int) port2ThermoLogic.getDesiredTemperature()) * 256.0);  // Set real temperature here. Hardcoded for now.

  responseReplyBuffer[14] = (int) 1 + port2ThermoLogic.getPower() - 1;  // Set real power level of the heater here. Hardcoded for now.

  responseReplyBuffer[15] = (int) port2ThermoLogic.getHumidity();  // Set real humidity here. Hardcoded for now.
  responseReplyBuffer[16] = (int) ((port2ThermoLogic.getHumidity() - (int) port2ThermoLogic.getHumidity()) * 256.0);



  responseReplyBuffer[17] = 0x01;  // Set real amount of 110v outlets here
  responseReplyBuffer[18] = (int) ledPower / 256;  // Set real status of outlet here. There might be more than one
  responseReplyBuffer[19] = (int) (ledPower - (256 * responseReplyBuffer[11])) % 256;  // Set real status of outlet here. There might be more than one

  // sendUdpBuffer(UDP.remoteIP(), remotePort, replyBuffer);
  sendUdpBuffer(ip, port, responseReplyBuffer, 20);
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

  if(packetSize){
    IPAddress remote = UDP.remoteIP();
    for (int i =0; i < 4; i++){
      // Serial.print(remote[i], DEC);
      if (i < 3) {
        // Serial.print(".");
      }
    }

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


      if(packetBuffer[1] == 1){
        float desiredTemperature = (float) (packetBuffer[2] / 1.0) + ((float) packetBuffer[3] / 0xFF);
        port1ThermoLogic.setDesiredTemperature(desiredTemperature);
        Serial.print("Setting desired temperature [1] to ");
        Serial.println(desiredTemperature);
      }

      if(packetBuffer[1] == 2){
        float desiredTemperature = (float) (packetBuffer[2] / 1.0) + ((float) packetBuffer[3] / 0xFF);
        port2ThermoLogic.setDesiredTemperature(desiredTemperature);
        Serial.print("Setting desired temperature [2] to ");
        Serial.println(desiredTemperature);
      }
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

void ledOn(int pinNumber){
  if(LED_BUILTIN == pinNumber){
    digitalWrite(pinNumber, 0);
    return ;
  }
  digitalWrite(pinNumber, 1);
}


void ledOff(int pinNumber){
  if(LED_BUILTIN == pinNumber){
    digitalWrite(pinNumber, 1);
    return ;
  }
  digitalWrite(pinNumber, 0);
}

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


void broadcastMovementDetectedAction(int sensorNumber){
  char buffer[4] = { 0x11, 0x00, 0x00, char(sensorNumber) };
  sendUdpBuffer( ~WiFi.subnetMask() | WiFi.gatewayIP(), 8888, buffer, 4);
}

void broadcastButtonAction(){
  char buffer[4] = { 0x41, 0xFF, 0x00, 0x01 };
  sendUdpBuffer( ~WiFi.subnetMask() | WiFi.gatewayIP(), 8888, buffer, 4);
}

void broadcastButtonLongPress(){
  char buffer[4] = { 0x41, 0xFF, 0x00, 0x02 };
  sendUdpBuffer( ~WiFi.subnetMask() | WiFi.gatewayIP(), 8888, buffer, 4);
}

void broadcastCurrentStatus(){
  // sendStatusResponse({192, 168, 1, 255}, 8888);
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

  if(Sensor1.movementDetected()){
    broadcastMovementDetectedAction(1);
  }

  if(Sensor2.movementDetected()){
    broadcastMovementDetectedAction(2);
  }

  return ;

  if(!digitalRead(movementSensorDataPin)){
    return ;
  }

  if(millis() < 65000){
    Serial.println("Sensor's still Initializing ...");
    return ;
  }

}

void monitorButton(){
  ButtonState = digitalRead(D2);
  if(ButtonState == 0 && PrevButtonState == 0){
    // Serial.println("BUTTON: No changes");
    return ;
  }

  if(ButtonState == 1 && PrevButtonState == 0){
    Serial.println("BUTTON: PRESSED");
    TimeWhenButtonWasPressed = millis();
    PrevButtonState = 1;
    return ;
  }

  if(ButtonState == 0 && PrevButtonState == 1){
    if(IsALongPress){
      Serial.println("BUTTON: RELEASED AFTER LONG PRESS. IGNORE.");
    } else {
      broadcastButtonAction();
      Serial.println("BUTTON: RELEASED AFTER SHORT PRESS. ACTION!");

    }
    IsALongPress = false;
    TimeWhenButtonWasPressed = 0;
    PrevButtonState = ButtonState;
    return ;
  }

  if(!IsALongPress && ButtonState == 1 && PrevButtonState == 1 && TimeWhenButtonWasPressed + 1500 < millis()){
    Serial.println("BUTTON: STILL IN LONG PRESS");
    broadcastButtonLongPress();
    IsALongPress = true;
    TimeWhenButtonWasPressed = millis();
    return ;
  }
}



void blinkLedAccordingToPower(){
  int power = (int) 1 + port1ThermoLogic.getPower() - 1;

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

  if(port1ThermoLogic.readSensorValues()){
    Serial.print("port1ThermoLogic values: Temp: ");
    Serial.print(port1ThermoLogic.getTemperature());
    Serial.print("*C , Humid: ");
    Serial.println(port1ThermoLogic.getHumidity());
  }
  port1ThermoLogic.calculatePower();
  port1ThermoLogic.writePwmValues();

  if(port2ThermoLogic.readSensorValues()){
    Serial.print("port2ThermoLogic values: Temp: ");
    Serial.print(port2ThermoLogic.getTemperature());
    Serial.print("*C , Humid: ");
    Serial.println(port2ThermoLogic.getHumidity());
  }
  port2ThermoLogic.calculatePower();
  port2ThermoLogic.writePwmValues();

  blinkLedAccordingToPower();

  monitorButton();
  broadcastCurrentStatusPeriodically();
  handleMovementDetection();



}
