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
#include "../lib/ThermoLogic/ThermoLogic.h"

unsigned int localPort = 8888;
WiFiUDP UDP;
boolean udpConnected = false;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
// char ReplyBuffer[] = "Acknowledged";
char replyBuffer[32];


// Setup a DHT22 instance
ThermoLogic livingThermoLogic(D6, DHT22, D0);

// Setup the button reader
int ButtonState      = 0;
int PrevButtonState = 0;
unsigned long TimeWhenButtonWasPressed = millis();
bool IsALongPress = false;

float desiredTemperature;


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
      Serial.println("BUTTON: RELEASED AFTER SHORT PRESS. ACTION!");
    }
    IsALongPress = false;
    TimeWhenButtonWasPressed = 0;
    PrevButtonState = ButtonState;
    return ;
  }

  if(!IsALongPress && ButtonState == 1 && PrevButtonState == 1 && TimeWhenButtonWasPressed + 1500 < millis()){
    Serial.println("BUTTON: STILL IN LONG PRESS");
    IsALongPress = true;
    TimeWhenButtonWasPressed = millis();
    return ;
  }
}

void sendUdpBuffer(IPAddress host, int port, char buffer[], int length){
    UDP.beginPacket(host, port);
    UDP.write(buffer, length);
    UDP.endPacket();
}

void sendUdpBufferResponse(char replyBuffer[], int length){
  sendUdpBuffer(UDP.remoteIP(), UDP.remotePort(), replyBuffer, length);
}


void sendStatusResponse(IPAddress ip, int port){
  char replyBuffer[256];

  replyBuffer[0] = 0x30;
  replyBuffer[1] = 0xFF;
  replyBuffer[2] = 0x01;  // Set real count of heaters here. Hardcoded to 1 now.

  replyBuffer[3] = (int) livingThermoLogic.getTemperature();  // Set real temperature here. Hardcoded for now.
  replyBuffer[4] = (int) ((livingThermoLogic.getTemperature() - (int) livingThermoLogic.getTemperature()) * 256.0);  // Set real temperature here. Hardcoded for now.

  replyBuffer[5] = (int) livingThermoLogic.getDesiredTemperature();  // Set real temperature here. Hardcoded for now.
  replyBuffer[6] = (int) ((livingThermoLogic.getDesiredTemperature() - (int) livingThermoLogic.getDesiredTemperature()) * 256.0);  // Set real temperature here. Hardcoded for now.

  replyBuffer[7] = (int) 1 + livingThermoLogic.getPower() - 1;  // Set real power level of the heater here. Hardcoded for now.

  replyBuffer[8] = (int) livingThermoLogic.getHumidity();  // Set real humidity here. Hardcoded for now.
  replyBuffer[9] = (int) ((livingThermoLogic.getHumidity() - (int) livingThermoLogic.getHumidity()) * 256.0);

  replyBuffer[10] = 0x01;  // Set real amount of 110v outlets here
  replyBuffer[11] = 0xFF;  // Set real status of outlet here. There might be more than one

  Serial.print(replyBuffer);

  // sendUdpBuffer(UDP.remoteIP(), remotePort, replyBuffer);
  sendUdpBuffer(ip, port, replyBuffer, 12);
}

void listenUdp(){
  // if there’s data available, read a packet
  int packetSize = UDP.parsePacket();
  // Serial.print("Packetsize: ");
  // Serial.println(packetSize);

  int affectedHeater = 0;

  char response[32];

  if(packetSize){
    Serial.println("");
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = UDP.remoteIP();
    for (int i =0; i < 4; i++){
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }

    Serial.print(", port ");
    Serial.println(UDP.remotePort());

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

        /*char replyBuffer[256];

        replyBuffer[0] = 0x30;
        replyBuffer[1] = 0xFF;
        replyBuffer[2] = 0x01;  // Set real count of heaters here. Hardcoded to 1 now.

        replyBuffer[3] = (int) livingThermoLogic.getTemperature();  // Set real temperature here. Hardcoded for now.
        replyBuffer[4] = (int) ((livingThermoLogic.getTemperature() - (int) livingThermoLogic.getTemperature()) * 256.0);  // Set real temperature here. Hardcoded for now.

        replyBuffer[5] = livingThermoLogic.getPower();  // Set real power level of the heater here. Hardcoded for now.

        replyBuffer[6] = 0x01;  // Set real amount of 110v outlets here
        replyBuffer[7] = 0xFF;  // Set real status of outlet here. There might be more than one

        sendUdpBuffer(UDP.remoteIP(), remotePort, replyBuffer);
        Serial.print("StatusRequest: Sent reply ");
        Serial.print(replyBuffer);
        Serial.print(" to port ");
        Serial.println(remotePort);
        */
        sendStatusResponse(UDP.remoteIP(), remotePort);
        Serial.print("StatusRequest: Sent reply ");
        Serial.print(" to port ");
        Serial.println(remotePort);

        return ;
    }

    if(packetBuffer[0] == 0x10){
      desiredTemperature = (float) (packetBuffer[2] / 1.0) + ((float) packetBuffer[3] / 0xFF);
      livingThermoLogic.setDesiredTemperature(desiredTemperature);
      Serial.print("Setting desired temperature to ");
      Serial.println(desiredTemperature);
      sendStatusResponse(UDP.remoteIP(), UDP.remotePort());
      Serial.print("SetTemperature: Sent reply ");
      Serial.print(" to port ");
      Serial.println(UDP.remotePort());
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


void loop()
{

  listenUdp();

  if(livingThermoLogic.readSensorValues()){
    Serial.print("LivingThermoLogic value ");
    Serial.print(livingThermoLogic.getTemperature());
    Serial.println(":)");
  }

  livingThermoLogic.calculatePower();
  livingThermoLogic.writePwmValues();
  monitorButton();
}
