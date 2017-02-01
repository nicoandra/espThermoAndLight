/**
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
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
char ReplyBuffer[32];


// Setup a DHT22 instance
#define DHTLIVINGPIN D6
#define DHTTYPE DHT22
DHT_Unified dhtLiving(DHTLIVINGPIN, DHTTYPE);


ThermoLogic livingThermoLogic(D6, DHT22, D8);

float measuredTemperatures[2] = {0,0};
float requestedTemperatures[2] = {19, 19};  // Default temperature to 19. GOod practice.
int thermostatPower[2] = {0,0};
unsigned long timeOfLastHeaterPowerUpdate = millis();
float humidity[2] = {-1,-1};
float temporaryData[2] = {-1, -1};
uint32_t delayMS;


// Setup the time tracking variable
unsigned long time = millis();
unsigned timeOfLastTemperatureRead = time;

// Setup the button reader
int ButtonState      = 0;
int PrevButtonState = 0;
unsigned long TimeWhenButtonWasPressed = millis();
bool IsALongPress = false;

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

  /*Serial.print(ButtonState);
  Serial.print(PrevButtonState);*/

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

void sendUdpResponse(char ReplyBuffer[]){
  UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
  UDP.write(ReplyBuffer);
  UDP.endPacket();
}

void listenUdp(){
  // if there’s data available, read a packet
  int packetSize = UDP.parsePacket();
  // Serial.print("Packetsize: ");
  // Serial.println(packetSize);

  int affectedHeater = 0;

  char response[16];

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
      sendUdpResponse((char *)"Unknown instruction");
      return ;
    }

    if(packetBuffer[0] == 0xFF && packetBuffer[1] == 0x00){

    }

    switch(packetBuffer[0]){
      case 0xFF:
        if (packetBuffer[1] != 0xFF){
          sendUdpResponse((char *)"Commands with 0xFF must continue with 0xFF as well");
          return;
        }

        if (packetBuffer[2] != 0x00){
          sendUdpResponse((char *)"Commands with 0xFF 0xFF must continue with 0x00");
          return;
        }

        switch(packetBuffer[3]){
          case 0x00: sendUdpResponse((char *)"PingBack!"); Serial.println("PingBack!"); break;
          case 0xFF: sendUdpResponse((char *)"ResetDevice!"); Serial.println("ResetDevice!"); break;
          default: sendUdpResponse((char *)"Last byte should be either 0x00 or 0xFF"); break;
        }
        return ;
        break;

      case 0x11: // Set temperature of heater in position [byte 2
        affectedHeater = packetBuffer[1];

        if(affectedHeater < 0 || sizeof(requestedTemperatures) / sizeof(float) < affectedHeater){
          sendUdpResponse((char *)"Heater not found");
          Serial.println("Heater not found");
          return ;
        }

        if(packetBuffer[3] > 9){
          packetBuffer[3] = 9;
        }
        requestedTemperatures[affectedHeater] = (float) packetBuffer[2] + (float) ((float) packetBuffer[3] / 10);
        // temperatures[affectedHeater] = (float) 0 + (float) (packetBuffer[3] / 10);
        Serial.print("TOCANDO EL HEATER 1 ");
        Serial.println(affectedHeater);

        // sprintf(response, "OK %i %.02f", affectedHeater, (float) 24.5);  // This did not work :( throws exception
        // sprintf(response, "OK %i %i.%i", affectedHeater, 24, 5);
        // sendUdpResponse((char *)response);
        // Serial.println(response);
        // break;

      case 0x10:  // Get temperature of heater in position [byte 2
        affectedHeater = packetBuffer[1];
        sprintf(response, "OK %d %d.%d %d.%d", affectedHeater,
          (int)measuredTemperatures[affectedHeater],
          (int)(measuredTemperatures[affectedHeater]*10) % 10,
          (int)requestedTemperatures[affectedHeater],
          (int)(requestedTemperatures[affectedHeater]*10) % 10);
        Serial.println(response);
        sendUdpResponse((char *)response);
        break;

      default:  Serial.println("EEEEEH BEN ...(??)"); break;
    }


    Serial.println("Contents:");
    for(int i = 0; i < packetSize; i++){
      int value = packetBuffer[i];
      // Serial.println(value);
      Serial.print(value);
      Serial.print("-");
    }
    Serial.println("");


    // send a reply, to the IP address and port that sent us the packet we received
    /*UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.write(ReplyBuffer);
    */
    UDP.endPacket();
  }
}


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

  // Start temperature heaters
  dhtLiving.begin();


  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  ledOff(LED_BUILTIN);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);
  digitalWrite(D8, LOW);
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

  time = millis();
  listenUdp();

  if(livingThermoLogic.readSensorValues()){
    Serial.print("LivingThermoLogic value ");
    Serial.print(livingThermoLogic.getTemperature());
    Serial.println(":)");
  }

  livingThermoLogic.calculatePower();
  livingThermoLogic.writePwmValues();
  livingThermoLogic.setDesiredTemperature(10);

  monitorButton();


}
