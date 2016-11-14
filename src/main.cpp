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

unsigned int localPort = 8888;
WiFiUDP UDP;
boolean udpConnected = false;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
char ReplyBuffer[] = "Acknowledged";

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

void listenUdp(){

  // if there’s data available, read a packet
  int packetSize = UDP.parsePacket();
  Serial.print("Packetsize: ");
  Serial.println(packetSize);

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
    Serial.println("Contents:");
    for(int i = 0; i < packetSize; i++){
      int value = packetBuffer[i];
      // Serial.println(value);
      Serial.print(value);
      Serial.print("-");
    }
    Serial.println("");


    // send a reply, to the IP address and port that sent us the packet we received
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.write(ReplyBuffer);
    UDP.endPacket();

    // turn LED on or off depending on value recieved
    // digitalWrite(5,value);
  }
  delay(10);

}





void setup()
{
  // Open Serial for Output
  Serial.begin(115200);
  Serial.println("Booting...");

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);

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


  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(D7, HIGH);
  digitalWrite(D8, HIGH);
  // wait for a second
  delay(1000);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);
  delay(50);
  digitalWrite(LED_BUILTIN, HIGH);
   // wait for a second
  delay(950);

}
