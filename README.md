# espThermoAndLight

# Setup
Turn on your ESP8266. Connect to the Wifi network it creates and setup the wifi network.
It will reboot and Connect to the specified network.
If it fails, it will set itself in Access-Point mode again.

# Connections
D1: DHT-22 #1 OK
D2: DTH-22 #2 OK

D3: Relay heater 1 OK
D4: Relay heater 2 OK

D5: PIR Sensor 1 OK
D6: PIR Sensor 2 OK

# Hardware
Movement detector: Code works with HC-SR501, read  http://henrysbench.capnfatz.com/henrys-bench/arduino-sensors-and-input/arduino-hc-sr501-motion-sensor-tutorial/


# To Do
* Have a method to reply back with the actual values
* Have a method to store the IP of the server, so the ESP pings back to it
* Write the NodeJS implementation:

const dgram = require('dgram');

server = dgram.createSocket('udp4');
var payload = [0xF0, 0x00, 0x10, 0x88];

server.send(new Buffer(payload), 0, 4, 8888, "192.168.1.113", function(err,res){
	console.log(err, res);
});
