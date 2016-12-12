const dgram = require('dgram');
const port = 8888;
const host = "192.168.1.128"

server = dgram.createSocket('udp4');

server.on('message', (msg, rinfo) => {
		stringMsg = msg.toString();
		console.log("<<< " ,msg, stringMsg, rinfo);
})

resetMcu  = [0xFF, 0xFF, 0x00, 0xFF];
pingBackWithHealthStatus = [0xFF, 0xFF, 0x00, 0x00];

setTemperatureOfHeater1 = [0x11, 0x01, 0x18, 0x00];
getTemperatureOfHeater1 = [0x10, 0x01, 0x00, 0x00];
turnOnRelay1 = [0x21, 0x01, 0x00, 0x01];
turnOffRelay1 = [0x21, 0x01, 0x00, 0x00];
getStatusOfRelay1 = [0x20, 0x01, 0x00, 0x00];

payload = pingBackWithHealthStatus;
server.send(new Buffer(payload), 0, payload.length, port, host, function(err,res){
	console.log(err, res);
})




/*
Possible messages;

BYTE
0					1					2					3					Meaning
11111111	11111111	00000000	11111111	Reset MCU
11111111	11111111	00000000	00000000	Ping back for health status. A response will be sent with : UPTIME, IP.
00010001	XXXXXXXX	yyyyyyyy  yyyyyyyy	Set temperature for heater in byte 1 to the value specified in byte 2 and 3. Desired temperature will be sent back as response.
00010000	XXXXXXXX	00000000	00000000	Get temperature of heater in byte 1. Response will be sent back immediately and later on as well.
00100001	XXXXXXXX	00000000	00000001	Turn ON  relay specified in byte 1. Response will be sent back immediately and later on as well.
00100001	XXXXXXXX	00000000	00000000	Turn OFF  relay specified in byte 1. Response will be sent back immediately and later on as well.
00100000	XXXXXXXX	00000000	00000000	Get status of relay specified in byte 1. Response will be sent back immediately and later on as well.
*/
