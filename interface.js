const dgram = require('dgram');
const port = 8888;
const host = "192.168.1.128"

server = dgram.createSocket('udp4');

server.on('message', (msg, rinfo) => {
		console.log("<<< " ,msg, rinfo);
})

payload = [0x01, 0xFF, 0xFF,0xFF];
server.send(new Buffer(payload), 0, 4, port, host, function(err,res){
	console.log(err, res);
})