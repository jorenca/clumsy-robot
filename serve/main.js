const express = require('express');
const path = require('path');
const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const SSE = require('express-sse');
const i2c = require('i2c-bus');
var telemetryEvents = new SSE();


async function connectToBoard() {
  const portsList = await SerialPort.list();
  console.log('Available COM ports:', portsList);
  console.log('Using telemetry provider on COM port', portsList[1]);
  const comPort = new SerialPort(portsList[1].comName, { baudRate: 115200 });
  const comParser = new Readline();
  comPort.pipe(comParser);

  return {
    addListener: callb => comParser.on('data', callb),
    send: data => comPort.write(data)
  };
};


const headingCalc = (function() {
  let data = [];
  return {
    addDataPoint: dp => data.push(Number(dp)),
    isReady: () => data.length >= 2,
    avg: () => {
      const resSin = data.reduce((acc, e) => Math.sin(e * Math.PI / 180.0) + acc, 0) / data.length;
      const resCos = data.reduce((acc, e) => Math.cos(e * Math.PI / 180.0) + acc, 0) / data.length;
      data = [];

      return Math.floor(Math.atan2(resSin, resCos) * 180 / Math.PI);
    }
  };
})();
function handleTelemetry(line) {
  const heading = line.match(/H(\d+)/)[1];
  headingCalc.addDataPoint(heading);
  if (headingCalc.isReady()) {
    telemetryEvents.send({
      heading: headingCalc.avg()
    });
  }

  const acceleration = {
    x: line.match(/AX(-?\d+)/)[1],
    y: line.match(/AY(-?\d+)/)[1],
    z: line.match(/AZ(-?\d+)/)[1]
  };
  const temperature = line.match(/T(\d+)/)[1];
  telemetryEvents.send({ acceleration, temperature });

  // sse.send(content, eventName);
  // sse.send(content, eventName, customID);
  // sse.updateInit(["array", "containing", "new", "content"]);
  // sse.serialize(["array", "to", "be", "sent", "as", "serialized", "events"]);
}
let boardConnection;
connectToBoard().then(conn => {
  conn.addListener(line => {

   // if (line[0] === 'D') return; // movement debugging aid

    console.log(`> ${line}`);

    if(line[0]!= 'H') return;
    handleTelemetry(line);
  });

  boardConnection = conn;
});

const PROXIMITY_SENSOR_I2C_ADDR = 0x40;
const PROXIMITY_SENSOR_I2C_DATA_REG = 0x5e;
const i2c1 = i2c.openSync(1);

function sendProximityData() {
  const proximity = i2c1.readByteSync(PROXIMITY_SENSOR_I2C_ADDR, PROXIMITY_SENSOR_I2C_DATA_REG);
  telemetryEvents.send({ proximity });
}
setInterval(sendProximityData, 500);

const webApp = express()
const webPort = 3000

webApp.use(express.static(path.join(__dirname, '../bot-control/build')));
webApp.get('/telemetry', telemetryEvents.init);

webApp.get('/move/:xr/:xrpm/:yr/:yrpm', (req, res) => {
  const { xr, xrpm, yr, yrpm } = req.params;
  const msg = `M ${xr.padStart(5, ' ')} ${xrpm.padStart(2, ' ')} ${yr.padStart(5, ' ')} ${yrpm.padStart(2, ' ')}`;
  console.log('SENDING MOVE ' + msg);

  boardConnection.send(msg);

  res.send(req.params)
});

webApp.listen(webPort, () => console.log(`Web app listening on port ${webPort}.`))
