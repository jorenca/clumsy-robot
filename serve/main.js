const express = require('express');
const path = require('path');
const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const SSE = require('express-sse');
const i2c = require('i2c-bus');
var telemetryEvents = new SSE();

const superMarioThemeSong = require('./super-mario-theme.json');

async function connectToBoard() {
  const portsList = await SerialPort.list();
  console.log('Available COM ports:', portsList);
  const comName = (portsList[1] || portsList[0]).comName;
  console.log('Using telemetry provider on COM port', comName);
  const comPort = new SerialPort(comName, { baudRate: 115200 });
  const comParser = new Readline();
  comPort.pipe(comParser);

  return {
    addListener: callb => comParser.on('data', callb),
    send: data => {
      console.log(`< ${data} (message length ${data.length})`);
      comPort.write(data);
    }
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
    console.log(`> ${line}`);

    if (line[0] == 'H') {
      handleTelemetry(line);
    }
  });

  boardConnection = conn;
});

const PROXIMITY_SENSOR_I2C_ADDR = 0x40;
const PROXIMITY_SENSOR_I2C_DATA_REG = 0x5e;
const i2c1 = i2c && i2c.openSync(1);

function sendProximityData() {
  const proximity = i2c1.readByteSync(PROXIMITY_SENSOR_I2C_ADDR, PROXIMITY_SENSOR_I2C_DATA_REG);
  telemetryEvents.send({ proximity });
}
if (i2c1) {
  setInterval(sendProximityData, 500);
}

const webApp = express()
const webPort = 3000

webApp.use(express.static(path.join(__dirname, '../bot-control/build')));
webApp.get('/telemetry', telemetryEvents.init);

const ACCELERATION_TIME = 500; //ms
const ACCELERATION_DIVS = 5;
const accelerationLerp = (targetRpm, div, totalDivs) => {
  const rpm = targetRpm * (div / totalDivs);
  const revs = rpm * (ACCELERATION_TIME / totalDivs) / 60000;
  return { rpm: rpm.toPrecision(3), revs: revs.toPrecision(2) };
};
const doMove = ({ xr, xrpm, yr, yrpm, shouldAddMore }) => {
  let addition = '';
  if (shouldAddMore) {
    for (let i = 1; i < ACCELERATION_DIVS; i++) {
      const lerp = accelerationLerp(xrpm, i, ACCELERATION_DIVS);
      addition += doMove({
        xr: '' + lerp.revs,
        xrpm: '' + lerp.rpm,
        yr: '' + lerp.revs,
        yrpm: '' + lerp.rpm,
        shouldAddMore: false
      }) + ':';
    }
  }

  const msg = `M ${xr} ${xrpm} ${yr} ${yrpm}`;
  return addition + msg;
};
webApp.get('/move/:xr/:xrpm/:yr/:yrpm', (req, res) => {
  const move = doMove({ ...req.params, shouldAddMore: false }) + ' ;';
  boardConnection.send(move);
  res.send(req.params)
});

const doDirectMove = ({ timeMs, frequency, dir }) => `DR ${timeMs} ${frequency} ${dir}`;
webApp.get('/sing', (req, res) => {
  let time = 0;
  superMarioThemeSong.forEach(note => {
    const move = doDirectMove({ ...note, dir: 1 });
    const startAfter = time;
    setTimeout(() => boardConnection.send(move + ' ;'), startAfter);
    time += note.delay * 1.2;
  });

  res.send(req.params)
});

webApp.listen(webPort, () => console.log(`Web app listening on port ${webPort}.`))
