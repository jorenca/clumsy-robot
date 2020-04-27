const express = require('express');
const path = require('path');
const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const SSE = require('express-sse');
const i2c = require('i2c-bus');
const gpio = require('rpi-gpio')
const gpiop = gpio.promise;
var telemetryEvents = new SSE();

const superMarioThemeSong = require('./super-mario-theme.json');
const ProximityInput = require('./proximityInput.js');

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

ProximityInput.create({
  readInterval: 500,
  callback: proximity => elemetryEvents.send({ proximity })
});

const webApp = express()
const webPort = 3000

webApp.use(express.static(path.join(__dirname, '../bot-control/build')));
webApp.get('/telemetry', telemetryEvents.init);

const ACCELERATION_TIME = 500; //ms
const ACCELERATION_DIVS = 5;
const doMove = ({ xr, xrpm, yr, yrpm }) => `M ${xr} ${xrpm} ${yr} ${yrpm} ;`;
webApp.get('/move/:xr/:xrpm/:yr/:yrpm', (req, res) => {
  boardConnection.send(doMove(req.params));
  res.send(req.params)
});

webApp.get('/cstop', (req, res) => {
  boardConnection.send('CSTOP;');
  res.send(req.params)
});

webApp.get('/motor_cmd/:cmd', (req, res) => {
  boardConnection.send(`${req.params.cmd}`);
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


const RGB_LED_PINS = [22, 17, 27];
const led = (async () => {
  if (!gpio) return;
  gpio.setMode(gpio.MODE_BCM);

  for (let i = 0; i < RGB_LED_PINS.length; i++) {
    const pin = RGB_LED_PINS[i];
    await gpiop.setup(pin, gpio.DIR_OUT);
    await gpiop.write(pin, 1);
  }

  await gpiop.write(RGB_LED_PINS[1], 0);
})();

webApp.listen(webPort, () => console.log(`Web app listening on port ${webPort}.`))
