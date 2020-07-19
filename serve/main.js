const SSE = require('express-sse');
const _ = require('lodash');

const PowerInput = require('./powerInput.js');
const ProximityInput = require('./proximityInput.js');
const TelemetryInput = require('./telemetryInput.js');
const MotorBoard = require('./motorBoard.js');
const StatusLed = require('./statusLed.js');
const IrDetectionModule = require('./irDetectionModule.js');

const LCD = require('./pcdlib/io.js');
const QRCode = require('qrcode');
const os = require('os')

const Server = require('./server.js');

const telemetrySSE = new SSE();

let motorBoard = {};
let lcdDisplay = {};

let lastProximityDist = 0;
const shouldPreventMovesAhead = (dist) => dist < 10;
ProximityInput.create({
  readInterval: 200,
  callback: proximity => {
    // TODO find a better place for this
    const dist = (0.237 * proximity - 0.39).toPrecision(3);

    // Emergency stop to avoid hitting a wall
    if (shouldPreventMovesAhead(dist)
      && lastProximityDist > (dist + 0.1) // only if we are moving ahead
      && motorBoard.cstop
    ) {
      motorBoard.cstop();
    }
    lastProximityDist = dist;

    telemetrySSE.send({ proximity });
  }
});

PowerInput.create({
  readInterval: 200,
  callback: battery => {
    telemetrySSE.send({ battery });

    if (lcdDisplay.drawString) {
      lcdDisplay.drawString(54, 8, battery.busV.toPrecision(3) + 'V');
    }
  }
});

StatusLed.init()
.then(StatusLed.goRed)
.then(MotorBoard.connect)
.then(async motorBoardConn => {
  motorBoard = motorBoardConn;

  motorBoard.addListener(line => {
    if (line[0] != 'H') return;
    telemetrySSE.send(TelemetryInput.recalcWithNewDataLine(line));
  });
})
//.then(IrDetectionModule.connect) FIXME GEORGI turn back the IR detection module when clash with motor board is resolved
.then(conn => {
  if (!conn) return;

  const nextSchedule = () => {
    const move = conn.getMoveToTarget();
    if (!move || move.direction === 'none') {
      setTimeout(nextSchedule, 1000);
      return;
    };

    const rpm = 80;
    const timeTakenMs = (move.revs / rpm) * 60000;
    setTimeout(nextSchedule, timeTakenMs);
    motorBoard.doBasicMove({ ...move, rpm });
  };

  nextSchedule();
})
.then(async () => {
  Server.init({
    motorBoard,
    shouldPreventMovesAhead: () => shouldPreventMovesAhead(lastProximityDist),
    telemetrySSE,
    onConnect: _.throttle(() => StatusLed.singleFlash(0, 0, 1), 300)
  });

  if (!motorBoard.error) await StatusLed.goGreen();
  await Server.listen();
})
.then(() => {
  return LCD({ // physical pins
    DIN: 38,
    SCLK: 36,
    DC: 40,
    RST: 35,
    CS: 32
  }).then(async lcd => {
    lcdDisplay = lcd;

    const ip = os.networkInterfaces().wlan0[0].address;

    const qrdata = await QRCode.toString(`http://${ip}:3000/`);
    qrdata.split('\n').map((row, y) => {
      y -= 1;
      if (y < 0) return; // skip empty space

      _.forEach(row, (char, x) => {
        x -= 4; // skip empty space
        if (x < 0) return;

        lcd.setpixel(2*x, 2*y, char === '▀' || char === '█');
        lcd.setpixel(2*x + 1, 2*y, char === '▀' || char === '█');

        lcd.setpixel(2*x, 2*y+1, char === '▄' || char === '█');
        lcd.setpixel(2*x+1, 2*y+1, char === '▄' || char === '█');
      });
    });

    lcd.drawString(0, 32, ip + ':3000');

    lcd.drawString(58, 0, 'Bat');
    setInterval(() => lcdDisplay.flushData(), 5000);
  }).then(console.log);
})
.catch(console.error);
