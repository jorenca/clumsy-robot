const SSE = require('express-sse');
const _ = require('lodash');

const PowerInput = require('./powerInput.js');
const ProximityInput = require('./proximityInput.js');
const TelemetryInput = require('./telemetryInput.js');
const MotorBoard = require('./motorBoard.js');
const StatusLed = require('./statusLed.js');
const IrDetectionModule = require('./irDetectionModule.js');

const Server = require('./server.js');

const telemetrySSE = new SSE();

let motorBoard = {};

ProximityInput.create({
  readInterval: 300,
  callback: proximity => {
    // TODO find a better place for this
    const dist = x => (0.237 * proximity - 0.39).toPrecision(3);
    if (dist < 7 && motorBoard.cstop) motorBoard.cstop();
    telemetrySSE.send({ proximity });
  }
});

PowerInput.create({
  readInterval: 300,
  callback: battery => telemetrySSE.send({ battery })
});

StatusLed.init()
.then(StatusLed.goRed)
.then(MotorBoard.connect)
.then(async motorBoardConn => {
  motorBoard = motorBoardConn;
  motorBoard.addListener(_.throttle(line => console.log(`[MB]> ${line}`), 1000));

  motorBoard.addListener(line => {
    if (line[0] != 'H') return;
    telemetrySSE.send(TelemetryInput.recalcWithNewDataLine(line));
  });
})
.then(IrDetectionModule.connect)
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
    telemetrySSE,
    onConnect: _.throttle(() => StatusLed.singleFlash(0, 0, 1), 300)
  });

  if (!motorBoard.error) await StatusLed.goGreen();
  await Server.listen();
})
.catch(console.error);
