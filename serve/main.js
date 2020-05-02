const SSE = require('express-sse');
const _ = require('lodash');

const ProximityInput = require('./proximityInput.js');
const TelemetryInput = require('./telemetryInput.js');
const MotorBoard = require('./motorBoard.js');
const StatusLed = require('./statusLed.js');
const IrDetectionModule = require('./irDetectionModule.js');

const Server = require('./server.js');

const telemetrySSE = new SSE();

let motorBoard = {};

ProximityInput.create({
  readInterval: 500,
  callback: proximity => telemetrySSE.send({ proximity })
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

  if (!motorBoard.error) await StatusLed.goGreen();
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
  await Server.listen();
})
.catch(console.error);
