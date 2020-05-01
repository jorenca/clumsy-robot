const SSE = require('express-sse');
const _ = require('lodash');

const ProximityInput = require('./proximityInput.js');
const TelemetryInput = require('./telemetryInput.js');
const ComBoards = require('./comBoards.js');
const StatusLed = require('./statusLed.js');

const Server = require('./server.js');

const telemetrySSE = new SSE();

let sendToMotorBoardFn = _.noop;

ProximityInput.create({
  readInterval: 500,
  callback: proximity => telemetrySSE.send({ proximity })
});

StatusLed.init()
.then(StatusLed.goRed)
.then(ComBoards.connectToMotorBoard)
.then(async conn => {
  conn.addListener(_.throttle(line => console.log(`[MB]> ${line}`), 1000));

  conn.addListener(line => {
    if (line[0] != 'H') return;
    telemetrySSE.send(TelemetryInput.recalcWithNewDataLine(line));
  });

  sendToMotorBoardFn = conn.send;

  if (!conn.error) await StatusLed.goGreen();
})
.then(ComBoards.connectToDetectorBoard)
.then(conn => {
    conn.addListener(_.throttle(line => console.log(`[IR]> ${line}`), 1000));

    conn.addListener(line => {
      const irData = JSON.parse(line);

      console.log(`[IR]> ${JSON.stringify(irData, null, 2)}`);
    });
})
.then(async () => {
  Server.init({ sendToMotorBoardFn, telemetrySSE });
  await Server.listen();
})
.catch(console.error);
