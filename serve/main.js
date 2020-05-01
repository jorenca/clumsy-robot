const SSE = require('express-sse');
const _ = require('lodash');

const ProximityInput = require('./proximityInput.js');
const TelemetryInput = require('./telemetryInput.js');
const ComBoards = require('./comBoards.js');
const StatusLed = require('./statusLed.js');

const Server = require('./server.js');

var telemetryEvents = new SSE();

ProximityInput.create({
  readInterval: 500,
  callback: proximity => telemetryEvents.send({ proximity })
});

StatusLed.init()
.then(StatusLed.goAmber)
.then(ComBoards.connectToMotorBoard)
.then(async conn => {
  conn.addListener(_.throttle(line => console.log(`> ${line}`), 1000));

  conn.addListener(line => {
    if (line[0] != 'H') return;
    telemetryEvents.send(TelemetryInput.recalcWithNewDataLine(line));
  });

  Server.init({
    sendToMotorBoardFn: conn.send,
    telemetrySSE: telemetryEvents
  });

  await StatusLed.goGreen();
  await Server.listen();
});
