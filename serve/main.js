const SSE = require('express-sse');
const _ = require('lodash');

const ProximityInput = require('./proximityInput.js');
const TelemetryInput = require('./telemetryInput.js');
const ComBoards = require('./comBoards.js');
const StatusLed = require('./statusLed.js');

const Server = require('./server.js');

var telemetryEvents = new SSE();

let boardConnection;
ComBoards.connectToMotorBoard().then(conn => {
  conn.addListener(_.throttle(line => console.log(`> ${line}`), 1000));

  conn.addListener(line => {
    if (line[0] != 'H') return;
    telemetryEvents.send(TelemetryInput.recalcWithNewDataLine(line));
  });
  boardConnection = conn;
});

ProximityInput.create({
  readInterval: 500,
  callback: proximity => telemetryEvents.send({ proximity })
});

setTimeout(() => {
  Server.init({
    sendToMotorBoardFn: boardConnection.send,
    telemetrySSE: telemetryEvents
  });

  StatusLed.init()
    .then(StatusLed.goGreen)
    .then(Server.listen);
}, 3000);
