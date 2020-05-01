const express = require('express');
const path = require('path');
const superMarioThemeSong = require('./super-mario-theme.json');

const webPort = 3000;

const doMove = ({ xr, xrpm, yr, yrpm }) => `M ${xr} ${xrpm} ${yr} ${yrpm} ;`;
const doDirectMove = ({ timeMs, frequency, dir }) => `DR ${timeMs} ${frequency} ${dir}`;

const webApp = express();

module.exports = {
  init: ({ sendToMotorBoardFn, telemetrySSE }) => {
    webApp.use(express.static(path.join(__dirname, '../bot-control/build')));
    webApp.get('/telemetry', telemetrySSE.init);

    webApp.get('/move/:xr/:xrpm/:yr/:yrpm', (req, res) => {
      sendToMotorBoardFn(doMove(req.params));
      res.send(req.params);
    });

    webApp.get('/cstop', (req, res) => {
      sendToMotorBoardFn('CSTOP;');
      res.send(req.params);
    });

    webApp.get('/motor_cmd/:cmd', (req, res) => {
      sendToMotorBoardFn(req.params.cmd);
      res.send(req.params);
    });

    webApp.get('/sing', (req, res) => {
      let time = 0;
      superMarioThemeSong.forEach(note => {
        const move = doDirectMove({ ...note, dir: 1 });
        const startAfter = time;
        setTimeout(() => sendToMotorBoardFn(move + ' ;'), startAfter);
        time += note.delay * 1.2;
      });

      res.send(req.params);
    });

  },
  listen: () => webApp.listen(webPort, () => console.log(`Web app listening on port ${webPort}.`))
};
