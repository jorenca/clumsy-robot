const express = require('express');
const path = require('path');
const superMarioThemeSong = require('./super-mario-theme.json');

const webPort = 3000;

const webApp = express();

module.exports = {
  init: ({ motorBoard, telemetrySSE, onConnect }) => {
    webApp.use(express.static(path.join(__dirname, '../bot-control/build')));
    webApp.get('/telemetry', (...args) => {
      onConnect();
      return telemetrySSE.init(...args);
    });

    webApp.get('/move/:xr/:xrpm/:yr/:yrpm', (req, res) => {
      onConnect();
      motorBoard.doMove(req.params);
      res.send(req.params);
    });

    webApp.get('/cstop', (req, res) => {
      onConnect();
      motorBoard.sendRaw('CSTOP;');
      res.send(req.params);
    });

    webApp.get('/motor_cmd/:cmd', (req, res) => {
      onConnect();
      motorBoard.sendRaw(req.params.cmd);
      res.send(req.params);
    });

    webApp.get('/sing', (req, res) => {
      let time = 0;
      superMarioThemeSong.forEach(note => {
        const startAfter = time;
        setTimeout(() => motorBoard.doDirectMove({ ...note, dir: 1 }), startAfter);
        time += note.delay * 1.2;
      });

      res.send(req.params);
    });

  },
  listen: () => webApp.listen(webPort, () => console.log(`Web app listening on port ${webPort}.`))
};
