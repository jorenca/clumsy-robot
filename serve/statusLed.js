const gpio = require('rpi-gpio');
const gpiop = gpio.promise;

const util = require('util');
const exec = util.promisify(require('child_process').exec);


const RGB_LED_PINS = [25, 23, 24];
const sleep = (ms) => new Promise(resolve => setTimeout(resolve, ms));

let currentState = { r: 0, g: 0, b: 0 };
const setColor = async (color) => {
  if (!gpio) return;
  currentState = color;
  const {r, g, b} = color;
  await exec('raspi-gpio set ' + RGB_LED_PINS[0] + ' ' + (r ? 'dl' : 'dh'));
  await exec('raspi-gpio set ' + RGB_LED_PINS[1] + ' ' + (g ? 'dl' : 'dh'));
  await exec('raspi-gpio set ' + RGB_LED_PINS[2] + ' ' + (b ? 'dl' : 'dh'));

  //await gpiop.write(RGB_LED_PINS[0], !r);
  //await gpiop.write(RGB_LED_PINS[1], !g);
  //await gpiop.write(RGB_LED_PINS[2], !b);
};

module.exports = {
  init: async () => {
    if (!gpio) return;
    gpio.setMode(gpio.MODE_RPI);

    for (let i = 0; i < RGB_LED_PINS.length; i++) {
      const pin = RGB_LED_PINS[i];
      await exec('raspi-gpio set ' + pin + ' op'); //await gpiop.setup(pin, gpio.DIR_OUT);
    }


    await setColor({ r: 0, g: 0, b: 0 });

  },
  singleFlash: async (r, g, b) => {
    const beforeFlashState = {...currentState};
    await setColor({ r, g, b });
    await sleep(50);
    await setColor(beforeFlashState);
  },
  goGreen: async () => await setColor({ r: 0, g: 1, b: 0 }),
  goRed: async () => await setColor({ r: 1, g: 0, b: 0 }),
};
