const gpio = require('rpi-gpio');
const gpiop = gpio.promise;

const RGB_LED_PINS = [22, 17, 27];
const sleep = (ms) => new Promise(resolve => setTimeout(resolve, ms));

let currentState = { r: 0, g: 0, b: 0 };
const setColor = async (color) => {
  currentState = color;
  const {r, g, b} = color;
  await gpiop.write(RGB_LED_PINS[0], !r);
  await gpiop.write(RGB_LED_PINS[1], !g);
  await gpiop.write(RGB_LED_PINS[2], !b);
};

module.exports = {
  init: async () => {
    if (!gpio) return;
    gpio.setMode(gpio.MODE_BCM);

    for (let i = 0; i < RGB_LED_PINS.length; i++) {
      const pin = RGB_LED_PINS[i];
      await gpiop.setup(pin, gpio.DIR_OUT);
    }
    await setColor(0, 0, 0);
  },
  singleFlash: async (r, g, b) => {
    if (!gpio) return;
    const beforeFlashState = {...currentState};
    await setColor({ r, g, b });
    await sleep(100);
    await setColor(beforeFlashState);
  },
  goGreen: async () => {
    if (!gpio) return;
    await setColor(0, 1, 0);
  },
  goRed: async () => {
    if (!gpio) return;
    await setColor(1, 0, 0);
  }
};
