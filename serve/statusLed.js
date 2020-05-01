const gpio = require('rpi-gpio');
const gpiop = gpio.promise;

const RGB_LED_PINS = [22, 17, 27];
const led = (async () => {


})();

module.exports = {
  init: async () => {
    if (!gpio) return;
    gpio.setMode(gpio.MODE_BCM);

    for (let i = 0; i < RGB_LED_PINS.length; i++) {
      const pin = RGB_LED_PINS[i];
      await gpiop.setup(pin, gpio.DIR_OUT);
      await gpiop.write(pin, 1);
    }
  },
  goGreen: async () => {
    if (!gpio) return;
    await gpiop.write(RGB_LED_PINS[0], 1);
    await gpiop.write(RGB_LED_PINS[1], 0);
    await gpiop.write(RGB_LED_PINS[2], 1);
  },
  goAmber: async () => {
    if (!gpio) return;
    await gpiop.write(RGB_LED_PINS[0], 0);
    await gpiop.write(RGB_LED_PINS[1], 0);
    await gpiop.write(RGB_LED_PINS[2], 1);
  }
};
