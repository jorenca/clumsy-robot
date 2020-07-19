const gpio = require('rpi-gpio');
const gpiop = gpio.promise;

const sleep = (ms) => new Promise(resolve => setTimeout(resolve, ms));

const LOW = 0;
const HIGH = 1;

const LCDWIDTH = 84;
const LCDHEIGHT = 48;

const PCD8544_POWERDOWN = 0x04;
const PCD8544_ENTRYMODE = 0x02;
const PCD8544_EXTENDEDINSTRUCTION = 0x01;

const PCD8544_DISPLAYBLANK = 0x0;
const PCD8544_DISPLAYNORMAL = 0x4;
const PCD8544_DISPLAYALLON = 0x1;
const PCD8544_DISPLAYINVERTED = 0x5;

// H = 0
const PCD8544_FUNCTIONSET = 0x20;
const PCD8544_DISPLAYCONTROL = 0x08;
const PCD8544_SETYADDR = 0x40;
const PCD8544_SETXADDR = 0x80;

// H = 1
const PCD8544_SETTEMP = 0x04;
const PCD8544_SETBIAS = 0x10;
const PCD8544_SETVOP = 0x80;

 // calibrate clock constants
const CLKCONST_1 = 8000;

const MSBFIRST = 1;
const LSBFIRST = 0;
const Bitbanger = ({ csPin, dataPin, clockPin, bitOrder = MSBFIRST }) => {
  // bitbang serial shift out on select GPIO pin. Data rate is defined by CPU clk speed and CLKCONST_2.
  // Calibrate these value for your need on target platform.
  const shiftOut = async (byteArray) => {
    for (let byteIndex in byteArray) {
      const byteVal = byteArray[byteIndex];

      for (let i = 0; i < 8; i++) {
        const nextBit = (bitOrder == LSBFIRST) ? (1 << i) : (1 << (7 - i));
        await gpiop.write(dataPin, !!(byteVal & nextBit));

        await gpiop.write(clockPin, HIGH);
        await gpiop.write(clockPin, LOW); // this transition is slow enough to be ok for clock
      }
    }
  };

  const spiwrite = async (byteArray) => {
    await gpiop.write(csPin, LOW);  //bugfix
    await shiftOut(byteArray);
    await gpiop.write(csPin, HIGH); //bugfix
  };

  return {
    spiwrite
  };
};

module.exports = async ({ SCLK, DIN, DC, CS, RST, contrast }) => {
  gpio.setMode(gpio.MODE_RPI);

  await gpiop.setup(DIN, gpio.DIR_OUT);
  await gpiop.setup(SCLK, gpio.DIR_OUT);
  await gpiop.setup(DC, gpio.DIR_OUT);
  await gpiop.setup(RST, gpio.DIR_OUT);
  if (CS) await gpiop.setup(CS, gpio.DIR_OUT);

  const bitbanger = Bitbanger({ clockPin: SCLK, dataPin: DIN, csPin: CS });

  const sendCommand = async (byteCommand) => {
    await gpiop.write(DC, LOW);
    await bitbanger.spiwrite([ byteCommand ]);
  };

  const sendData = async (byteArray) => {
    await gpiop.write(DC, HIGH);
    await bitbanger.spiwrite(byteArray);
  };

  const setContrast = async (contrast, bias = 0x4) => {
    if (contrast > 0x7f) {
            contrast = 0x7f;
    }
    await sendCommand(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION);
    await sendCommand(PCD8544_SETBIAS | bias); // LCD bias select (4 is optimal?)
    await sendCommand(PCD8544_SETVOP | contrast);
    await sendCommand(PCD8544_FUNCTIONSET);
  };

  const reset = async () => {
    // toggle RST low to reset; CS low so it'll listen to us
    if (CS) await gpiop.write(CS, LOW);
    await gpiop.write(RST, LOW);
    await sleep(500);
    await gpiop.write(RST, HIGH);
  };

  const displayBuffer = async (data) => {
    for (let p = 0; p < 6; p++) { // 6 banks of rows
      await sendCommand(PCD8544_SETYADDR | p);
      await sendCommand(PCD8544_SETXADDR | 0); // start at X 0

      const pageDataStartIndex = LCDWIDTH * p;

      await sendData(data.slice(pageDataStartIndex, pageDataStartIndex + LCDWIDTH)); // this actually prints several lines

      // for (let col = 0; col < LCDWIDTH; col++) {
      //   await sendData(data[pageDataStartIndex + col]);
      // } TODO remove
    }
    await sendCommand(PCD8544_SETYADDR);  // no idea why this is necessary but it is to finish the last byte?
  };

  await reset();
  await setContrast(contrast);

  // Set display to Normal
  await sendCommand(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);

  return {
    reset,
    setContrast,
    displayBuffer
  };
};
