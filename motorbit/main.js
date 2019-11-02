function report() {
    const hdg = input.compassHeading();
    const ax = input.acceleration(Dimension.X);
    const ay = input.acceleration(Dimension.Y);
    const az = input.acceleration(Dimension.Z);
    const temp = input.temperature();
    return `H${hdg}AX${ax}AY${ay}AZ${az}T${temp}`;
}

const MOTOR_ENABLE_PIN = DigitalPin.P8;
const MOTOR_STEP_PIN_A = [AnalogPin.P0, AnalogPin.P2];
const MOTOR_DIR_PINS = [DigitalPin.P1, DigitalPin.P16];
const STEPS_PER_REV = 800;
function MotorCtrl(motor: number) {
    let workQueue: Array<Array<number>> = [];
    const hasWork = (): boolean => workQueue.length > 0;
    let doingSpeedChange: boolean = false;
    let currentSpeed: number = 0;

    const doMove = (timeMs: number, speed: number, dir: number) => {  // speed is measured in steps per second
        const pulseDuration = 1000000 / speed;

        pins.digitalWritePin(MOTOR_DIR_PINS[motor], dir);
        pins.analogWritePin(MOTOR_STEP_PIN_A[motor], 512);
        pins.analogSetPeriod(MOTOR_STEP_PIN_A[motor], pulseDuration);

        currentSpeed = speed;
        basic.pause(timeMs); // wait to complete
    };

    const stopMotor = () => {
      pins.analogWritePin(MOTOR_STEP_PIN_A[motor], 0);
      currentSpeed = 0;
    }

    const SPEED_JERK_STEPS = 100;
    const JERK_STEP_LENGTH_MS = 20;
    const MIN_SPEED = 300;
    const accelerateTo = (desiredSpeed: number, dir: number): number => { // speed is measured in steps per second
      doingSpeedChange = true;
      let msPassed = 0;
      const initialStep = Math.max(currentSpeed + SPEED_JERK_STEPS, MIN_SPEED);
      for (let step = initialStep; step < desiredSpeed; step += SPEED_JERK_STEPS) {
        doMove(JERK_STEP_LENGTH_MS, step, dir);
        msPassed += JERK_STEP_LENGTH_MS;
      }
      doingSpeedChange = false;
      return msPassed;
    };

    const deccelerateTo = (desiredSpeed: number, dir: number): number => { // speed is measured in steps per second
      doingSpeedChange = true;
      let msPassed = 0;
      for (
        let step = currentSpeed - SPEED_JERK_STEPS;
        step > desiredSpeed && step >= MIN_SPEED;
        step -= SPEED_JERK_STEPS) {
        doMove(JERK_STEP_LENGTH_MS, step, dir);
        msPassed += JERK_STEP_LENGTH_MS;
      }
      if (desiredSpeed === 0) {
        stopMotor();
      }
      doingSpeedChange = false;
      return msPassed;
    };

    const runNextTask = () => {
      const work = workQueue.get(0);
      const timeMs = work[0];
      const speed = work[1]; // speed is measured in steps per second
      const dir = work[2];

      const msChangingSpeed =  currentSpeed < speed ? accelerateTo(speed, dir) : deccelerateTo(speed, dir);

      doMove(Math.max(timeMs - msChangingSpeed, 0), speed, dir);
      workQueue.shift();

      if (!hasWork()) { deccelerateTo(0, dir); }
    };

    return {
        worker: () => {
            if (!hasWork()) { return; }
            runNextTask();
        },
        cstop: () => {
            workQueue = [];
            stopMotor();
        },
        addMove: (revs: number, rpm: number) => {
            let dir: number;
            if (motor === 0) {
                dir = revs < 0 ? 1 : 0;
            } else {
                dir = revs < 0 ? 0 : 1;
            }
            const steps = (revs > 0 ? revs : -revs) * STEPS_PER_REV;
            const speed = rpm * STEPS_PER_REV / 60;
            const timeMs = steps * 1000 / speed; // TODO GEORGI cleanup calc

            workQueue.push([timeMs, speed, dir]);
        },
        addDirectMove: (timeMs: number, speed: number, dir: number) =>  // speed is measured in steps per second
            workQueue.push([timeMs, speed, dir]),
        state: (): boolean => hasWork() || doingSpeedChange
    }
}
const motorCtrl = {
    left: MotorCtrl(0),
    right: MotorCtrl(1),
    enable: () => pins.digitalWritePin(MOTOR_ENABLE_PIN, 0),
    disable: () => pins.digitalWritePin(MOTOR_ENABLE_PIN, 1)
};
basic.forever(function () {
    while (motorCtrl.left.state()) {
        motorCtrl.left.worker();
    }
    basic.pause(10);
});
basic.forever(function () {
    while (motorCtrl.right.state()) {
        motorCtrl.right.worker();
    }
    basic.pause(10);
});

basic.forever(function () {
    basic.pause(3000);
    if (!motorCtrl.left.state() && !motorCtrl.right.state()) {
        motorCtrl.disable();
    }
});

input.onButtonPressed(Button.A, function () {
    motorCtrl.enable();
    motorCtrl.left.addMove(-1, 60);
    motorCtrl.right.addMove(1, 60);
});

function omgSplit(inp: string, delimiter: string): string[] {
    const res = [];
    let nextStartsAt = 0;
    for (let i = 0; i <= inp.length; i++) {
        if (i != inp.length && inp.charAt(i) != delimiter) continue;
        res.push(inp.substr(nextStartsAt, i - nextStartsAt));
        nextStartsAt = i + 1;
    }
    return res;
}
function handleMoveRequest(req: string) {
    serial.writeString('MOVE - ' + req + '\n');
    // M  -1.1 60 009.0 60
    const params = omgSplit(req, " ");
    const leftRevs = parseFloat(params[1]);
    const leftRPM = parseInt(params[2]);
    const rightRevs = parseFloat(params[3]);
    const rightRPM = parseInt(params[4]);

    motorCtrl.enable();
    if (leftRevs != 0) {
        motorCtrl.left.addMove(leftRevs, leftRPM);
    }
    if (rightRevs != 0) {
        motorCtrl.right.addMove(rightRevs, rightRPM);
    }
}

function handleDirectMoveRequest(req: string) {
    serial.writeString('DIRECT MOVE - ' + req + '\n');
    // D[LR] timeMs freq dir
    const params = omgSplit(req, " ");
    const motor = params[0].charAt(1);
    const timeMs = parseInt(params[1]);
    const freq = parseInt(params[2]);
    const dir = parseInt(params[3]);

    motorCtrl.enable();
    motorCtrl[motor == 'L' ? 'left' : 'right']
        .addDirectMove(timeMs, freq, dir);
}

function handleCoordinatedTurn(req: string) {
    const params = omgSplit(req, " ");
    const targetHeading = parseInt(params[1]);

    motorCtrl.enable();
    motorCtrl.left.addDirectMove(1000 * 10, 1000, 0);
    motorCtrl.right.addDirectMove(1000 * 10, 1000, 0);

    while (motorCtrl.left.state()) {
        const hdg = input.compassHeading();

        const deltaHdg = Math.abs(hdg - targetHeading);
        if (deltaHdg < 5) {
            motorCtrl.left.cstop();
            motorCtrl.right.cstop();
        }

        basic.pause(10);
    }
}

serial.setRxBufferSize(128);
serial.onDataReceived(";", function () {
    const line = serial.readString();
    if (line.charAt(0) === 'M') { // M  -1.1 60 009.0 60
        omgSplit(line, ":").forEach(handleMoveRequest);
    }

    if (line.charAt(0) == 'D') {
        omgSplit(line, ":").forEach(handleDirectMoveRequest);
    }

    if (line.charAt(0) == 'C' && line.charAt(1) == ' ') { // C 180
        handleCoordinatedTurn(line);
    }

    if (line === 'CSTOP;') {
        motorCtrl.enable();
        motorCtrl.left.cstop();
        motorCtrl.right.cstop();
    }

    led.toggle(1, 0);
});

serial.writeString('GO!\n');
basic.forever(function () { // telemetry
    const reportStr = report();
    serial.writeString(reportStr + '\n');

    led.toggle(0, 0);
    basic.pause(100);
});
