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
    const workQueue: Array<Array<number>> = [];

    return {
        worker: () => {
            if (workQueue.length === 0) {
                return;
            }

            const work = workQueue.get(0);
            const steps = work[0];
            const speed = work[1]; // speed is measured in steps per second
            const dir = work[2];
            const isSpeedup = work[3] === 1;

            const pulseDuration = 1000000 / speed;
            pins.digitalWritePin(MOTOR_DIR_PINS[motor], dir);

            pins.analogWritePin(MOTOR_STEP_PIN_A[motor], 512);
            pins.analogSetPeriod(MOTOR_STEP_PIN_A[motor], pulseDuration);
            basic.pause(steps * 1000 / speed); // wait to complete

            workQueue.shift();
            if (workQueue.length === 0) {
                pins.analogWritePin(MOTOR_STEP_PIN_A[motor], 0);
            }
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

            workQueue.push([steps, speed, dir, 0]);
        },
        state: (): boolean => workQueue.length > 0
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
    serial.writeString('executing - ' + req + '\n');
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

serial.setRxBufferSize(128);
serial.onDataReceived(";", function () {
    const line = serial.readString();
    if (line.charAt(0) === 'M') { // M  -1.1 60 009.0 60
        omgSplit(line, ":").forEach(handleMoveRequest);
    }

    led.toggle(1, 0);
});

serial.writeString('GO!\n');
basic.forever(function () { // telemetry
    //const reportStr = report();
    //serial.writeString(reportStr + '\n');

    led.toggle(0, 0);
    basic.pause(100);
});
