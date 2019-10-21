#define F_CPU 8000000

#include "atmega-timers.h"


int STEP_PIN[] = {6, 8};
boolean toggle[] = {0, 0};
void toggleStep(int motor) {
  digitalWrite(STEP_PIN[motor], toggle[motor] ? HIGH : LOW);
  toggle[motor] = !toggle[motor];
}

///////////////////////////////////////////////////////////////////////////////
const int QUEUE_SIZE = 50;
int workQueueIndex[2] = {0, 0};
int workQueue[2][QUEUE_SIZE] = {{0}, {0}};
int workQueueSpeed[2][QUEUE_SIZE] = {{0}, {0}};

///////////////////////////////////////////////////////////////////////////////
int actualSpeed[2] = {0, 0};
int actualToggleOnIthCycle[2] = {0, 0};
const int SLOWEST_SPEED_STEP_CYCLES = 80;
const int DELTA_CYCLES_PER_SPEED = 2;

void setActualSpeed(int motor, int nthSpeed) {
  actualSpeed[motor] = nthSpeed;
  actualToggleOnIthCycle[motor] = nthSpeed > 0
      ? SLOWEST_SPEED_STEP_CYCLES - nthSpeed * DELTA_CYCLES_PER_SPEED
      : 0;
}

int requestedSpeed[2] = {0, 0};
void requestSpeed(int motor, int nthSpeed) {
  requestedSpeed[motor] = nthSpeed;
}

///////////////////////////////////////////////////////////////////////////////
void applyJerk(int motor) {
  if (requestedSpeed[motor] > actualSpeed[motor]) {
    setActualSpeed(motor, actualSpeed[motor] + 1);
  } else if (requestedSpeed[motor] < actualSpeed[motor]) {
    setActualSpeed(motor, actualSpeed[motor] - 1);
  }
}

///////////////////////////////////////////////////////////////////////////////
int jerkApplyCounter = 0;
const int JERK_APPLY_ON_COUNTER_VALUE = 200; // 4kHz / 200 = 20Hz

volatile int cyclesPerHalfStep[] = {20, 0};
volatile int cyclesLeftUntilToggle[] = {0, 0};
void timerSnapCallback() { // 8Mhz / 2000 counter = 4kHz
  // if (jerkApplyCounter++ >= JERK_APPLY_ON_COUNTER_VALUE) {
  //   applyJerk(0);
  //   applyJerk(1);
  //   jerkApplyCounter = 0;
  // }

  for(int motor = 0; motor <= 1; motor++) {
    //stepTickHappens(motor, true);
    if (cyclesPerHalfStep[motor] > 0) {
      if (cyclesLeftUntilToggle[motor] == 0) {
        toggleStep(motor);
        cyclesLeftUntilToggle[motor] = cyclesPerHalfStep[motor];
      }
      cyclesLeftUntilToggle[motor]--;
    }
  }
}

const int INTERRUPT_FREQ = F_CPU / 1000; // 8k
void setFreq(int motor, int hz) {
  cyclesPerHalfStep[motor] = (INTERRUPT_FREQ / hz) / 2;
}

///////////////////////////////////////////////////////////////////////////////
void setup() {
  cli();//stop interrupts

  Serial.begin(115200);
  //inputString.reserve(200);

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);

  timer1(TIMER1_PRESCALER_1, 999U, timerSnapCallback); // execute 10khz
  //timer2(TIMER2_PRESCALER_1, 255, cb2); // execute every 2000th clock
  sei();
}

///////////////////////////////////////////////////////////////////////////////
void loop() {
}

///////////////////////////////////////////////////////////////////////////////
String inputString = "";         // a String to hold incoming data
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    if (inChar == ';') {
      cli();
      int steps = (int) inputString[0];
      int cycles = inputString.toInt();
      // workQueue[0][workQueueIndex[0]] = steps;
      // workQueueSpeed[0][workQueueIndex[0]] = cycles;
      // requestSpeed(0, cycles);
      setFreq(0, cycles);
      Serial.printf("> %d %d\r\n", steps, cycles);
      inputString = "";
      sei();

    } else {
      // add it to the inputString:
      inputString += inChar;
    }
  }
}
