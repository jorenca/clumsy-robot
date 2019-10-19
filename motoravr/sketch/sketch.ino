

int STEP_PIN[] = {6, 8};
boolean toggle[] = {0, 0};
void toggleStep(int motor) {
  digitalWrite(STEP_PIN[motor], toggle[motor] ? HIGH : LOW);
  toggle[motor] = !toggle[motor];
}

const int QUEUE_SIZE = 50;
int workQueueIndex[2] = {0, 0};
int workQueue[2][QUEUE_SIZE] = {{0}, {0}};
int workQueueSpeed[2][QUEUE_SIZE] = {{0}, {0}};

int actualSpeed[2] = {0, 0};
int actualToggleOnIthCycle[2] = {0, 0};
const int SPEEDS = 10;
const int SLOWEST_SPEED_STEP_CYCLES = 10;
const int DELTA_CYCLES_PER_SPEED = SLOWEST_SPEED_STEP_CYCLES / SPEEDS; 

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

void applyJerk(int motor) {
  if (requestedSpeed[motor] > actualSpeed[motor]) {
    setActualSpeed(motor, actualSpeed[motor] + 1);
  } else if (requestedSpeed[motor] < actualSpeed[motor]) {
    setActualSpeed(motor, actualSpeed[motor] - 1);
  }
}

int elapsedCycles[2] = {0, 0};
boolean toggleStepIfTimeHasCome(int motor) {
  int& elapsed = elapsedCycles[motor];
  elapsed++;
  
  int& toggleOnCycle = actualToggleOnIthCycle[motor];
  if (toggleOnCycle == 0) { // turned off, 0 speed
    return false;
  }
  
  if (elapsed >= toggleOnCycle) {
    toggleStep(motor);
    elapsed = 0;
    
    return true;
  }

  return false;
}

int jerkApplyCounter = 0;
int JERK_APPLY_ON_COUNTER_VALUE = 40;
void stepTickHappens(int motor, boolean freeMode) {
  if (!freeMode) {
    int& stepsToDo = (workQueue[motor][workQueueIndex[motor]]);
    if (stepsToDo == 0) {
      return;
    }

    boolean stepHappened = toggleStepIfTimeHasCome(motor);
    if (!stepHappened) {
      return;
    }
    
    stepsToDo--;

    if (stepsToDo == 0) {
      int& nextStepsToDo = (workQueue[motor][workQueueIndex[motor] + 1]);
      if (nextStepsToDo > 0) {
        workQueueIndex[motor]++;
        // FIXME USE WORK QUEUE SPEEDS
        //requestSpeed(motor, );
      }
    }
  } else { // FREE MODE
     toggleStepIfTimeHasCome(motor);
  }
}

ISR (TIMER1_OVF_vect) {
   //TCNT1 = 0x0F00;
   TCNT1H = 0x50;
    
  jerkApplyCounter++;
  if (jerkApplyCounter >= JERK_APPLY_ON_COUNTER_VALUE) {
    applyJerk(0);
    applyJerk(1);
    jerkApplyCounter = 0;
  }
  
  for(int motor = 0; motor <= 1; motor++) {
      stepTickHappens(motor, true);
  }
}



String inputString = "";         // a String to hold incoming data

void setup() {
  cli();//stop interrupts
  
  Serial.begin(115200);
  inputString.reserve(200);
  
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);

  TIMSK |= (1 << TOIE1);
    sei(); 
    //enable interrupts
    TCCR1B |= (1 << CS10);// | (1 << CS00);
}

void loop() {
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    
    if (inChar == ';') {
      cli();
      int steps = (int) inputString[0];
      int cycles = inputString.toInt();
      workQueue[0][workQueueIndex[0]] = steps;
      workQueueSpeed[0][workQueueIndex[0]] = cycles;
      requestSpeed(0, cycles);
      Serial.printf("> %d %d\r\n", steps, cycles);
      inputString = "";
      sei();
      
    } else {
      // add it to the inputString:
      inputString += inChar;
    }
  }
}
