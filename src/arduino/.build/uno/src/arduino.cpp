#include <Arduino.h>

void positionMotors();
void setup();
void loop();
void baseStep();
void baseDir(int dir);
void mainStep();
void mainDir(int dir);
void secStep();
void secDir(int dir);
void toolStep();
void toolDir(int dir);
void resetStepperInfo(struct stepperInfo& si);
void resetStepper(volatile struct stepperInfo& si);
float getDurationOfAcceleration(volatile stepperInfo& s, unsigned int numSteps);
void prepareMovement(int whichMotor, long steps);
void setNextInterruptInterval();
void runAndWait();
void adjustSpeedScales();
#line 1 "src/arduino.ino"
#define resolutionL1        2
#define resolutionL2        3
#define resolutionR1        8
#define resolutionR2        9

#define baseStepPin         4
#define baseDirPin          5

#define mainStepPin         6
#define mainDirPin          7

#define secStepPin          10
#define secDirPin           11

#define toolStepPin         12
#define toolDirPin          13

#define baseStepH           PORTD |=  0b00010000;
#define baseStepL           PORTD &= ~0b00010000;

#define mainStepH           PORTD |=  0b01000000;
#define mainStepL           PORTD &= ~0b01000000;

#define secStepH            PORTB |=  0b00000100;
#define secStepL            PORTB &= ~0b00000100;

#define toolStepH           PORTB |=  0b00010000;
#define toolStepL           PORTB &= ~0b00010000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

#define NUM_STEPPERS 4

char arduinoReady = '1';
char arduinoBusy  = '0';

struct stepperInfo {
  float acceleration;
  volatile unsigned long minStepInterval;
  void (*dirFunc)(int);
  void (*stepFunc)();

  unsigned int c0;
  long stepPosition;

  volatile int dir;
  volatile unsigned int totalSteps;
  volatile bool movementDone = false;
  volatile unsigned int rampUpStepCount;
  volatile unsigned long estStepsToSpeed;
  volatile unsigned long estTimeForMove;
  volatile unsigned long rampUpStepTime;
  volatile float speedScale;

  volatile unsigned int n;
  volatile float d;
  volatile unsigned long di;
  volatile unsigned int stepCount;
};

void positionMotors(){

    while (Serial.available() == 0){/* Do Nothing */}
    while (Serial.available() > 0){
      float baseAng = Serial.parseFloat();

      prepareMovement(0, baseAng);
      prepareMovement(1, baseAng);
      prepareMovement(2, baseAng);
      prepareMovement(3, baseAng);
      
      runAndWait();
      delay(50);
    }

}

volatile stepperInfo steppers[NUM_STEPPERS];

void setup(){

    Serial.begin(115200);

    pinMode(resolutionL1,      OUTPUT);
    pinMode(resolutionL2,      OUTPUT);
    pinMode(resolutionR1,      OUTPUT);
    pinMode(resolutionR2,      OUTPUT);
    pinMode(baseStepPin,       OUTPUT);
    pinMode(baseDirPin,        OUTPUT);
    pinMode(mainStepPin,       OUTPUT);
    pinMode(mainDirPin,        OUTPUT);
    pinMode(secStepPin,        OUTPUT);
    pinMode(secDirPin,         OUTPUT);
    pinMode(toolStepPin,       OUTPUT);
    pinMode(toolDirPin,        OUTPUT);

    digitalWrite(resolutionL1, LOW);
    digitalWrite(resolutionL2, HIGH);
    digitalWrite(resolutionR1, LOW);
    digitalWrite(resolutionR2, HIGH);

    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    OCR1A = 1000;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= ((1 << CS11) | (1 << CS10));
    interrupts();

    steppers[0].dirFunc = baseDir;
    steppers[0].stepFunc = baseStep;
    steppers[0].acceleration = 1000;
    steppers[0].minStepInterval = 50;

    steppers[1].dirFunc = mainDir;
    steppers[1].stepFunc = mainStep;
    steppers[1].acceleration = 1000;
    steppers[1].minStepInterval = 50;

    steppers[2].dirFunc = secDir;
    steppers[2].stepFunc = secStep;
    steppers[2].acceleration = 1000;
    steppers[2].minStepInterval = 50;

    steppers[3].dirFunc = toolDir;
    steppers[3].stepFunc = toolStep;
    steppers[3].acceleration = 1000;
    steppers[3].minStepInterval = 50;
}

void loop()
{
  Serial.write(arduinoReady);
  positionMotors();
}

void baseStep(){
    baseStepH
    baseStepL
}

void baseDir(int dir){
    digitalWrite(baseDirPin, dir);
}

void mainStep(){
    mainStepH
    mainStepL
}

void mainDir(int dir){
    digitalWrite(mainDirPin, dir);
}

void secStep(){
    secStepH
    secStepL
}

void secDir(int dir){
    digitalWrite(secDirPin, dir);
}

void toolStep(){
    toolStepH
    toolStepL
}

void toolDir(int dir){
    digitalWrite(toolDirPin, dir);
}

void resetStepperInfo(struct stepperInfo& si) {
  si.n = 0;
  si.d = 0;
  si.di = 0;
  si.stepCount = 0;
  si.rampUpStepCount = 0;
  si.rampUpStepTime = 0;
  si.totalSteps = 0;
  si.stepPosition = 0;
  si.movementDone = false;
}

void resetStepper(volatile struct stepperInfo& si) {
  si.c0 = si.acceleration;
  si.d = si.c0;
  si.di = si.d;
  si.stepCount = 0;
  si.n = 0;
  si.rampUpStepCount = 0;
  si.movementDone = false;
  si.speedScale = 1;

  float a = si.minStepInterval / (float)si.c0;
  a *= 0.676;

  float m = ((a*a - 1) / (-2 * a));
  float n = m * m;

  si.estStepsToSpeed = n;
}

volatile byte remainingSteppersFlag = 0;

float getDurationOfAcceleration(volatile stepperInfo& s, unsigned int numSteps) {
  float d = s.c0;
  float totalDuration = 0;
  for (unsigned int n = 1; n < numSteps; n++) {
    d = d - (2 * d) / (4 * n + 1);
    totalDuration += d;
  }
  return totalDuration;
}

void prepareMovement(int whichMotor, long steps) {
  volatile stepperInfo& si = steppers[whichMotor];
  si.dirFunc( steps < 0 ? HIGH : LOW );
  si.dir = steps > 0 ? 1 : -1;
  si.totalSteps = abs(steps);
  resetStepper(si);
  
  remainingSteppersFlag |= (1 << whichMotor);

  unsigned long stepsAbs = abs(steps);

  if ( (2 * si.estStepsToSpeed) < stepsAbs ) {
    unsigned long stepsAtFullSpeed = stepsAbs - 2 * si.estStepsToSpeed;
    float accelDecelTime = getDurationOfAcceleration(si, si.estStepsToSpeed);
    si.estTimeForMove = 2 * accelDecelTime + stepsAtFullSpeed * si.minStepInterval;
  }
  else {
    float accelDecelTime = getDurationOfAcceleration( si, stepsAbs / 2 );
    si.estTimeForMove = 2 * accelDecelTime;
  }
}

volatile byte nextStepperFlag = 0;

void setNextInterruptInterval() {

  bool movementComplete = true;

  unsigned long mind = 999999;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di < mind ) {
      mind = steppers[i].di;
    }
  }

  nextStepperFlag = 0;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! steppers[i].movementDone )
      movementComplete = false;
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di == mind )
      nextStepperFlag |= (1 << i);
  }

  if ( remainingSteppersFlag == 0 ) {
    TIMER1_INTERRUPTS_OFF
    OCR1A = 65500;
  }

  OCR1A = mind;
}

ISR(TIMER1_COMPA_vect)
{
  unsigned int tmpCtr = OCR1A;

  OCR1A = 65500;

  for (int i = 0; i < NUM_STEPPERS; i++) {

    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;

    if ( ! (nextStepperFlag & (1 << i)) ) {
      steppers[i].di -= tmpCtr;
      continue;
    }

    volatile stepperInfo& s = steppers[i];

    if ( s.stepCount < s.totalSteps ) {
      s.stepFunc();
      s.stepCount++;
      s.stepPosition += s.dir;
      if ( s.stepCount >= s.totalSteps ) {
        s.movementDone = true;
        remainingSteppersFlag &= ~(1 << i);
      }
    }

    if ( s.rampUpStepCount == 0 ) {
      s.n++;
      s.d = s.d - (2 * s.d) / (4 * s.n + 1);
      if ( s.d <= s.minStepInterval ) {
        s.d = s.minStepInterval;
        s.rampUpStepCount = s.stepCount;
      }
      if ( s.stepCount >= s.totalSteps / 2 ) {
        s.rampUpStepCount = s.stepCount;
      }
      s.rampUpStepTime += s.d;
    }
    else if ( s.stepCount >= s.totalSteps - s.rampUpStepCount ) {
      s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);
      s.n--;
    }

    s.di = s.d * s.speedScale; // integer
  }

  setNextInterruptInterval();

  TCNT1  = 0;
}

void runAndWait() {
  adjustSpeedScales();
  setNextInterruptInterval();
  TIMER1_INTERRUPTS_ON
  while ( remainingSteppersFlag );
  remainingSteppersFlag = 0;
  nextStepperFlag = 0;
}

void adjustSpeedScales() {
  float maxTime = 0;
  
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;
    if ( steppers[i].estTimeForMove > maxTime )
      maxTime = steppers[i].estTimeForMove;
  }

  if ( maxTime != 0 ) {
    for (int i = 0; i < NUM_STEPPERS; i++) {
      if ( ! ( (1 << i) & remainingSteppersFlag) )
        continue;
      steppers[i].speedScale = maxTime / steppers[i].estTimeForMove;
    }
  }
}
