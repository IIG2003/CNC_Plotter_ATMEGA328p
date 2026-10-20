// ============================================================
// xy_home_and_max.ino
//
// Homes X then Y using their limit switches, then drives each
// axis out to its tuned max travel. Per-axis timing/direction
// values below are the ones already confirmed working for each
// motor individually.
//
// X motor:  STEP -> 8, DIR -> 9,  EN -> 10, limit -> pin 2
// Y motor:  DIR  -> 5, STEP -> 6, EN -> 7,  limit -> pin 3
// ============================================================

// ---------------- X axis ----------------
#define STEP_PIN_X   8
#define DIR_PIN_X    9
#define EN_PIN_X     10
#define LIMIT_PIN_X  2

#define STEPS_PER_MM_X     80.0
#define MAX_MM_X           275.0

#define HOMING_DELAY_US_X  500
#define RUN_DELAY_US_X     400
#define BACKOFF_STEPS_X    100

#define TOWARD_HOME_X      HIGH
#define AWAY_FROM_HOME_X   LOW

// ---------------- Y axis ----------------
#define DIR_PIN_Y    5
#define STEP_PIN_Y   6
#define EN_PIN_Y     7
#define LIMIT_PIN_Y  3

#define STEPS_PER_MM_Y     80.0
#define MAX_MM_Y           190.0

#define HOMING_DELAY_US_Y  500
#define RUN_DELAY_US_Y     300
#define BACKOFF_STEPS_Y    100

#define TOWARD_HOME_Y      HIGH
#define AWAY_FROM_HOME_Y   LOW

void setup() {
  Serial.begin(9600);

  pinMode(STEP_PIN_X, OUTPUT);
  pinMode(DIR_PIN_X, OUTPUT);
  pinMode(EN_PIN_X, OUTPUT);
  pinMode(LIMIT_PIN_X, INPUT_PULLUP);

  pinMode(STEP_PIN_Y, OUTPUT);
  pinMode(DIR_PIN_Y, OUTPUT);
  pinMode(EN_PIN_Y, OUTPUT);
  pinMode(LIMIT_PIN_Y, INPUT_PULLUP);

  digitalWrite(EN_PIN_X, LOW);  // active-low: LOW = driver enabled
  digitalWrite(EN_PIN_Y, LOW);

  Serial.println("Homing X...");
  homeAxis(STEP_PIN_X, DIR_PIN_X, LIMIT_PIN_X, TOWARD_HOME_X, HOMING_DELAY_US_X, BACKOFF_STEPS_X);
  Serial.println("X homed (0).");

  Serial.println("Homing Y...");
  homeAxis(STEP_PIN_Y, DIR_PIN_Y, LIMIT_PIN_Y, TOWARD_HOME_Y, HOMING_DELAY_US_Y, BACKOFF_STEPS_Y);
  Serial.println("Y homed (0).");

  delay(500);

  Serial.println("Moving X to max...");
  moveToMax(STEP_PIN_X, DIR_PIN_X, AWAY_FROM_HOME_X, MAX_MM_X, STEPS_PER_MM_X, RUN_DELAY_US_X);
  Serial.println("X at max.");

  Serial.println("Moving Y to max...");
  moveToMax(STEP_PIN_Y, DIR_PIN_Y, AWAY_FROM_HOME_Y, MAX_MM_Y, STEPS_PER_MM_Y, RUN_DELAY_US_Y);
  Serial.println("Y at max.");

  Serial.println("Done.");
}

void loop() {
  // done — nothing to repeat. Reset the board to run again.
}

void stepPulse(int stepPin, int delayUs) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayUs);
}

// Drive toward the switch until triggered, back off, then
// re-approach slowly for a repeatable zero point.
void homeAxis(int stepPin, int dirPin, int limitPin, int towardHomeDir,
              int homingDelayUs, int backoffSteps) {
  int awayDir = (towardHomeDir == HIGH) ? LOW : HIGH;

  digitalWrite(dirPin, towardHomeDir);
  while (digitalRead(limitPin) == HIGH) {
    stepPulse(stepPin, homingDelayUs);
  }

  digitalWrite(dirPin, awayDir);
  for (int i = 0; i < backoffSteps; i++) {
    stepPulse(stepPin, homingDelayUs);
  }

  digitalWrite(dirPin, towardHomeDir);
  while (digitalRead(limitPin) == HIGH) {
    stepPulse(stepPin, homingDelayUs * 2);
  }

  // this is now position 0 for this axis
}

// Drive one axis from 0 to its max travel
void moveToMax(int stepPin, int dirPin, int awayFromHomeDir,
               float maxMm, float stepsPerMm, int runDelayUs) {
  long steps = (long)(maxMm * stepsPerMm);

  digitalWrite(dirPin, awayFromHomeDir);
  for (long i = 0; i < steps; i++) {
    stepPulse(stepPin, runDelayUs);
  }
}
