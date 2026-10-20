// ============================================================
// y_axis_home_and_max.ino
//
// Single axis test: homes using the limit switch on pin 3, then
// drives to the far end of travel (MAX_MM).
//
// Motor:  DIR -> 5, STEP -> 6, EN -> 7
// Switch: pin 3, COM -> pin 3, NO -> GND, INPUT_PULLUP
//
// If it drives INTO the switch during homing instead of toward
// it, flip TOWARD_HOME. If it drives back toward the switch
// during the max-travel move instead of away from it, flip
// AWAY_FROM_HOME.
// ============================================================

#define DIR_PIN    5
#define STEP_PIN   6
#define EN_PIN     7
#define LIMIT_PIN  3

#define STEPS_PER_MM   80.0   // 1/16 microstepping (MS1-3 all HIGH), 20T GT2 pulley
#define MAX_MM         190.0  // Y-axis travel

#define HOMING_DELAY_US   2000   // slow speed while homing — safer near hard-stops
#define RUN_DELAY_US      300    // normal speed for the max-travel move (tune per motor)
#define BACKOFF_STEPS     100    // steps to back off after first contact

// Direction that drives the axis TOWARD the switch (used while homing)
#define TOWARD_HOME   LOW

// Direction that drives the axis AWAY from the switch (used to reach MAX_MM)
#define AWAY_FROM_HOME   HIGH

void setup() {
  Serial.begin(9600);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(LIMIT_PIN, INPUT_PULLUP);

  digitalWrite(EN_PIN, LOW);  // active-low: LOW = driver enabled

  Serial.println("Homing Y...");
  homeAxis();
  Serial.println("Homed (0). Moving to max...");

  delay(500);
  moveToMax();
  Serial.println("Reached max travel. Done.");
}

void loop() {
  // done — nothing to repeat. Reset the board to run again.
}

void stepPulse(int delayUs) {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delayUs);
}

// Drive toward the switch until triggered, back off, then
// re-approach slowly for a repeatable zero point.
void homeAxis() {
  int awayDir = (TOWARD_HOME == HIGH) ? LOW : HIGH;

  digitalWrite(DIR_PIN, TOWARD_HOME);
  while (digitalRead(LIMIT_PIN) == HIGH) {
    stepPulse(HOMING_DELAY_US);
  }

  digitalWrite(DIR_PIN, awayDir);
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    stepPulse(HOMING_DELAY_US);
  }

  digitalWrite(DIR_PIN, TOWARD_HOME);
  while (digitalRead(LIMIT_PIN) == HIGH) {
    stepPulse(HOMING_DELAY_US * 2);
  }

  // this is now position 0
}

// Drive from 0 to MAX_MM
void moveToMax() {
  long steps = (long)(MAX_MM * STEPS_PER_MM);

  digitalWrite(DIR_PIN, AWAY_FROM_HOME);
  for (long i = 0; i < steps; i++) {
    stepPulse(RUN_DELAY_US);
  }
}
