// ============================================================
// raw_circle_draw.ino
//
// Standalone motion test: homes X and Y, then drives the two
// axes together to trace a circle -- no servo, no serial G-code
// parsing. Just verifies the mechanical motion works before you
// mount the pen or wire up TTL streaming.
//
// X motor:  STEP 8, DIR 9, EN 10, limit 2
// Y motor:  DIR 5, STEP 6, EN 7,  limit 3
// ============================================================

#include <math.h>

// ---------------- X axis ----------------
#define STEP_PIN_X        8
#define DIR_PIN_X         9
#define EN_PIN_X          10
#define LIMIT_PIN_X       2
#define STEPS_PER_MM_X    80.0
#define MAX_MM_X          275.0
#define HOMING_DELAY_US_X 500
#define BACKOFF_STEPS_X   100
#define TOWARD_HOME_X     HIGH
#define AWAY_FROM_HOME_X  LOW

// ---------------- Y axis ----------------
#define DIR_PIN_Y         5
#define STEP_PIN_Y        6
#define EN_PIN_Y          7
#define LIMIT_PIN_Y       3
#define STEPS_PER_MM_Y    80.0
#define MAX_MM_Y          190.0
#define HOMING_DELAY_US_Y 500
#define BACKOFF_STEPS_Y   100
#define TOWARD_HOME_Y     HIGH
#define AWAY_FROM_HOME_Y  LOW

// ---------------- circle parameters ----------------
#define CENTER_X_MM   137.5   // middle of 275mm X travel
#define CENTER_Y_MM   95.0    // middle of 190mm Y travel
#define RADIUS_MM     60.0
#define SEGMENTS      144     // 2.5 degree steps -> smooth circle

#define DRAW_DELAY_US 500     // step half-pulse width while tracing

long curXsteps = 0;
long curYsteps = 0;

void stepPulse(int stepPin, int delayUs) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayUs);
}

void homeAxis(int stepPin, int dirPin, int limitPin, int towardHomeDir,
              int homingDelayUs, int backoffSteps) {
  int awayDir = (towardHomeDir == HIGH) ? LOW : HIGH;

  digitalWrite(dirPin, towardHomeDir);
  while (digitalRead(limitPin) == HIGH) stepPulse(stepPin, homingDelayUs);

  digitalWrite(dirPin, awayDir);
  for (int i = 0; i < backoffSteps; i++) stepPulse(stepPin, homingDelayUs);

  digitalWrite(dirPin, towardHomeDir);
  while (digitalRead(limitPin) == HIGH) stepPulse(stepPin, homingDelayUs * 2);
}

void homeAll() {
  homeAxis(STEP_PIN_X, DIR_PIN_X, LIMIT_PIN_X, TOWARD_HOME_X, HOMING_DELAY_US_X, BACKOFF_STEPS_X);
  homeAxis(STEP_PIN_Y, DIR_PIN_Y, LIMIT_PIN_Y, TOWARD_HOME_Y, HOMING_DELAY_US_Y, BACKOFF_STEPS_Y);
  curXsteps = 0;
  curYsteps = 0;
}

long clampSteps(long s, float maxMm, float stepsPerMm) {
  long maxSteps = (long)(maxMm * stepsPerMm);
  if (s < 0)        return 0;
  if (s > maxSteps) return maxSteps;
  return s;
}

// Coordinated linear move to absolute (tx, ty) in steps -- Bresenham
// timed so the shorter axis' pulses spread evenly for a straight line.
void moveLinear(long tx, long ty, int delayUs) {
  tx = clampSteps(tx, MAX_MM_X, STEPS_PER_MM_X);
  ty = clampSteps(ty, MAX_MM_Y, STEPS_PER_MM_Y);

  long dx = tx - curXsteps;
  long dy = ty - curYsteps;

  digitalWrite(DIR_PIN_X, (dx >= 0) ? AWAY_FROM_HOME_X : TOWARD_HOME_X);
  digitalWrite(DIR_PIN_Y, (dy >= 0) ? AWAY_FROM_HOME_Y : TOWARD_HOME_Y);

  long ax = labs(dx);
  long ay = labs(dy);
  long n  = (ax > ay) ? ax : ay;
  if (n == 0) return;

  long errX = n / 2;
  long errY = n / 2;

  for (long i = 0; i < n; i++) {
    bool sx = false, sy = false;

    errX -= ax; if (errX < 0) { errX += n; sx = true; }
    errY -= ay; if (errY < 0) { errY += n; sy = true; }

    if (sx) digitalWrite(STEP_PIN_X, HIGH);
    if (sy) digitalWrite(STEP_PIN_Y, HIGH);
    delayMicroseconds(delayUs);
    if (sx) digitalWrite(STEP_PIN_X, LOW);
    if (sy) digitalWrite(STEP_PIN_Y, LOW);
    delayMicroseconds(delayUs);
  }

  curXsteps = tx;
  curYsteps = ty;
}

void moveToMm(float xmm, float ymm, int delayUs) {
  long tx = lround(xmm * STEPS_PER_MM_X);
  long ty = lround(ymm * STEPS_PER_MM_Y);
  moveLinear(tx, ty, delayUs);
}

void drawCircle() {
  // travel to the start point of the circle first (angle 0)
  moveToMm(CENTER_X_MM + RADIUS_MM, CENTER_Y_MM, DRAW_DELAY_US);

  for (int i = 1; i <= SEGMENTS; i++) {
    float ang = 2.0 * PI * i / SEGMENTS;
    float x = CENTER_X_MM + RADIUS_MM * cos(ang);
    float y = CENTER_Y_MM + RADIUS_MM * sin(ang);
    moveToMm(x, y, DRAW_DELAY_US);
  }

  // return to center so it's obvious the circle closed correctly
  moveToMm(CENTER_X_MM, CENTER_Y_MM, DRAW_DELAY_US);
}

void setup() {
  Serial.begin(9600);

  pinMode(STEP_PIN_X, OUTPUT); pinMode(DIR_PIN_X, OUTPUT); pinMode(EN_PIN_X, OUTPUT);
  pinMode(LIMIT_PIN_X, INPUT_PULLUP);
  pinMode(STEP_PIN_Y, OUTPUT); pinMode(DIR_PIN_Y, OUTPUT); pinMode(EN_PIN_Y, OUTPUT);
  pinMode(LIMIT_PIN_Y, INPUT_PULLUP);

  digitalWrite(EN_PIN_X, LOW);  // active-low: LOW = driver enabled
  digitalWrite(EN_PIN_Y, LOW);

  Serial.println("Homing...");
  homeAll();
  Serial.println("Homed. Drawing circle...");

  drawCircle();

  Serial.println("Circle done.");
}

void loop() {
  // done — nothing to repeat. Reset the board to run again.
}
