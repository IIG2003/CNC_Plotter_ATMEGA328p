// ============================================================
// xy_gcode_plotter_no_servo.ino
//
// Same as xy_gcode_plotter.ino, but with the pen servo removed
// for now. M3/M4/M5 and Z-triggered pen moves are still parsed
// (so your .gcode files don't need editing) but they just print
// a message instead of moving a servo. Swap penDown()/penUp()
// back to the Servo.write() version once the servo is mounted --
// see the commented block at the bottom.
//
// Homes X then Y on boot, then listens on serial (TTL) for
// G-code and executes it. Sends "ok" after EVERY line so a
// streaming host can flow-control.
//
// X motor:  STEP 8, DIR 9, EN 10, limit 2
// Y motor:  DIR 5, STEP 6, EN 7,  limit 3
// ============================================================

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
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

// ---------------- pen (servo not mounted yet) ----------------
#define PEN_Z_THRESHOLD   0.0     // Z at or below this = pen down

// ---------------- speeds (half-pulse delay, us) ----------------
#define DRAW_DELAY_US     350     // G1 pen-down feed
#define TRAVEL_DELAY_US   180     // G0 rapid
#define MIN_DELAY_US      150     // fastest a step is ever allowed

long  curXsteps    = 0;
long  curYsteps    = 0;
bool  absoluteMode = true;
float unitScale    = 1.0;   // 1.0 = mm, 25.4 = inches
bool  penIsDown    = false;

char    lineBuf[96];
uint8_t lineIdx = 0;

// ------------------------------------------------------------
void stepPulse(int stepPin, int delayUs) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayUs);
}

// ---- pen control: stubbed until the servo is mounted ----
void penDown() {
  if (penIsDown) return;
  Serial.println("(pen down - servo not mounted, no-op)");
  penIsDown = true;
}

void penUp() {
  if (!penIsDown) return;
  Serial.println("(pen up - servo not mounted, no-op)");
  penIsDown = false;
}

// Drive toward the switch until triggered, back off, re-approach slowly.
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
  penUp();
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

// Coordinated linear move to absolute (tx, ty) in steps, Bresenham-timed
// so the shorter axis' pulses are spread evenly -> straight diagonals.
void moveLinear(long tx, long ty, int delayUs) {
  tx = clampSteps(tx, MAX_MM_X, STEPS_PER_MM_X);   // soft limits
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

// Pull a numeric value that follows `key` (e.g. 'X'). Returns false if absent.
bool getVal(const char *s, char key, float &out) {
  const char *p = strchr(s, key);
  if (!p) return false;
  out = atof(p + 1);
  return true;
}

// Approx feed (mm/min) -> per-step half-delay. Uses X scale; clamped.
int feedToDelay(float feed_mm_min) {
  if (feed_mm_min <= 0) return DRAW_DELAY_US;
  float sps = (feed_mm_min / 60.0) * STEPS_PER_MM_X;
  if (sps <= 0) return DRAW_DELAY_US;
  long d = (long)(1000000.0 / (2.0 * sps));
  if (d < MIN_DELAY_US) d = MIN_DELAY_US;
  if (d > 4000)         d = 4000;
  return (int)d;
}

void processLine(char *line) {
  // strip comments: everything after ';' and anything from '('
  char *c;
  if ((c = strchr(line, ';')) != NULL) *c = 0;
  if ((c = strchr(line, '(')) != NULL) *c = 0;

  for (char *p = line; *p; p++) *p = toupper(*p);

  bool hasContent = false;
  for (char *p = line; *p; p++) if (!isspace(*p)) { hasContent = true; break; }
  if (!hasContent) { Serial.println("ok"); return; }

  float g, m, z, f, xmm, ymm;

  // ---- modal / non-motion codes ----
  if (getVal(line, 'G', g)) {
    int gi = (int)g;
    if      (gi == 20) unitScale = 25.4;
    else if (gi == 21) unitScale = 1.0;
    else if (gi == 90) absoluteMode = true;
    else if (gi == 91) absoluteMode = false;
    else if (gi == 28) homeAll();
  }

  if (getVal(line, 'M', m)) {
    int mi = (int)m;
    if      (mi == 3 || mi == 4) penDown();
    else if (mi == 5)            penUp();
    // M2 / M30 -> program end, nothing to do
  }

  if (getVal(line, 'Z', z)) {
    if (z * unitScale <= PEN_Z_THRESHOLD) penDown();
    else                                  penUp();
  }

  // ---- motion ----
  bool isG0 = false, isG1 = false;
  if (getVal(line, 'G', g)) {
    int gi = (int)g;
    if      (gi == 0) isG0 = true;
    else if (gi == 1) isG1 = true;
  }

  if (isG0 || isG1) {
    long tx = curXsteps;
    long ty = curYsteps;

    if (getVal(line, 'X', xmm)) {
      long xs = lround(xmm * unitScale * STEPS_PER_MM_X);
      tx = absoluteMode ? xs : curXsteps + xs;
    }
    if (getVal(line, 'Y', ymm)) {
      long ys = lround(ymm * unitScale * STEPS_PER_MM_Y);
      ty = absoluteMode ? ys : curYsteps + ys;
    }

    int delayUs = isG0 ? TRAVEL_DELAY_US : DRAW_DELAY_US;
    if (getVal(line, 'F', f)) delayUs = feedToDelay(f * unitScale);

    moveLinear(tx, ty, delayUs);
  }

  Serial.println("ok");
}

// ------------------------------------------------------------
void setup() {
  Serial.begin(9600);   // match your host baud; 115200 is fine if the host does too

  pinMode(STEP_PIN_X, OUTPUT); pinMode(DIR_PIN_X, OUTPUT); pinMode(EN_PIN_X, OUTPUT);
  pinMode(LIMIT_PIN_X, INPUT_PULLUP);
  pinMode(STEP_PIN_Y, OUTPUT); pinMode(DIR_PIN_Y, OUTPUT); pinMode(EN_PIN_Y, OUTPUT);
  pinMode(LIMIT_PIN_Y, INPUT_PULLUP);

  digitalWrite(EN_PIN_X, LOW);  // active-low: LOW = driver enabled
  digitalWrite(EN_PIN_Y, LOW);

  Serial.println("Homing...");
  homeAll();
  Serial.println("Ready (pen servo not mounted yet — pen commands are no-ops)");
  Serial.println("ok");        // primes hosts that wait for a first ok
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineIdx > 0) {
        lineBuf[lineIdx] = 0;
        processLine(lineBuf);
        lineIdx = 0;
      }
    } else if (lineIdx < sizeof(lineBuf) - 1) {
      lineBuf[lineIdx++] = ch;
    }
  }
}

/* ============================================================
 * ONCE THE SERVO IS MOUNTED, restore real pen control like this:
 *
 * 1. Add near the top:
 *      #include <Servo.h>
 *      #define PEN_SERVO_PIN   11
 *      #define PEN_UP_ANGLE    40   // tune to your linkage
 *      #define PEN_DOWN_ANGLE  90   // tune to your linkage
 *      #define PEN_MOVE_MS     180
 *      Servo penServo;
 *
 * 2. Replace penDown()/penUp() with:
 *      void penDown() {
 *        if (penIsDown) return;
 *        penServo.write(PEN_DOWN_ANGLE);
 *        delay(PEN_MOVE_MS);
 *        penIsDown = true;
 *      }
 *      void penUp() {
 *        if (!penIsDown) return;
 *        penServo.write(PEN_UP_ANGLE);
 *        delay(PEN_MOVE_MS);
 *        penIsDown = false;
 *      }
 *
 * 3. In setup(), before homeAll():
 *      penServo.attach(PEN_SERVO_PIN);
 *      penServo.write(PEN_UP_ANGLE);
 *      penIsDown = false;
 *      delay(PEN_MOVE_MS);
 * ============================================================ */
