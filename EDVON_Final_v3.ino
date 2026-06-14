// ── Motor Pins ───────────────────────────────────────────────
#define LEFT_DIR1   2
#define LEFT_PWM    3
#define LEFT_DIR2   4
#define RIGHT_DIR1  5
#define RIGHT_PWM   6
#define RIGHT_DIR2  7

// ── IR Sensor Pins ───────────────────────────────────────────
#define SENSOR_LEFT    A4
#define SENSOR_CENTER  A3
#define SENSOR_RIGHT   A2

// ── Per-sensor thresholds (CALIBRATED — adjust if needed) ────
#define THRESHOLD_LEFT    715
#define THRESHOLD_CENTER  820
#define THRESHOLD_RIGHT   445

// ── Speed Settings ───────────────────────────────────────────
const uint8_t SPD_FWD    =  80;
const uint8_t SPD_FAST   =  90;
const uint8_t SPD_SLOW   =  40;
const uint8_t SPD_SEARCH =  70;

const uint32_t SEARCH_TIMEOUT = 2000;

int8_t   lastDir  = 0;
uint32_t lostTime = 0;
bool     lost     = false;

// ============================================================
//  SENSOR READING — per-sensor calibrated threshold
// ============================================================

bool seeBlack(uint8_t pin, int threshold) {
  return analogRead(pin) > threshold;
}

// ============================================================
//  MOTOR CONTROL
// ============================================================

void stopMotors() {
  digitalWrite(LEFT_DIR1,  LOW); analogWrite(LEFT_PWM,  0); digitalWrite(LEFT_DIR2,  LOW);
  digitalWrite(RIGHT_DIR1, LOW); analogWrite(RIGHT_PWM, 0); digitalWrite(RIGHT_DIR2, LOW);
}

void leftFwd(uint8_t spd)  { digitalWrite(LEFT_DIR1, HIGH); analogWrite(LEFT_PWM, spd);  digitalWrite(LEFT_DIR2, LOW);  }
void leftRev(uint8_t spd)  { digitalWrite(LEFT_DIR1, LOW);  analogWrite(LEFT_PWM, spd);  digitalWrite(LEFT_DIR2, HIGH); }
void leftOff()             { digitalWrite(LEFT_DIR1, LOW);  analogWrite(LEFT_PWM, 0);    digitalWrite(LEFT_DIR2, LOW);  }
void rightFwd(uint8_t spd) { digitalWrite(RIGHT_DIR1,HIGH); analogWrite(RIGHT_PWM, spd); digitalWrite(RIGHT_DIR2, LOW); }
void rightRev(uint8_t spd) { digitalWrite(RIGHT_DIR1,LOW);  analogWrite(RIGHT_PWM, spd); digitalWrite(RIGHT_DIR2,HIGH); }
void rightOff()            { digitalWrite(RIGHT_DIR1,LOW);  analogWrite(RIGHT_PWM, 0);   digitalWrite(RIGHT_DIR2,LOW);  }

// ============================================================
//  MOVEMENTS
// ============================================================

void goForward()  { leftFwd(SPD_FWD);  rightFwd(SPD_FWD);  }
void gentleLeft() { leftFwd(SPD_SLOW); rightFwd(SPD_FWD);  }
void gentleRight(){ leftFwd(SPD_FWD);  rightFwd(SPD_SLOW); }
void sharpLeft()  { leftOff();         rightFwd(SPD_FAST);  }
void sharpRight() { leftFwd(SPD_FAST); rightOff();          }
void searchLeft() { leftRev(SPD_SEARCH); rightFwd(SPD_SEARCH); }
void searchRight(){ leftFwd(SPD_SEARCH); rightRev(SPD_SEARCH); }

// ============================================================
//  LINE FOLLOW LOGIC
// ============================================================

void followLine() {
  bool L = seeBlack(SENSOR_LEFT,   THRESHOLD_LEFT);
  bool C = seeBlack(SENSOR_CENTER, THRESHOLD_CENTER);
  bool R = seeBlack(SENSOR_RIGHT,  THRESHOLD_RIGHT);

  // All on — crossing, go straight
  if (L && C && R)   { goForward();   lastDir= 0; lost=false; return; }

  // Centre only — on line, go straight
  if (C && !L && !R) { goForward();   lastDir= 0; lost=false; return; }

  // Centre + Left — correct left gently
  if (C && L && !R)  { gentleLeft();  lastDir=-1; lost=false; return; }

  // Centre + Right — correct right gently
  if (C && R && !L)  { gentleRight(); lastDir= 1; lost=false; return; }

  // Left only — sharp left corner
  if (L && !C && !R) { sharpLeft();   lastDir=-1; lost=false; return; }

  // Right only — sharp right corner
  if (R && !C && !L) { sharpRight();  lastDir= 1; lost=false; return; }

  // Left + Right no centre — straddle, go straight
  if (L && R && !C)  { goForward();   lost=false; return; }

  // Nothing — line lost, search
  if (!lost) { lostTime = millis(); lost = true; }
  if (millis() - lostTime > SEARCH_TIMEOUT) { stopMotors(); return; }
  if (lastDir <= 0) searchLeft(); else searchRight();
}

// ============================================================
//  SETUP & LOOP
// ============================================================

void setup() {
  pinMode(LEFT_DIR1,  OUTPUT);
  pinMode(LEFT_PWM,   OUTPUT);
  pinMode(LEFT_DIR2,  OUTPUT);
  pinMode(RIGHT_DIR1, OUTPUT);
  pinMode(RIGHT_PWM,  OUTPUT);
  pinMode(RIGHT_DIR2, OUTPUT);

  pinMode(SENSOR_LEFT,   INPUT);
  pinMode(SENSOR_CENTER, INPUT);
  pinMode(SENSOR_RIGHT,  INPUT);

  stopMotors();
  delay(2000);
}

void loop() {
  followLine();
}
