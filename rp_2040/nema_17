#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

#define STEP_1 3
#define DIR_1 2
#define EN_1 4
#define STEP_2 9
#define DIR_2 8
#define EN_2 7

const float BASE = 42.0f;
const float LP1 = 160.0f;
const float LD1 = 175.0f;
const float LP2 = 160.0f;
const float LD2 = 175.0f;
const float CELL = 28.0f;
const long STEPS_PER_REV = 1600;
const int DIR_SIGN_1 = 1;
const int DIR_SIGN_2 = 1;
const float HOME_DEG_1 = 160.10f;
const float HOME_DEG_2 = 19.90f;
const float FIELD_X_CENTER = 126.0f;
const float ROW_E_DIST = 140.0f;
const float FIELD_ROT_DEG = 0.0f;
float vMax = 80.0f;
const float V_START = 50.0f;
const float ACCEL = 150.0f;
const float SEG_MM = 0.5f;
const float JOINT_SPEED = 150.0f;
const float MAX_REACH_FRAC = 0.98f;
const float MIN_SIN = 0.20f;

struct Pose {
  float th1, th2;
  float reach;
  float sinD;
};

bool solve(float x, float y, Pose &p) {
  float dx1 = x + BASE * 0.5f;
  float dx2 = x - BASE * 0.5f;
  float r1 = sqrtf(dx1 * dx1 + y * y);
  float r2 = sqrtf(dx2 * dx2 + y * y);
  float max1 = LP1 + LD1, max2 = LP2 + LD2;
  if (r1 >= max1 || r2 >= max2 || r1 < 1.0f || r2 < 1.0f) return false;
  float c1 = (LP1 * LP1 + r1 * r1 - LD1 * LD1) / (2.0f * LP1 * r1);
  float c2 = (LP2 * LP2 + r2 * r2 - LD2 * LD2) / (2.0f * LP2 * r2);
  if (c1 > 1.0f || c1 < -1.0f || c2 > 1.0f || c2 < -1.0f) return false;
  p.th1 = atan2f(y, dx1) + acosf(c1);
  p.th2 = atan2f(y, dx2) - acosf(c2);
  float f1 = r1 / max1, f2 = r2 / max2;
  p.reach = f1 > f2 ? f1 : f2;
  float e1x = -BASE * 0.5f + LP1 * cosf(p.th1), e1y = LP1 * sinf(p.th1);
  float e2x = BASE * 0.5f + LP2 * cosf(p.th2), e2y = LP2 * sinf(p.th2);
  float w1x = x - e1x, w1y = y - e1y;
  float w2x = x - e2x, w2y = y - e2y;
  p.sinD = fabsf(w1x * w2y - w1y * w2x) / (LD1 * LD2);
  return true;
}

bool isSafe(const Pose &p) {
  return p.reach <= MAX_REACH_FRAC && p.sinD >= MIN_SIN;
}

bool forward(float th1, float th2, float &x, float &y) {
  float e1x = -BASE * 0.5f + LP1 * cosf(th1), e1y = LP1 * sinf(th1);
  float e2x = BASE * 0.5f + LP2 * cosf(th2), e2y = LP2 * sinf(th2);
  float dx = e2x - e1x, dy = e2y - e1y;
  float d = sqrtf(dx * dx + dy * dy);
  if (d < 1.0f || d > LD1 + LD2 + 0.5f) return false;
  float a = (LD1 * LD1 - LD2 * LD2 + d * d) / (2.0f * d);
  float h2 = LD1 * LD1 - a * a;
  float h = h2 > 0.0f ? sqrtf(h2) : 0.0f;
  x = e1x + (a * dx - h * dy) / d;
  y = e1y + (a * dy + h * dx) / d;
  return true;
}

float homeX, homeY;
float homeTh1, homeTh2;
long cur1 = 0, cur2 = 0;
float curX, curY;
bool atHome = true;

long toSteps(float th, float thHome) {
  return lroundf((th - thHome) * ((float)STEPS_PER_REV / (2.0f * PI)));
}

inline void pulse(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(3);
  digitalWrite(pin, LOW);
}

void runSteps(long d1, long d2, uint32_t durUs, uint32_t t0) {
  long a1 = labs(d1), a2 = labs(d2);
  long n = a1 > a2 ? a1 : a2;
  digitalWrite(DIR_1, ((d1 >= 0) == (DIR_SIGN_1 > 0)) ? HIGH : LOW);
  digitalWrite(DIR_2, ((d2 >= 0) == (DIR_SIGN_2 > 0)) ? HIGH : LOW);
  delayMicroseconds(5);
  if (n == 0) {
    while ((uint32_t)(micros() - t0) < durUs) {}
    return;
  }
  float dt = (float)durUs / (float)n;
  long e1 = n / 2, e2 = n / 2;
  for (long k = 1; k <= n; k++) {
    uint32_t due = (uint32_t)(dt * (float)k);
    while ((uint32_t)(micros() - t0) < due) {}
    e1 += a1;
    if (e1 >= n) {
      e1 -= n;
      pulse(STEP_1);
    }
    e2 += a2;
    if (e2 >= n) {
      e2 -= n;
      pulse(STEP_2);
    }
  }
}

void jointMoveTo(float th1, float th2, float stepsPerSec) {
  long s1 = toSteps(th1, homeTh1), s2 = toSteps(th2, homeTh2);
  long d1 = s1 - cur1, d2 = s2 - cur2;
  long n = labs(d1) > labs(d2) ? labs(d1) : labs(d2);
  if (n == 0) return;
  uint32_t dur = (uint32_t)((float)n / stepsPerSec * 1.0e6f);
  runSteps(d1, d2, dur, micros());
  cur1 = s1;
  cur2 = s2;
}

bool lineOk(float x0, float y0, float x1, float y1) {
  float dx = x1 - x0, dy = y1 - y0;
  float len = sqrtf(dx * dx + dy * dy);
  int n = (int)ceilf(len / SEG_MM);
  if (n < 1) n = 1;
  for (int i = 0; i <= n; i++) {
    float f = (float)i / (float)n;
    Pose p;
    if (!solve(x0 + dx * f, y0 + dy * f, p) || !isSafe(p)) return false;
  }
  return true;
}

bool leaveHome() {
  Pose r;
  if (!solve(0.0f, ROW_E_DIST, r) || !isSafe(r)) {
    Serial.println("err: start");
    return false;
  }
  jointMoveTo(r.th1, r.th2, JOINT_SPEED * 0.5f);
  curX = 0.0f;
  curY = ROW_E_DIST;
  atHome = false;
  return true;
}

bool moveLine(float x1, float y1) {
  if (atHome && !leaveHome()) return false;
  float x0 = curX, y0 = curY;
  if (!lineOk(x0, y0, x1, y1)) {
    Serial.println("err: bounds");
    return false;
  }
  float dx = x1 - x0, dy = y1 - y0;
  float len = sqrtf(dx * dx + dy * dy);
  if (len < 0.01f) return true;
  int n = (int)ceilf(len / SEG_MM);
  if (n < 1) n = 1;
  float ds = len / (float)n;
  for (int i = 1; i <= n; i++) {
    uint32_t t0 = micros();
    float f = (float)i / (float)n;
    float x = x0 + dx * f, y = y0 + dy * f;
    Pose p;
    solve(x, y, p);
    long s1 = toSteps(p.th1, homeTh1), s2 = toSteps(p.th2, homeTh2);
    float smid = ((float)i - 0.5f) * ds;
    float v = vMax;
    float va = sqrtf(V_START * V_START + 2.0f * ACCEL * smid);
    float vb = sqrtf(V_START * V_START + 2.0f * ACCEL * (len - smid));
    if (va < v) v = va;
    if (vb < v) v = vb;
    uint32_t dur = (uint32_t)(ds / v * 1.0e6f);
    runSteps(s1 - cur1, s2 - cur2, dur, t0);
    cur1 = s1;
    cur2 = s2;
    curX = x;
    curY = y;
  }
  return true;
}

void goHome() {
  if (atHome) return;
  if (!moveLine(0.0f, ROW_E_DIST)) return;
  jointMoveTo(homeTh1, homeTh2, JOINT_SPEED * 0.5f);
  curX = homeX;
  curY = homeY;
  atHome = true;
}

void fieldToRobot(float X, float Y, float &x, float &y) {
  float c = cosf(FIELD_ROT_DEG * PI / 180.0f);
  float s = sinf(FIELD_ROT_DEG * PI / 180.0f);
  x = FIELD_X_CENTER - X * c + Y * s;
  y = ROW_E_DIST + 6.0f * CELL - X * s - Y * c;
}

bool moveToField(float X, float Y) {
  float x, y;
  fieldToRobot(X, Y, x, y);
  return moveLine(x, y);
}

bool moveToCell(int n, int k) {
  if (n < 0 || n > 9 || k < 1 || k > 6 || (n == 9 && k == 6)) {
    return false;
  }
  return moveToField(n * CELL, k * CELL);
}

void handle(char *s) {
  char c = (char)toupper(s[0]);
  char *p = s + 1;
  float a = (float)strtod(p, &p);
  float b = (float)strtod(p, &p);
  if (c == 'H') {
    goHome();
  } else if (c == 'C') {
    if (!moveToCell((int)a, (int)b)) Serial.println("err: cell");
  } else {
    Serial.println("H / C n k");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(STEP_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(EN_1, OUTPUT);
  pinMode(STEP_2, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(EN_2, OUTPUT);
  digitalWrite(EN_1, LOW);
  digitalWrite(EN_2, LOW);
  homeTh1 = HOME_DEG_1 * PI / 180.0f;
  homeTh2 = HOME_DEG_2 * PI / 180.0f;
  homeX = 0.0f;
  homeY = 0.0f;
  forward(homeTh1, homeTh2, homeX, homeY);
  curX = homeX;
  curY = homeY;
  delay(500);
  Serial.println("ready");
}

void loop() {
  static char buf[40];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) {
        buf[len] = 0;
        handle(buf);
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}
