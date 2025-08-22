#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Servo.h>

Adafruit_INA219 ina219;
Servo myServo;

// -------- Config --------
const int   SERVO_PIN          = 9;
const int   STEP_DEG           = 5;
const int   STEP_DELAY_MS      = 100;
const int   SETTLE_MS          = 800;

// Flicker/integration settings (CZ mains)
const float    FLICKER_HZ      = 50.0;     // set to 100.0 if your lamp ripple is 100 Hz
const uint16_t WINDOW_CYCLES   = 200;      // 200 @ 50 Hz ≈ 4 s integration
// ------------------------

int  currentAngle = 0;

// ------------ Helpers ------------
static void moveServoTo(int angle) {
  if (angle < 0)  angle = 0;
  if (angle > 90) angle = 90;

  while (currentAngle != angle) {
    if (abs(currentAngle - angle) <= STEP_DEG) {
      currentAngle = angle;
    } else if (currentAngle < angle) {
      currentAngle += STEP_DEG;
    } else {
      currentAngle -= STEP_DEG;
    }
    myServo.write(currentAngle);
    delay(STEP_DELAY_MS);
  }
}

static void measureIntegrated(float &V_mean, float &I_mean_mA, float &P_mean_mW) {
  const double   window_s  = (double)WINDOW_CYCLES / (double)FLICKER_HZ;
  const uint32_t window_us = (uint32_t)(window_s * 1e6);

  uint32_t t0    = micros();
  uint32_t tPrev = t0;

  double numV_dt = 0.0, numI_dt = 0.0, numP_dt = 0.0, sum_dt = 0.0;

  while (true) {
    float shunt_mV = ina219.getShuntVoltage_mV();     // mV
    float busV     = ina219.getBusVoltage_V();        // V (VIN- to GND)
    float I_mA     = ina219.getCurrent_mA();          // mA
    float V_load   = busV + (shunt_mV / 1000.0f);     // V (VIN+ to GND)
    float P_mW     = V_load * I_mA;                   // mW (V * mA)

    uint32_t tNow  = micros();
    uint32_t dt_us = tNow - tPrev;
    tPrev = tNow;

    numV_dt += (double)V_load * (double)dt_us;
    numI_dt += (double)I_mA   * (double)dt_us;
    numP_dt += (double)P_mW   * (double)dt_us;
    sum_dt  += (double)dt_us;

    if ((tNow - t0) >= window_us) break;
  }

  if (sum_dt <= 0.0) {
    V_mean = 0; I_mean_mA = 0; P_mean_mW = 0;
  } else {
    const double inv = 1.0 / sum_dt;
    V_mean     = (float)(numV_dt * inv);
    I_mean_mA  = (float)(numI_dt * inv);
    P_mean_mW  = (float)(numP_dt * inv);
  }
}

static void printVIP(float V, float I_mA, float P_mW, int angle) {
  // angle,Voltage[V],Current[mA],Power[mW]
  Serial.print(angle); Serial.print(",");
  Serial.print(V,   6); Serial.print(",");
  Serial.print(I_mA,6); Serial.print(",");
  Serial.println(P_mW,6);
}

static void randomizeAngles(int *arr, int n) {
  // Fisher–Yates shuffle
  for (int i = n - 1; i > 0; --i) {
    int j = random(i + 1);
    int tmp = arr[i]; arr[i] = arr[j]; arr[j] = tmp;
  }
}

// ------------ Setup/Loop ------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  myServo.attach(SERVO_PIN);
  myServo.write(0);
  currentAngle = 0;
  delay(500);

  if (!ina219.begin()) {
    while (true) { delay(1000); }  // fail silently to keep output clean
  }
  ina219.setCalibration_16V_400mA();

  // Seed RNG for random sweep order
  randomSeed(analogRead(A0) + micros());

  // Optional header (comment out if you want only numbers)
  // Serial.println("angle,Voltage,Current,Power");
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');  // requires monitor_eol = LF
    s.trim();
    if (!s.length()) return;

    int cmd = s.toInt();

    // --- Sweep mode: enter 100 ---
    if (cmd == 100) {
      // Build angle list 0..90 step 5 (19 entries)
      const int N = 19;
      int angles[N];
      for (int i = 0; i < N; ++i) angles[i] = i * 5;
      randomizeAngles(angles, N);

      for (int k = 0; k < N; ++k) {
        int a = angles[k];
        moveServoTo(a);
        delay(SETTLE_MS);

        float V, I_mA, P_mW;
        measureIntegrated(V, I_mA, P_mW);
        printVIP(V, I_mA, P_mW, a);
      }
      Serial.println("DONE");
      return;
    }

    // --- Single-angle mode: 0..90 ---
    if (cmd >= 0 && cmd <= 90) {
      moveServoTo(cmd);
      delay(SETTLE_MS);

      float V, I_mA, P_mW;
      measureIntegrated(V, I_mA, P_mW);
      printVIP(V, I_mA, P_mW, cmd);
    }
  }
}
