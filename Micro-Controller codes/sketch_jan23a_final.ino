/************************************************************
 * Arduino GIGA R1
 * 4-MOTOR + 4-ENCODER CONTROL
 * DIR-safe deceleration (NO reverse twitch)
 *
 * Serial IN : fl,fr,rl,rr\n
 * Serial OUT: ENC,fl,fr,rl,rr
 ************************************************************/

#include <math.h>

// ======================= ENCODERS ========================
#define ENC_FL_A 30
#define ENC_FL_B 31
#define ENC_FR_A 32
#define ENC_FR_B 33
#define ENC_RL_A 34
#define ENC_RL_B 35
#define ENC_RR_A 36
#define ENC_RR_B 37

volatile long ticks[4] = {0, 0, 0, 0};

unsigned long last_enc_pub = 0;
const unsigned long ENC_PUB_MS = 20;

// ======================= MOTORS ==========================
#define ENABLE_PIN 10   // LOW = enabled

const int STEP_PINS[4] = {2, 4, 6, 8};
const int DIR_PINS[4]  = {3, 5, 7, 9};

// Direction correction per wheel
// Order: FL, FR, RL, RR
const int DIR_SIGN[4] = {
  +1,   // FL
  +1,   // FR
  +1,   // RL
  +1    // RR
};

float target_rate[4]  = {0, 0, 0, 0};   // steps/sec (always >= 0)
float current_rate[4] = {0, 0, 0, 0};   // steps/sec (always >= 0)
int   target_dir[4]   = {HIGH, HIGH, HIGH, HIGH};

unsigned long last_step_us[4] = {0, 0, 0, 0};
unsigned long last_update_us = 0;

// ================= MOTOR PARAMETERS ======================
const float WHEEL_RADIUS = 0.05;
const float GEAR_RATIO   = 5.0;
const int MOTOR_STEPS    = 200;
const int MICROSTEPS     = 8;
const int STEPS_PER_REV  = MOTOR_STEPS * MICROSTEPS;

// Operational limits
const float MAX_STEP_RATE   = 10000.0;
const float ACCEL_STEP_RATE = 1000.0;
const float DECEL_STEP_RATE = 2000.0;

// ==================== ENCODER ISRs ======================
void ISR_FL_A() { digitalRead(ENC_FL_B) ? ticks[0]-- : ticks[0]++; }
void ISR_FR_A() { digitalRead(ENC_FR_B) ? ticks[1]-- : ticks[1]++; }
void ISR_RL_A() { digitalRead(ENC_RL_B) ? ticks[2]-- : ticks[2]++; }
void ISR_RR_A() { digitalRead(ENC_RR_B) ? ticks[3]-- : ticks[3]++; }

// ================= SPEED → STEP RATE ====================
float speedToStepRate(float v) {
  if (fabs(v) < 0.001) return 0.0;

  float wheel_rpm = (fabs(v) / (2.0 * PI * WHEEL_RADIUS)) * 60.0;
  float motor_rpm = wheel_rpm * GEAR_RATIO;
  float rate = (motor_rpm * STEPS_PER_REV) / 60.0;

  return constrain(rate, 0.0, MAX_STEP_RATE);
}

// ======================= SETUP ==========================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Encoders
  pinMode(ENC_FL_A, INPUT_PULLUP); pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(ENC_FR_A, INPUT_PULLUP); pinMode(ENC_FR_B, INPUT_PULLUP);
  pinMode(ENC_RL_A, INPUT_PULLUP); pinMode(ENC_RL_B, INPUT_PULLUP);
  pinMode(ENC_RR_A, INPUT_PULLUP); pinMode(ENC_RR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), ISR_FL_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), ISR_FR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_A), ISR_RL_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), ISR_RR_A, RISING);

  // Motors
  for (int i = 0; i < 4; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], HIGH);
  }

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);   // enable motors

  last_update_us = micros();
}

// ================= ENCODER OUTPUT ======================
void publishEncoders() {
  if (millis() - last_enc_pub < ENC_PUB_MS) return;
  last_enc_pub = millis();

  noInterrupts();
  long e[4];
  for (int i = 0; i < 4; i++) {
    e[i] = ticks[i];
    ticks[i] = 0;
  }
  interrupts();

  Serial.print("ENC,");
  Serial.print(e[0]); Serial.print(",");
  Serial.print(e[1]); Serial.print(",");
  Serial.print(e[2]); Serial.print(",");
  Serial.println(e[3]);
}

// ======================== LOOP =========================
void loop() {

  // -------- READ ROS COMMAND ----------
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');

    float v[4] = {0, 0, 0, 0};
    sscanf(line.c_str(), "%f,%f,%f,%f",
           &v[0], &v[1], &v[2], &v[3]);

    for (int i = 0; i < 4; i++) {
      float signed_v = v[i] * DIR_SIGN[i];

      target_dir[i] = (signed_v >= 0) ? HIGH : LOW;
      target_rate[i] = speedToStepRate(signed_v);
    }
  }

  unsigned long now = micros();
  float dt = (now - last_update_us) * 1e-6;
  last_update_us = now;

  // -------- ACCEL / DECEL ----------
  for (int i = 0; i < 4; i++) {

    if (current_rate[i] < target_rate[i]) {
      current_rate[i] += ACCEL_STEP_RATE * dt;
      if (current_rate[i] > target_rate[i])
        current_rate[i] = target_rate[i];
    }
    else if (current_rate[i] > target_rate[i]) {
      current_rate[i] -= DECEL_STEP_RATE * dt;
      if (current_rate[i] < target_rate[i])
        current_rate[i] = target_rate[i];
    }

    // 🔑 DIR changes ONLY when fully stopped
    if (current_rate[i] < 1.0) {
      digitalWrite(DIR_PINS[i], target_dir[i]);
    }
  }

  // -------- STEP GENERATION ----------
  for (int i = 0; i < 4; i++) {
    if (current_rate[i] >= 1.0) {
      unsigned long interval = 1000000.0 / current_rate[i];
      if (now - last_step_us[i] >= interval) {
        last_step_us[i] = now;
        digitalWrite(STEP_PINS[i], HIGH);
        delayMicroseconds(3);
        digitalWrite(STEP_PINS[i], LOW);
      }
    }
  }

  publishEncoders();
}
