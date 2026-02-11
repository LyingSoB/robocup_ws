/*********************************************************************
 * FINAL MANIPULATOR FIRMWARE — PHASE C (COMMAND + SAFETY)
 * Board  : Arduino Giga R1
 * J1     : Step counting + switch
 * J2–J4  : Potentiometer + switch
 * Control: Serial commands from Jetson
 *********************************************************************/

// ===================== PIN DEFINITIONS =====================

// -------- J1 BASE --------
#define J1_STEP_PIN    38
#define J1_DIR_PIN     39
#define J1_EN_PIN      46
#define J1_SWITCH_PIN  22

// -------- J2 --------
#define J2_STEP_PIN    40
#define J2_DIR_PIN     41
#define J2_EN_PIN      46
#define J2_SWITCH_PIN  48
#define J2_POT_PIN     A0

// -------- J3 --------
#define J3_STEP_PIN    42
#define J3_DIR_PIN     43
#define J3_EN_PIN      46
#define J3_SWITCH_PIN  49
#define J3_POT_PIN     A1

// -------- J4 --------
#define J4_STEP_PIN    44
#define J4_DIR_PIN     45
#define J4_EN_PIN      46
#define J4_SWITCH_PIN  50
#define J4_POT_PIN     A2

// ===================== CONSTANTS =====================

#define MOTOR_STEPS     200.0
#define MICROSTEP       8.0
#define BASE_GEARBOX    12.0

#define J1_SPUR_RATIO  (50.0 / 40.0)
#define J1_TOTAL_RATIO (BASE_GEARBOX * J1_SPUR_RATIO)

#define J1_STEPS_PER_DEG ((MOTOR_STEPS * MICROSTEP * J1_TOTAL_RATIO) / 360.0)
#define J1_MAX_DEG     180.0
#define J1_MAX_STEPS  (long)(J1_MAX_DEG * J1_STEPS_PER_DEG)

#define STEP_DELAY_US 800
#define BACKOFF_STEPS 200

// ===================== GLOBAL STATE =====================

long j1_step_count = 0;
int j2_home_adc = 0;
int j3_home_adc = 0;
int j4_home_adc = 0;

bool emergency_stop = false;

// ===================== LOW LEVEL =====================

void step_motor(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

// ===================== HOMING =====================

void home_j1() {
  digitalWrite(J1_DIR_PIN, LOW);
  while (digitalRead(J1_SWITCH_PIN) == HIGH) {
    step_motor(J1_STEP_PIN);
  }
  digitalWrite(J1_DIR_PIN, HIGH);
  for (int i = 0; i < BACKOFF_STEPS; i++) step_motor(J1_STEP_PIN);
  j1_step_count = 0;
}

int home_joint(int stepPin, int dirPin, int switchPin, int potPin) {
  digitalWrite(dirPin, LOW);
  while (digitalRead(switchPin) == HIGH) step_motor(stepPin);
  delay(200);
  return analogRead(potPin);
}

void home_all() {
  Serial.println("HOMING START");
  home_j1();
  j2_home_adc = home_joint(J2_STEP_PIN, J2_DIR_PIN, J2_SWITCH_PIN, J2_POT_PIN);
  j3_home_adc = home_joint(J3_STEP_PIN, J3_DIR_PIN, J3_SWITCH_PIN, J3_POT_PIN);
  j4_home_adc = home_joint(J4_STEP_PIN, J4_DIR_PIN, J4_SWITCH_PIN, J4_POT_PIN);
  Serial.println("HOMING DONE");
}

// ===================== MOTION =====================

void move_j1_to_deg(float target_deg) {
  if (target_deg < 0 || target_deg > J1_MAX_DEG) return;

  long target_steps = (long)(target_deg * J1_STEPS_PER_DEG);
  long delta = target_steps - j1_step_count;

  digitalWrite(J1_DIR_PIN, (delta > 0) ? HIGH : LOW);

  for (long i = 0; i < abs(delta); i++) {
    if (emergency_stop) return;
    if (j1_step_count <= 0 && delta < 0) break;
    if (j1_step_count >= J1_MAX_STEPS && delta > 0) break;
    step_motor(J1_STEP_PIN);
    j1_step_count += (delta > 0) ? 1 : -1;
  }
}

// ===================== SERIAL COMMANDS =====================

void handle_command(String cmd) {
  cmd.trim();

  if (cmd == "HOME") {
    home_all();
  }
  else if (cmd == "STOP") {
    emergency_stop = true;
    Serial.println("EMERGENCY STOP");
  }
  else if (cmd.startsWith("SET J1")) {
    emergency_stop = false;
    float deg = cmd.substring(6).toFloat();
    move_j1_to_deg(deg);
  }
}

// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(J1_STEP_PIN, OUTPUT);
  pinMode(J1_DIR_PIN, OUTPUT);
  pinMode(J1_EN_PIN, OUTPUT);
  pinMode(J1_SWITCH_PIN, INPUT_PULLUP);

  pinMode(J2_STEP_PIN, OUTPUT);
  pinMode(J2_DIR_PIN, OUTPUT);
  pinMode(J2_EN_PIN, OUTPUT);
  pinMode(J2_SWITCH_PIN, INPUT_PULLUP);

  pinMode(J3_STEP_PIN, OUTPUT);
  pinMode(J3_DIR_PIN, OUTPUT);
  pinMode(J3_EN_PIN, OUTPUT);
  pinMode(J3_SWITCH_PIN, INPUT_PULLUP);

  pinMode(J4_STEP_PIN, OUTPUT);
  pinMode(J4_DIR_PIN, OUTPUT);
  pinMode(J4_EN_PIN, OUTPUT);
  pinMode(J4_SWITCH_PIN, INPUT_PULLUP);

  digitalWrite(J1_EN_PIN, LOW);
  digitalWrite(J2_EN_PIN, LOW);
  digitalWrite(J3_EN_PIN, LOW);
  digitalWrite(J4_EN_PIN, LOW);

  home_all();
}

// ===================== LOOP =====================

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handle_command(cmd);
  }

  float j1_deg = j1_step_count / J1_STEPS_PER_DEG;
  int j2_adc = analogRead(J2_POT_PIN) - j2_home_adc;
  int j3_adc = analogRead(J3_POT_PIN) - j3_home_adc;
  int j4_adc = analogRead(J4_POT_PIN) - j4_home_adc;

  Serial.print("J1(deg): "); Serial.print(j1_deg, 2);
  Serial.print(" | J2(adc): "); Serial.print(j2_adc);
  Serial.print(" | J3(adc): "); Serial.print(j3_adc);
  Serial.print(" | J4(adc): "); Serial.println(j4_adc);

  delay(100);
}
