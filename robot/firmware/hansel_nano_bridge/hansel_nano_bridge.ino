/* HANSEL Arduino Nano hardware bridge.
 *
 * Pin map from 캡스톤_배선초안_0729.xlsx:
 * D5  shared left PWM (front + rear drivers)
 * D6  shared right PWM (front + rear drivers)
 * D7/D8 front-left direction, D11/D12 front-right direction
 * D13/A0 rear-left direction, A1/A2 rear-right direction
 * D2/D4 rear-left encoder A/B, D3/A3 rear-right encoder A/B
 * D9  150 kg head servo, D10 SG90 detach servo
 * STBY is hardwired to 5 V.
 */

#include <Servo.h>
#include <string.h>
#include <stdlib.h>

const uint8_t PWM_LEFT = 5;
const uint8_t PWM_RIGHT = 6;
const uint8_t FRONT_L_IN1 = 7;
const uint8_t FRONT_L_IN2 = 8;
const uint8_t FRONT_R_IN1 = 11;
const uint8_t FRONT_R_IN2 = 12;
const uint8_t REAR_L_IN1 = 13;
const uint8_t REAR_L_IN2 = A0;
const uint8_t REAR_R_IN1 = A1;
const uint8_t REAR_R_IN2 = A2;
const uint8_t ENC_L_A = 2;
const uint8_t ENC_L_B = 4;
const uint8_t ENC_R_A = 3;
const uint8_t ENC_R_B = A3;
const uint8_t HEAD_SERVO_PIN = 9;
const uint8_t DETACH_SERVO_PIN = 10;

// The wiring note explicitly says one side must be inverted in software.
bool rearLeftReverse = false;
bool rearRightReverse = true;
bool frontLeftReverse = false;
bool frontRightReverse = true;

volatile long leftCount = 0;
volatile long rightCount = 0;
long lastLeftCount = 0;
long lastRightCount = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastMotorCommandMs = 0;
const unsigned long COMMAND_WATCHDOG_MS = 500;
const unsigned long TELEMETRY_INTERVAL_MS = 50;

float currentHeadAngle = 0.0f;
float servoMinAngle = -180.0f;
float servoCenterAngle = 0.0f;
float servoMaxAngle = 180.0f;
int servoMinPulseUs = 500;
int servoCenterPulseUs = 1500;
int servoMaxPulseUs = 2500;
bool headServoHold = false;

Servo headServo;
Servo detachServo;
char lineBuffer[160];
uint8_t lineLength = 0;

void encoderLeftISR() {
  // A-channel interrupt, B-channel direction decoding as required by the sheet.
  leftCount += (digitalRead(ENC_L_B) == HIGH) ? 1 : -1;
}
void encoderRightISR() {
  rightCount += (digitalRead(ENC_R_B) == HIGH) ? 1 : -1;
}

void setHBridge(uint8_t pin1, uint8_t pin2, int direction, bool reverse) {
  if (reverse) direction = -direction;
  if (direction > 0) {
    digitalWrite(pin1, HIGH); digitalWrite(pin2, LOW);
  } else if (direction < 0) {
    digitalWrite(pin1, LOW); digitalWrite(pin2, HIGH);
  } else {
    digitalWrite(pin1, LOW); digitalWrite(pin2, LOW);
  }
}

int signedDirection(float pwm) {
  return (pwm > 0.01f) ? 1 : ((pwm < -0.01f) ? -1 : 0);
}
uint8_t pwmDuty(float pwm) {
  float value = fabs(pwm);
  if (value > 100.0f) value = 100.0f;
  return (uint8_t)(value * 2.55f + 0.5f);
}

void stopAllMotors() {
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
  setHBridge(FRONT_L_IN1, FRONT_L_IN2, 0, frontLeftReverse);
  setHBridge(FRONT_R_IN1, FRONT_R_IN2, 0, frontRightReverse);
  setHBridge(REAR_L_IN1, REAR_L_IN2, 0, rearLeftReverse);
  setHBridge(REAR_R_IN1, REAR_R_IN2, 0, rearRightReverse);
}

void applyDrive(float leftPwm, float rightPwm, bool frontFollow) {
  int leftDir = signedDirection(leftPwm);
  int rightDir = signedDirection(rightPwm);
  setHBridge(REAR_L_IN1, REAR_L_IN2, leftDir, rearLeftReverse);
  setHBridge(REAR_R_IN1, REAR_R_IN2, rightDir, rearRightReverse);
  setHBridge(FRONT_L_IN1, FRONT_L_IN2, frontFollow ? leftDir : 0, frontLeftReverse);
  setHBridge(FRONT_R_IN1, FRONT_R_IN2, frontFollow ? rightDir : 0, frontRightReverse);
  analogWrite(PWM_LEFT, pwmDuty(leftPwm));
  analogWrite(PWM_RIGHT, pwmDuty(rightPwm));
  lastMotorCommandMs = millis();
}

void applyFrontOnly(int direction, float pwmPercent) {
  // D5/D6 are shared, so isolate the rear directions before using the PWM for
  // a front-only command. A following normal M command restores normal drive.
  setHBridge(REAR_L_IN1, REAR_L_IN2, 0, rearLeftReverse);
  setHBridge(REAR_R_IN1, REAR_R_IN2, 0, rearRightReverse);
  setHBridge(FRONT_L_IN1, FRONT_L_IN2, direction, frontLeftReverse);
  setHBridge(FRONT_R_IN1, FRONT_R_IN2, direction, frontRightReverse);
  analogWrite(PWM_LEFT, pwmDuty(pwmPercent));
  analogWrite(PWM_RIGHT, pwmDuty(pwmPercent));
  lastMotorCommandMs = millis();
}

int piecewisePulse(float angle) {
  angle = constrain(angle, servoMinAngle, servoMaxAngle);
  if (angle <= servoCenterAngle) {
    float denominator = servoCenterAngle - servoMinAngle;
    float ratio = denominator == 0.0f ? 0.0f : (angle - servoMinAngle) / denominator;
    return (int)(servoMinPulseUs + ratio * (servoCenterPulseUs - servoMinPulseUs));
  }
  float denominator = servoMaxAngle - servoCenterAngle;
  float ratio = denominator == 0.0f ? 0.0f : (angle - servoCenterAngle) / denominator;
  return (int)(servoCenterPulseUs + ratio * (servoMaxPulseUs - servoCenterPulseUs));
}

void applyHeadAngle(float angle) {
  currentHeadAngle = constrain(angle, servoMinAngle, servoMaxAngle);
  headServo.attach(HEAD_SERVO_PIN, 400, 2600);
  headServo.writeMicroseconds(piecewisePulse(currentHeadAngle));
  delay(80);
  if (!headServoHold) headServo.detach();
}

char *nextToken(char **context) { return strtok_r(NULL, ",", context); }

void handleLine(char *line) {
  char *context = NULL;
  char *command = strtok_r(line, ",", &context);
  if (!command) return;
  if (strcmp(command, "M") == 0) {
    char *l = nextToken(&context), *r = nextToken(&context), *f = nextToken(&context);
    if (!l || !r || !f) { Serial.println("E,bad_motor"); return; }
    applyDrive(atof(l), atof(r), atoi(f) != 0);
  } else if (strcmp(command, "F") == 0) {
    char *m = nextToken(&context), *p = nextToken(&context);
    if (!m || !p) { Serial.println("E,bad_front"); return; }
    applyFrontOnly(atoi(m), atof(p));
  } else if (strcmp(command, "H") == 0) {
    char *a = nextToken(&context);
    if (!a) { Serial.println("E,bad_head"); return; }
    applyHeadAngle(atof(a));
  } else if (strcmp(command, "S") == 0) {
    char *v[7];
    for (uint8_t i = 0; i < 7; ++i) v[i] = nextToken(&context);
    for (uint8_t i = 0; i < 7; ++i) if (!v[i]) { Serial.println("E,bad_servo_config"); return; }
    float minA = atof(v[0]), centerA = atof(v[1]), maxA = atof(v[2]);
    if (!(minA >= -180.0f && minA < centerA && centerA < maxA && maxA <= 180.0f)) {
      Serial.println("E,bad_servo_range"); return;
    }
    servoMinAngle = minA; servoCenterAngle = centerA; servoMaxAngle = maxA;
    servoMinPulseUs = atoi(v[3]); servoCenterPulseUs = atoi(v[4]); servoMaxPulseUs = atoi(v[5]);
    headServoHold = atoi(v[6]) != 0;
    currentHeadAngle = constrain(currentHeadAngle, servoMinAngle, servoMaxAngle);
  } else if (strcmp(command, "D") == 0) {
    char *duration = nextToken(&context), *press = nextToken(&context), *rest = nextToken(&context);
    if (!duration || !press || !rest) { Serial.println("E,bad_detach"); return; }
    detachServo.attach(DETACH_SERVO_PIN, 500, 2500);
    detachServo.writeMicroseconds(atoi(press));
    delay(constrain(atoi(duration), 50, 2000));
    detachServo.writeMicroseconds(atoi(rest));
    delay(120);
    detachServo.detach();
  } else if (strcmp(command, "X") == 0) {
    stopAllMotors();
  } else if (strcmp(command, "Q") == 0) {
    Serial.println("E,ready");
  } else {
    Serial.println("E,unknown_command");
  }
}

void publishTelemetry() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastTelemetryMs;
  if (elapsed < TELEMETRY_INTERVAL_MS) return;
  noInterrupts();
  long l = leftCount, r = rightCount;
  interrupts();
  float seconds = elapsed / 1000.0f;
  float leftCps = (l - lastLeftCount) / seconds;
  float rightCps = (r - lastRightCount) / seconds;
  lastLeftCount = l; lastRightCount = r; lastTelemetryMs = now;
  Serial.print("T,"); Serial.print(leftCps, 3); Serial.print(',');
  Serial.print(rightCps, 3); Serial.print(','); Serial.println(currentHeadAngle, 3);
}

void setup() {
  Serial.begin(115200);
  pinMode(PWM_LEFT, OUTPUT); pinMode(PWM_RIGHT, OUTPUT);
  uint8_t outputs[] = {FRONT_L_IN1, FRONT_L_IN2, FRONT_R_IN1, FRONT_R_IN2,
                       REAR_L_IN1, REAR_L_IN2, REAR_R_IN1, REAR_R_IN2};
  for (uint8_t i = 0; i < sizeof(outputs); ++i) pinMode(outputs[i], OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encoderRightISR, RISING);
  stopAllMotors();
  lastTelemetryMs = lastMotorCommandMs = millis();
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineLength > 0) {
        lineBuffer[lineLength] = '\0';
        handleLine(lineBuffer);
        lineLength = 0;
      }
    } else if (lineLength < sizeof(lineBuffer) - 1) {
      lineBuffer[lineLength++] = ch;
    } else {
      lineLength = 0;
      Serial.println("E,line_too_long");
    }
  }
  if (millis() - lastMotorCommandMs > COMMAND_WATCHDOG_MS) stopAllMotors();
  publishTelemetry();
}
