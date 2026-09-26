#include <cassert>
#include "../../robot/firmware/hansel_nano_bridge/hansel_nano_bridge.ino"

void command(const char *text) {
  char line[160]; strcpy(line, text); handleLine(line);
}
void advance(int ms) {
  for (int i=0; i<ms; ++i) { ++clockMs; loop(); }
}
int main() {
  setup();
  command("M,100,-100,1");
  assert(duties[PWM_LEFT] == 191 && duties[PWM_RIGHT] == 191);
  command("H,90");
  assert(currentHeadAngle == 0 && clockMs == 0);
  advance(20);
  assert(currentHeadAngle > 0 && currentHeadAngle < 1);
  advance(1800);
  assert(fabs(currentHeadAngle - 90) < .001 && !headMotion.moving());
  assert(headServo.attaches == 1 && headServo.attached());
  assert(lastTelemetryMs >= 1800 && duties[PWM_LEFT] == 0);
  command("H,95"); advance(50); command("H,100");
  advance(1000);
  assert(headServo.attaches == 1 && fabs(currentHeadAngle-100) < .001);
  command("H,0"); advance(2000);
  assert(fabs(currentHeadAngle) < .001);

  command("D,350,1515,800,23");
  assert(detachPhase == 1 && detachServo.pulse == 1515);
  command("M,75,75,1");
  assert(duties[PWM_LEFT] == 0);
  command("D,350,1515,800,24");
  assert(Serial.output.str().find("D,24,busy") != std::string::npos);
  advance(349);
  assert(detachPhase == 1 && detachServo.pulse == 1515);
  advance(1);
  assert(detachPhase == 2 && detachServo.pulse == 800);
  advance(119); assert(detachServo.attached());
  advance(1);
  assert(detachPhase == 0 && !detachServo.attached());
  assert(Serial.output.str().find("D,23,done") != std::string::npos);
  command("M,75,0,1");
  assert(duties[PWM_LEFT] == 191 && duties[PWM_RIGHT] == 0);
  command("F,1,20");
  advance(100);
  assert(duties[PWM_LEFT] == 51 && duties[PWM_RIGHT] == 51);
  command("F,0,0");
  assert(duties[PWM_LEFT] == 0 && duties[PWM_RIGHT] == 0);
  leftCount = (uint32_t)-7; rightCount = 9;
  advance(50);
  assert(Serial.output.str().find(",-7,9,") != std::string::npos);
  assert(Serial.output.str().find("B,boot") != std::string::npos);

  ServoMotion trajectory;
  trajectory.target = 180;
  for (int i=0; i<60; ++i) trajectory.step(.02);
  trajectory.target = -180;
  for (int i=0; i<600; ++i) {
    trajectory.step(.02);
    assert(fabs(trajectory.velocity) <= ServoMotion::SPEED + .001);
    assert(trajectory.position <= 180.001 && trajectory.position >= -180.001);
  }
  assert(fabs(trajectory.position + 180) < .001 && !trajectory.moving());
}
