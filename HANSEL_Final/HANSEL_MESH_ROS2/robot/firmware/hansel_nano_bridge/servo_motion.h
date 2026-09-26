#ifndef HANSEL_SERVO_MOTION_H
#define HANSEL_SERVO_MOTION_H
#include <math.h>

// Logical degrees, independent of the calibrated pulse-width mapping.
struct ServoMotion {
  float position = 0.0f;
  float target = 0.0f;
  float velocity = 0.0f;
  static constexpr float SPEED = 90.0f;
  static constexpr float ACCELERATION = 360.0f;

  bool moving() const {
    return fabsf(target - position) > 0.001f || fabsf(velocity) > 0.001f;
  }

  void step(float dt) {
    if (dt <= 0.0f || !moving()) return;
    dt = fminf(dt, 0.04f);
    float error = target - position;
    float speed = fminf(SPEED, sqrtf(2.0f * ACCELERATION * fabsf(error)));
    float desired = error < 0.0f ? -speed : speed;
    float change = ACCELERATION * dt;
    velocity += fmaxf(-change, fminf(change, desired - velocity));
    float delta = velocity * dt;
    if (error * delta >= 0.0f && fabsf(delta) >= fabsf(error)) {
      position = target;
      velocity = 0.0f;
    } else {
      position += delta;
    }
  }
};
#endif
