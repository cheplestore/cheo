// =====================================================================
//  Cheo biped: joint limits (from the leg design + collision test, leg v1)
//  Angles are in degrees from the STRAIGHT-LEG pose (standing straight = 0).
//
//  Positive directions:
//    hip roll    + = leg swings OUT to the side      - = leg swings in (toward the other leg)
//    hip pitch   + = leg swings FORWARD              - = leg swings back
//    knee        + = knee BENDS (shin goes back)     (never below 0: no bending backward)
//    ankle pitch + = toes UP                         - = toes down
//    ankle roll  + = foot tilts so its OUTER edge goes up
//
//  "physical" = where printed parts actually hit (measured in the collision test)
//  min/max    = safe software limits (5 deg+ before any hit, and inside the servo's 0-180)
//
//  PCA9685 channels: left leg 0-4, right leg 5-9, arms 10-11, head turn 12
// =====================================================================
#pragma once

struct LegJoint {
  const char* name;
  uint8_t ch;          // PCA9685 channel
  int16_t minDeg;      // safe limits (joint angle, deg)
  int16_t maxDeg;
  int16_t servoAtZero; // servo angle (0-180) when the leg is straight - set while assembling
  int8_t  dir;         // +1 or -1: flip if the joint moves the wrong way
  int16_t trim;        // small fix (deg) found while calibrating, so straight really is straight
};

//                   name                  ch  min  max  zero dir trim       physical limit (where parts hit)
LegJoint LEG[10] = {
  { "L hip roll",     0, -15,  45,   90, +1, 0 },   // in: other leg at ~3 deg, out: 79
  { "L hip pitch",    1, -80,  80,   90, +1, 0 },   // +-107 (servo stops at +-90)
  { "L knee",         2,   0, 115,   45, +1, 0 },   // 121 (assemble with servo at 90 = knee bent 45)
  { "L ankle pitch",  3, -80,  80,   90, +1, 0 },   // +-106
  { "L ankle roll",   4, -45,  45,   90, +1, 0 },   // +-91
  { "R hip roll",     5, -15,  45,   90, -1, 0 },   // right leg is a mirror image: roll dirs flip
  { "R hip pitch",    6, -80,  80,   90, -1, 0 },
  { "R knee",         7,   0, 115,   45, -1, 0 },
  { "R ankle pitch",  8, -80,  80,   90, -1, 0 },
  { "R ankle roll",   9, -45,  45,   90, -1, 0 },
};

// Clamp a wanted joint angle to the safe range
inline float clampJoint(int j, float deg) {
  if (deg < LEG[j].minDeg) return LEG[j].minDeg;
  if (deg > LEG[j].maxDeg) return LEG[j].maxDeg;
  return deg;
}

// Joint angle -> servo angle (0-180), always inside the safe range
inline float servoAngle(int j, float jointDeg) {
  float s = LEG[j].servoAtZero + LEG[j].dir * (clampJoint(j, jointDeg) + LEG[j].trim);
  if (s < 0) s = 0;
  if (s > 180) s = 180;
  return s;
}
