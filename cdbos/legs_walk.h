// =====================================================================
//  legs_walk.h - Cheo biped legs for CDBos (v17)
//  Reuses the firmware's PCA9685 code (pcaUs, pcaFound), so it shares the
//  chip with the arms. Legs = ch 0-9, arms = ch 10-11 (armsLoop owns those).
//  The neck stays on SERVO_PIN. Only active on the S3 (HAS_ARMS).
//  Needs leg_limits.h in this same folder (copy of leg/leg_limits.h).
// =====================================================================
#pragma once
#include "leg_limits.h"

#if HAS_ARMS

// ---- walking tunables (defaults follow body/sim: 20 mm step, 15 mm lift, 1.6 s per step) ----
float legStepDeg   = 7;      // hip pitch swing each way (about 20 mm per step)
float legShiftDeg  = 8;      // hip roll weight shift (max 15; legs can only roll in ~3 deg with both feet down)
float legLiftDeg   = 35;     // extra knee bend on the swing leg
float legCrouchDeg = 30;     // knee bend while walking (hip leans 15, like the sim's crouch)
float legCycleMs   = 3200;   // one full left+right step = 2 x 1.6 s
float legSlewDegS  = 150;    // max joint speed (deg/s)

bool legsOn = true;          // false = legs limp (use while assembling)
static float legNow[10], legGoal[10], legSent[10];
static bool  legWalking = false, legLimp = false;
static float legDir = 1, legAmp = 0, legAmpGoal = 0, legPhase = 0;
static int16_t legTrimDefault[10];        // trims from leg_limits.h (the saved nudges override them)
static bool  legRawHold[10];             // true = joint is being held at a raw servo angle (calibration)

static int legUs(float servoDeg) {
  return 500 + (int)(constrain(servoDeg, 0.0f, 180.0f) / 180.0f * 1900.0f);   // same range as the arms
}

bool legsAvailable() { return pcaFound; }

static void legsLoadTrims() {            // trims saved from the web page replace the ones in leg_limits.h
  for (int j = 0; j < 10; j++) {
    legTrimDefault[j] = LEG[j].trim;
    String k = "lt" + String(j);
    if (prefs.isKey(k.c_str())) LEG[j].trim = prefs.getShort(k.c_str(), LEG[j].trim);
  }
}

void legsBegin() {                       // call AFTER armsBegin()
  legsLoadTrims();
  legsOn = prefs.getUChar("legson", 1);
  for (int j = 0; j < 10; j++) { legNow[j] = legGoal[j] = 0; legSent[j] = -999; }
  if (!pcaFound) return;
  Serial.println("Legs: PCA9685 found, channels 0-9");
  if (!legsOn) return;                                        // saved as limp: leave the servos alone
  for (int j = 0; j < 10; j++) {                              // start giving signal right away, one servo at a time
    pcaUs(LEG[j].ch, legUs(servoAngle(j, 0)));                // (so they don't all pull current at once and wander)
    legSent[j] = 0;
    delay(120);
  }
}

static void legsReleaseRaw() {           // go back to normal control for every joint
  for (int j = 0; j < 10; j++) if (legRawHold[j]) { legRawHold[j] = false; legSent[j] = -999; }
}

void legsSetOn(bool on) { legsOn = on; prefs.putUChar("legson", on ? 1 : 0); }

void legsSetJoint(int j, float deg) {    // one joint, for testing / calibrating
  if (j < 0 || j > 9) return;
  legWalking = false; legAmp = legAmpGoal = 0;
  if (legRawHold[j]) { legRawHold[j] = false; legSent[j] = -999; }
  legGoal[j] = clampJoint(j, deg);
}

// Nudge a joint's trim by d degrees (saved right away). d = 0 just reports it.
String legsNudge(int j, int d) {
  if (j < 0 || j > 9) return "";
  int t = constrain((int)LEG[j].trim + d, -90, 90);
  LEG[j].trim = t;
  prefs.putShort(("lt" + String(j)).c_str(), (int16_t)t);
  if (legRawHold[j]) legRawHold[j] = false;
  legSent[j] = -999;                                          // write the joint again with the new trim
  return String(LEG[j].name) + ": trim = " + String(t);
}

void legsTrimReset() {                   // forget the nudges, go back to leg_limits.h
  for (int j = 0; j < 10; j++) {
    prefs.remove(("lt" + String(j)).c_str());
    LEG[j].trim = legTrimDefault[j];
    legSent[j] = -999;
  }
}

// Calibration: put a joint at a raw servo angle (0-180), ignoring the limits and trim.
// Returns the trim to type into leg_limits.h if the joint looks straight at this angle.
String legsRaw(int j, float ang) {
  if (j < 0 || j > 9) return "";
  ang = constrain(ang, 0.0f, 180.0f);
  legWalking = false; legAmp = legAmpGoal = 0;
  legRawHold[j] = true;
  pcaUs(LEG[j].ch, legUs(ang));
  int trim = LEG[j].dir * ((int)(ang + 0.5f) - LEG[j].servoAtZero);
  return String(LEG[j].name) + ": servo " + String((int)(ang + 0.5f)) + " -> if straight here, trim = " + String(trim);
}

static void legsSetLeg(int leg, float hipRoll, float hipPitch, float knee, float anklePitch, float ankleRoll) {
  int b = leg * 5;
  legGoal[b + 0] = clampJoint(b + 0, hipRoll);
  legGoal[b + 1] = clampJoint(b + 1, hipPitch);
  legGoal[b + 2] = clampJoint(b + 2, knee);
  legGoal[b + 3] = clampJoint(b + 3, anklePitch);
  legGoal[b + 4] = clampJoint(b + 4, ankleRoll);
}

void legsStand()  { legsReleaseRaw(); legWalking = false; legAmp = legAmpGoal = 0; for (int l = 0; l < 2; l++) legsSetLeg(l, 0, 0, 0, 0, 0); }
void legsCrouch() { legsReleaseRaw(); legWalking = false; legAmp = legAmpGoal = 0; for (int l = 0; l < 2; l++) legsSetLeg(l, 0, 25, 50, 25, 0); }
void legsSit()    { legsReleaseRaw(); legWalking = false; legAmp = legAmpGoal = 0; for (int l = 0; l < 2; l++) legsSetLeg(l, 0, 75, 90, 15, 0); }

// dir: +1 forward, -1 backward, 0 = ease to a stop
void legsWalk(int dir) {
  if (dir == 0) { legAmpGoal = 0; return; }       // eases out, then stands by itself
  legsReleaseRaw();
  legDir = dir > 0 ? 1 : -1;
  legAmpGoal = 1;
  legWalking = true;
}

bool legsIsWalking() { return legWalking; }

// Left leg is the STANCE leg while sin(phase) > 0, right leg while sin(phase) < 0
static void legsWalkPose() {
  float s = sinf(legPhase), c = cosf(legPhase), a = legAmp;
  float kL = a * legCrouchDeg + a * legLiftDeg * max(0.0f, -s);
  float kR = a * legCrouchDeg + a * legLiftDeg * max(0.0f,  s);
  float hb = a * legCrouchDeg * 0.5f;                         // keeps weight over the feet
  float hL = hb + a * legStepDeg * legDir *  c;
  float hR = hb + a * legStepDeg * legDir * -c;
  float rL = -a * legShiftDeg * s;                            // lean over the stance leg
  float rR =  a * legShiftDeg * s;
  legsSetLeg(0, rL, hL, kL, kL - hL, -rL);                    // feet flat: ankle = knee - hip, ankle roll = -hip roll
  legsSetLeg(1, rR, hR, kR, kR - hR, -rR);
}

void legsUpdate() {                      // call every loop()
  if (!pcaFound) return;
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 20) return;
  float dt = (now - last) / 1000.0f;
  last = now;
  if (dt > 0.2f) dt = 0.02f;

  if (!legsOn) {                                              // limp
    if (!legLimp) { for (int j = 0; j < 10; j++) { pcaUs(LEG[j].ch, 0); legSent[j] = -999; } legLimp = true; }
    return;
  }
  legLimp = false;

  if (legWalking) {
    float ramp = dt / 1.5f;                                   // ~1.5 s to start / stop
    if (legAmp < legAmpGoal) legAmp = min(legAmpGoal, legAmp + ramp);
    else if (legAmp > legAmpGoal) legAmp = max(legAmpGoal, legAmp - ramp);
    legPhase += dt / (legCycleMs / 1000.0f) * 2.0f * PI;
    if (legPhase > 2.0f * PI) legPhase -= 2.0f * PI;
    legsWalkPose();
    if (legAmp <= 0.001f && legAmpGoal == 0) legsStand();
  }

  float maxStep = legSlewDegS * dt;
  for (int j = 0; j < 10; j++) {
    if (legRawHold[j]) continue;
    legNow[j] += constrain(legGoal[j] - legNow[j], -maxStep, maxStep);
    if (fabsf(legNow[j] - legSent[j]) > 0.3f) {
      pcaUs(LEG[j].ch, legUs(servoAngle(j, legNow[j])));
      legSent[j] = legNow[j];
    }
  }
}

// voice / chat commands: returns true if handled
bool legsCommand(const String& l, String& reply, Mood& m) {
  bool walk = l == "walk" || l == "walk forward" || l == "walk forwards" || l == "start walking";
  bool back = l == "walk back" || l == "walk backward" || l == "walk backwards";
  bool stopw = l == "stop walking" || l == "stand still";
  bool stand = l == "stand" || l == "stand up";
  bool sit = l == "sit" || l == "sit down";
  bool crouch = l == "crouch";
  if (!(walk || back || stopw || stand || sit || crouch)) return false;
  if (!pcaFound) { reply = "I can't find my legs yet."; m = CONFUSED; return true; }
  if (walk) { legsWalk(1); reply = "Here I go!"; m = HAPPY; }
  else if (back) { legsWalk(-1); reply = "Backing up!"; m = SURPRISED; }
  else if (stopw) { legsWalk(0); reply = "Okay, stopping."; m = NORMAL; }
  else if (stand) { legsStand(); reply = "Standing tall!"; m = COOL; }
  else if (sit) { legsSit(); reply = "Sitting down."; m = SLEEPY; }
  else { legsCrouch(); reply = "Getting low."; m = NORMAL; }
  return true;
}

// Serial Monitor tester (set it to send a newline):
//   stand | sit | crouch | walk | back | stop | limp | hold
//   j <0-9> <deg>   move one joint, e.g.  j 2 30
void legsSerial() {
  static bool tmo = false;
  if (!tmo) { Serial.setTimeout(30); tmo = true; }
  if (!Serial.available()) return;
  String c = Serial.readStringUntil('\n');
  c.trim();
  if (c == "stand") legsStand();
  else if (c == "sit") legsSit();
  else if (c == "crouch") legsCrouch();
  else if (c == "walk") legsWalk(1);
  else if (c == "back") legsWalk(-1);
  else if (c == "stop") legsWalk(0);
  else if (c == "limp") legsSetOn(false);
  else if (c == "hold") legsSetOn(true);
  else if (c.startsWith("j ")) { int j; float d; if (sscanf(c.c_str(), "j %d %f", &j, &d) == 2) legsSetJoint(j, d); }
}

#else   // ESP32-C6: no legs, empty stubs so the rest of the code still compiles
bool legsOn = false;
bool legsAvailable() { return false; }
void legsBegin() {}
void legsUpdate() {}
void legsSerial() {}
void legsStand() {}
void legsSit() {}
void legsCrouch() {}
void legsWalk(int) {}
bool legsIsWalking() { return false; }
void legsSetOn(bool) {}
void legsSetJoint(int, float) {}
String legsRaw(int, float) { return ""; }
String legsNudge(int, int) { return ""; }
void legsTrimReset() {}
bool legsCommand(const String&, String&, Mood&) { return false; }
#endif
