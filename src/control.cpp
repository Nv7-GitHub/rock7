#include "control.h"

#include <Arduino.h>

float desiredCd = BASE_CD;

float getCoastAltitude(float velocity, float cd) {
  // Clamp inputs
  if (velocity < COAST_VEL_MIN) velocity = COAST_VEL_MIN;
  if (velocity > COAST_VEL_MAX) velocity = COAST_VEL_MAX;
  if (cd < COAST_CD_MIN) cd = COAST_CD_MIN;
  if (cd > COAST_CD_MAX) cd = COAST_CD_MAX;

  // Find lower indices
  int vi = (int)((velocity - COAST_VEL_MIN) / COAST_VEL_STEP);
  int ci = (int)((cd - COAST_CD_MIN) / COAST_CD_STEP);
  if (vi > COAST_N_VEL - 2) vi = COAST_N_VEL - 2;
  if (ci > COAST_N_CD - 2) ci = COAST_N_CD - 2;

  // Bilinear interpolation weights
  float vw =
      (velocity - (COAST_VEL_MIN + vi * COAST_VEL_STEP)) / COAST_VEL_STEP;
  float cw = (cd - (COAST_CD_MIN + ci * COAST_CD_STEP)) / COAST_CD_STEP;

  float a00 = COAST_ALTITUDE_TABLE[vi][ci];
  float a01 = COAST_ALTITUDE_TABLE[vi][ci + 1];
  float a10 = COAST_ALTITUDE_TABLE[vi + 1][ci];
  float a11 = COAST_ALTITUDE_TABLE[vi + 1][ci + 1];

  float a0 = a00 * (1.0f - vw) + a10 * vw;
  float a1 = a01 * (1.0f - vw) + a11 * vw;
  return a0 * (1.0f - cw) + a1 * cw;
}

void controlUpdate() {
  // Controller state persists across calls and is reset outside coast phase.
  static bool controllerActive = false;
  static uint32_t lastUpdateUs = 0;
  static float posIntegral = MOTOR_MIN;

  // Stay closed during any residual positive accel (late motor burn)
  if (aGlob[2] >= 0.0f) {
    desiredCd = Cd;
    controllerActive = false;
    posIntegral = MOTOR_MIN;
    lastUpdateUs = micros();
    odrvPosition(MOTOR_MIN);
    return;
  }

  uint32_t nowUs = micros();
  float dt = CD_CTRL_DT_NOMINAL;
  if (controllerActive) {
    uint32_t dtUs = nowUs - lastUpdateUs;
    if (dtUs > 0 && dtUs < 1000000UL) {
      dt = 1e-6f * (float)dtUs;
    }
  }
  lastUpdateUs = nowUs;

  if (!controllerActive) {
    controllerActive = true;
    posIntegral = motorpos;
  }

  if (dt < 0.0001f || dt > 0.05f) {
    dt = CD_CTRL_DT_NOMINAL;
  }

  // Search full Cd range in COAST_N_CD steps for the Cd that best hits
  // TARGET_ALTITUDE
  float bestCd = Cd;
  float minError = 1e10f;

  for (int i = 0; i < COAST_N_CD; i++) {
    float cdTest = COAST_CD_MIN + COAST_CD_STEP * (float)i;
    float predictedApogee = x[0] + getCoastAltitude(x[1], cdTest);
    float err = fabsf(predictedApogee - TARGET_ALTITUDE);
    if (err < minError) {
      minError = err;
      bestCd = cdTest;
    }
  }

  desiredCd = bestCd;

  // Closed-loop Cd tracking:
  // If Cd is too low, error is positive and controller opens airbrakes.
  // If Cd is too high, error is negative and controller closes airbrakes.
  float cdError = bestCd - Cd;
  if (fabsf(cdError) < CD_CTRL_ERR_DEADBAND) {
    cdError = 0.0f;
  }

  float unsatTarget = posIntegral + CD_CTRL_KP * cdError;
  float targetPos = unsatTarget;
  if (targetPos < MOTOR_MIN) targetPos = MOTOR_MIN;
  if (targetPos > MOTOR_MAX) targetPos = MOTOR_MAX;

  // Anti-windup: stop integrating when saturated and error pushes further out.
  bool pushingHigh = (targetPos >= MOTOR_MAX) && (cdError > 0.0f);
  bool pushingLow = (targetPos <= MOTOR_MIN) && (cdError < 0.0f);
  if (!(pushingHigh || pushingLow)) {
    posIntegral += CD_CTRL_KI * cdError * dt;
    if (posIntegral < MOTOR_MIN) posIntegral = MOTOR_MIN;
    if (posIntegral > MOTOR_MAX) posIntegral = MOTOR_MAX;
  }

  targetPos = posIntegral + CD_CTRL_KP * cdError;
  if (targetPos < MOTOR_MIN) targetPos = MOTOR_MIN;
  if (targetPos > MOTOR_MAX) targetPos = MOTOR_MAX;
  odrvPosition(targetPos);
}