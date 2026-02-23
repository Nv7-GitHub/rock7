#include "control.h"

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
  // Stay closed during any residual positive accel (late motor burn)
  if (aGlob[2] >= 0.0f) {
    odrvPosition(MOTOR_MIN);
    return;
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

  // Map optimal Cd to a motor position using the current drag per position
  // Cd = COAST_CD_MIN + dragPerPos * motorpos  =>  dragPerPos = (Cd -
  // COAST_CD_MIN) / motorpos
  float dragPerPos;
  if (motorpos > 1.0f) {
    dragPerPos = (Cd - COAST_CD_MIN) / motorpos;
  } else {
    dragPerPos = (COAST_CD_MAX - COAST_CD_MIN) /
                 MOTOR_MAX;  // default when nearly closed
  }

  float targetPos = MOTOR_MIN;
  if (fabsf(dragPerPos) > 1e-6f) {
    targetPos = (bestCd - COAST_CD_MIN) / dragPerPos;
  }

  if (targetPos < MOTOR_MIN) targetPos = MOTOR_MIN;
  if (targetPos > MOTOR_MAX) targetPos = MOTOR_MAX;
  odrvPosition(targetPos);
}