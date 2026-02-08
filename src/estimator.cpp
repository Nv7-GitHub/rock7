#include "estimator.h"

#include "algebra.h"
#include "state.h"

// Constants
#define LOOPRATE 500  // Hz
const float dT = 1.0f / (float)LOOPRATE;

// Experimentally calibrated z-accel data
// const float sigma_aZ = 0.0332f;  // m/s^2, experimental
const float sigma_aZ = 0.044f;   // m/s^2, calculated from datasheet
const float sigma_bZ = 0.0014f;  // m/s^2 per sqrt(s), experimental
const float sigma_x = 2.1f;      // m, experimental
const float scale_aZ =
    1.0f /
    1.015f;  // Accelerometer scale factor, for some reason only z is scaled

// Orientation estimation
float C[3][3];

// Sensors
extern Mpu6500 mpu;
void ReadIMU() { mpu.readSensor(); }

float gBias[3] = {0.0f, 0.0f, 0.0f};
float hpBias = 0.0f;
float aBias[3] = {0.0f, 0.0f, 0.0f};

// Gyro bias handling, note: run hp.startConversion() before
float BiasUpdate() {
  unsigned long start = micros();
  ReadIMU();
  float g[3];
  mpu.getGyro(g[0], g[1], g[2]);

  // Simple low-pass filter, use negative cuz we wanna subtract this later
  float alpha = 0.01f;
  gBias[0] = (1.0f - alpha) * gBias[0] - alpha * g[0];
  gBias[1] = (1.0f - alpha) * gBias[1] - alpha * g[1];
  gBias[2] = (1.0f - alpha) * gBias[2] - alpha * g[2];

  // Barometer bias
  if (hp.isConversionReady()) {
    hp.readAllData();
    if (fabsf(hpBias) < 0.01f) {
      hpBias = hp.hp_sensorData.A;
    } else {
      hpBias = 0.9f * hpBias + 0.1f * hp.hp_sensorData.A;
    }
    debugPrintf("Baro: %.2f m, Bias: %.2f m\n", hp.hp_sensorData.A, hpBias);
    hp.startConversion();
  }

  // Accel bias (Z only)
  float a[3];
  mpu.getAccel(a[0], a[1], a[2]);
  a[2] *= scale_aZ;  // Scale correction

  float aMag;
  vectorLength(&aMag, a);

  // Subtract gravity from accel vector, in direction of magnitude
  float accNorm[3];
  copyVector(accNorm, a);
  normalizeVector(accNorm);
  scaleVector(accNorm, G, accNorm);
  subtractVectors(a, a, accNorm);
  // Also negative for same reason as gyro
  aBias[0] = (1.0f - alpha) * aBias[0] - alpha * a[0];
  aBias[1] = (1.0f - alpha) * aBias[1] - alpha * a[1];
  aBias[2] = (1.0f - alpha) * aBias[2] - alpha * a[2];

  // Delay to looprate
  unsigned long deltmicros = micros() - start;
  if (deltmicros < (unsigned long)(dT * 1e6f)) {
    delayMicroseconds((unsigned long)(dT * 1e6f) - deltmicros);
  } else {
    Serial.println("ERROR: BIAS LOOP OVERRUN");
  }

  // Return magnitude of accel vector for launch detect
  return aMag;
}

// Initializes orientation matrix to follow gravity vector
void OrientationInit() {
  // Read
  float a[3];
  mpu.getAccel(a[0], a[1], a[2]);
  a[2] *= scale_aZ;  // Scale correction
  if (a[2] == 0.0f) {
    a[2] = 0.0001f;  // Prevent division by zero
  }

  // Initialize C
  float tmp[3];
  copyVector(tmp, a);
  normalizeVector(tmp);

  float cx = atan2f(tmp[1], tmp[2]);
  float cy = -asinf(tmp[0]);

  if (isnan(cx) || isnan(cy)) {
    return;
  }

  float scx = sin(cx);
  float ccx = cos(cx);
  float scy = sin(cy);
  float ccy = cos(cy);
  C[0][0] = ccy;
  C[0][1] = scx * scy;
  C[0][2] = ccx * scy;

  C[1][0] = 0;
  C[1][1] = ccx;
  C[1][2] = -scx;

  C[2][0] = -scy;
  C[2][1] = ccy * scx;
  C[2][2] = ccx * ccy;
}

void OrientationUpdate() {
  // Read gyro
  float g[3];
  mpu.getGyro(g[0], g[1], g[2]);

  // Make sure none is 0
  if (g[0] == 0.0f) {
    g[0] = 0.0001f;
  }
  if (g[1] == 0.0f) {
    g[1] = 0.0001f;
  }
  if (g[2] == 0.0f) {
    g[2] = 0.0001f;
  }

  // Subtract bias
  sumVectors(g, g, gBias);

  // Rotation matrix code is from
  // https://huskiecommons.lib.niu.edu/cgi/viewcontent.cgi?article=1032&context=allgraduate-thesesdissertations

  // Calculate sigma
  float sig;
  vectorLength(&sig, g);
  sig = dT * sig;
  float csig = sinf(sig) / sig;
  float ssig = (1 - cosf(sig)) / pow(sig, 2);

  // Calculate B
  float B[3][3];

  float phi[3][3];  // "B"
  skew(phi, g);
  scaleMatrix3x3(phi, dT, phi);
  float tmp[3][3];  // "B^2"
  matrixProduct3x3(tmp, phi, phi);

  identityMatrix3x3(B);
  scaleAndAccumulateMatrix3x3(B, csig, phi);
  scaleAndAccumulateMatrix3x3(B, ssig, tmp);

  // Save
  matrixProduct3x3(tmp, C, B);
  copyMatrix3x3(C, tmp);
}

float aGlob[3];
void GetGlobalAccel() {
  float aBody[3];
  mpu.getAccel(aBody[0], aBody[1], aBody[2]);
  aBody[2] *= scale_aZ;             // Scale correction
  sumVectors(aBody, aBody, aBias);  // Bias calibration
  matrixDotVector3x3(aGlob, C, aBody);

  // Subtract gravity
  aGlob[2] -= G;
}

// Kalman filter state
float x[3];
float P[3][3];
float Cd;
float rawSensorData[2] = {0.0f, 0.0f};

// Kalman filter constants
float A[3][3] = {
    {1.0f, dT, -0.5f * dT* dT},
    {0.0f, 1.0f, -dT},
    {0.0f, 0.0f, 1.0f},
};
float B[3] = {0.5f * dT * dT, dT, 0.0f};
float Q[3][3] = {
    {sigma_aZ * sigma_aZ * dT * dT * dT * dT / 4.0f,
     sigma_aZ * sigma_aZ * dT * dT * dT / 2.0f, 0.0f},
    {sigma_aZ * sigma_aZ * dT * dT * dT / 2.0f, sigma_aZ * sigma_aZ * dT* dT,
     0.0f},
    {0.0f, 0.0f, sigma_bZ * sigma_bZ * dT},
};
float AT[3][3];  // A transpose
float R = sigma_x * sigma_x;

void FilterReset() {
  ReadIMU();
  OrientationInit();
  transposeMatrix3x3(AT, A);
  hp.startConversion();

  // Initialize state
  x[0] = 0.0f;
  x[1] = 0.0f;
  x[2] = 0.0f;  // Bias calibrated out in local frame

  // Initialize covariance to 0
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      P[i][j] = 0.0f;
    }
  }

  // Initial coefficient of drag
  Cd = BASE_CD;
}

const float alpha_cd = 0.05f;  // 4.2 Hz cutoff @ 500 Hz looprate
void FilterUpdate() {
  // Timing
  unsigned long start = micros();

  // Read IMU
  ReadIMU();

  // Update orientation filter
  OrientationUpdate();
  GetGlobalAccel();

  // Kalman prediction step
  float x_pred[3];
  matrixDotVector3x3(x_pred, A, x);
  x_pred[0] += B[0] * (aGlob[2]);
  x_pred[1] += B[1] * (aGlob[2]);
  x_pred[2] += B[2] * (aGlob[2]);
  copyVector(x, x_pred);

  float AP[3][3];
  matrixProduct3x3(AP, A, P);
  float P_pred[3][3];
  matrixProduct3x3(P_pred, AP, AT);
  scaleAndAccumulateMatrix3x3(P_pred, 1.0f, Q);
  copyMatrix3x3(P, P_pred);

  // See if we have a new altitude reading
  if (hp.isConversionReady()) {
    hp.readAllData();
    hp.startConversion();

    // Only do Kalman update during coast phase, high accel (3G) or vel (40m/s)
    // make it unreliable
    if (fabsf(aGlob[2]) < 30.0f && fabsf(x[1]) < 40.0f) {
      // Kalman update step
      float z = hp.hp_sensorData.A - hpBias;
      float y = z - x[0];  // innovation
      float S = P[0][0] + R;
      float K[3];  // Kalman gain
      K[0] = P[0][0] / S;
      K[1] = P[1][0] / S;
      K[2] = P[2][0] / S;
      x[0] += K[0] * y;
      x[1] += K[1] * y;
      x[2] += K[2] * y;
      float I_KH[3][3] = {
          {1.0f - K[0], 0.0f, 0.0f},
          {-K[1], 1.0f, 0.0f},
          {-K[2], 0.0f, 1.0f},
      };
      float I_KH_P[3][3];
      matrixProduct3x3(I_KH_P, I_KH, P);
      copyMatrix3x3(P, I_KH_P);

      rawSensorData[1] = z;  // For flight data
    }
  }

  rawSensorData[0] = aGlob[2];  // For flight data

  // Update Cd with low-pass
  float currCd = -(2.0f * MASS * (aGlob[2] + G)) / (rhoA * x[1] * fabsf(x[1]));
  Cd = (1.0f - alpha_cd) * Cd + alpha_cd * currCd;

  // Delay to looprate
  unsigned long deltmicros = micros() - start;
  if (deltmicros < (unsigned long)(dT * 1e6f)) {
    delayMicroseconds((unsigned long)(dT * 1e6f) - deltmicros);
  } else {
    Serial.println("ERROR: LOOP OVERRUN");
    Serial.printf("Loop time: %lu us\n", deltmicros);
  }
}

// Extract roll, pitch, yaw from C matrix (Direction Cosine Matrix)
void GetOrientation(float* roll, float* pitch, float* yaw) {
  // Extract Euler angles from rotation matrix C
  // Using ZYX (yaw-pitch-roll) convention
  // C is body-to-global, so we need to extract angles from the transpose

  // pitch = asin(-C[2][0])
  *pitch = asinf(-C[2][0]);

  // roll = atan2(C[2][1], C[2][2])
  *roll = atan2f(C[2][1], C[2][2]);

  // yaw = atan2(C[1][0], C[0][0])
  *yaw = atan2f(C[1][0], C[0][0]);
}