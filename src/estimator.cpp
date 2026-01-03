#include "estimator.h"

#include "algebra.h"

// Constants
#define LOOPRATE 500  // Hz
const float dT = 1.0f / (float)LOOPRATE;

// Orientation estimation
float C[3][3];

// Sensors
extern Mpu6500 mpu;
void ReadIMU() { mpu.readSensor(); }

float gBias[3] = {0.0f, 0.0f, 0.0f};

// Gyro bias handling
void OrientationBiasUpdate() {
  unsigned long start = micros();
  ReadIMU();
  float g[3];
  mpu.getGyro(g[0], g[1], g[2]);

  // Simple low-pass filter, use negative cuz we wanna subtract this later
  float alpha = 0.01f;
  gBias[0] = (1.0f - alpha) * gBias[0] - alpha * g[0];
  gBias[1] = (1.0f - alpha) * gBias[1] - alpha * g[1];
  gBias[2] = (1.0f - alpha) * gBias[2] - alpha * g[2];

  // Delay to looprate
  unsigned long deltmicros = micros() - start;
  if (deltmicros < (unsigned long)(dT * 1e6f)) {
    delayMicroseconds((unsigned long)(dT * 1e6f) - deltmicros);
  } else {
    Serial.println("ERROR: BIAS LOOP OVERRUN");
  }
}

// Initializes orientation matrix to follow gravity vector
void OrientationInit() {
  // Read
  float a[3];
  mpu.getAccel(a[0], a[1], a[2]);
  if (a[2] == 0.0f) {
    a[2] = 0.0001f;  // Prevent division by zero
  }

  // Initialize C
  float tmp[3];
  copyVector(tmp, a);
  normalizeVector(tmp);

  float cx = atanf(tmp[1] / tmp[2]);
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
  matrixDotVector3x3(aGlob, C, aBody);

  // Subtract gravity
  aGlob[2] -= 9.80665f;
}

// Kalman filter
float x[3];
float P[3][3];

void FilterReset() {
  ReadIMU();
  OrientationInit();
}
void FilterUpdate() {
  // Timing
  unsigned long start = micros();

  // Read IMU
  ReadIMU();

  // Update orientation filter
  OrientationUpdate();
  GetGlobalAccel();

  // Delay to looprate
  unsigned long deltmicros = micros() - start;
  if (deltmicros < (unsigned long)(dT * 1e6f)) {
    delayMicroseconds((unsigned long)(dT * 1e6f) - deltmicros);
  } else {
    Serial.println("ERROR: LOOP OVERRUN");
  }
}