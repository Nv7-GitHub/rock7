#ifndef CONFIG_H
#define CONFIG_H

#include "math.h"

// Changes
#define MASS 0.603f         // kg
#define BASE_CD 0.44f       // From OpenRocket
#define LAUNCH_ACCEL 30.0f  // m/s^2, acceleration at launch detection
#define LAUNCH_VEL 1.0f     // m/s, velocity at launch detection (real: 4.5)
#define VEL_CONTROL_START 50.0f  // m/s, vel drops below this to start control
#define VEL_DESCENT -1.0f        // m/s, vel drops below this to go to DESCENT
#define ALT_LANDED 2.0f          // m, alt below this to consider landed
#define VEL_LANDED 1.0f          // m/s, abs(vel) below this to consider landed

// Constants
#define AREA (M_PI * (6.6f / 100.0f) * (6.6f / 100.0f)) / 4.0f  // m^2
#define RHO 1.225f                                              // kg/m^3
#define G 9.80665f                                              // m/s^2
constexpr float rhoA = RHO * AREA;  // Precomputed RHO * A

#endif  // CONFIG_H