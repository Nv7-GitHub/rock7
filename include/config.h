#ifndef CONFIG_H
#define CONFIG_H

#include "math.h"

// Changes
#define MASS 0.62f               // kg
#define BASE_CD 0.44f            // From OpenRocket
#define LAUNCH_ACCEL 30.0f       // m/s^2, acceleration at launch detection
#define LAUNCH_VEL 5.0f          // m/s, velocity at launch detection
#define VEL_CONTROL_START 60.0f  // m/s, vel drops below this to start control

// Constants
#define AREA (M_PI * (6.6f / 100.0f) * (6.6f / 100.0f)) / 4.0f  // m^2
#define RHO 1.225f                                              // kg/m^3
#define G 9.80665f                                              // m/s^2
constexpr float rhoA = RHO * AREA;  // Precomputed RHO * A

#endif  // CONFIG_H