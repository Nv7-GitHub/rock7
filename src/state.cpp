#include "state.h"

State currentState = STATE_IDLE;

// Indicates whether bias calibration is currently active (for LED/debug)
bool biasActive = false;
/*
Color codes:
  IDLE: Blue to Green over 10s
  PAD: Solid Green (dim)
  BOOST: Solid Red
  CONTROL: Solid Yellow
  DESCENT: Solid White
  LANDED: Solid Purple
*/

uint32_t lastNonIdleTime = 0;

// Launch tracking in PAD: arm early and recover pre-roll dv before BOOST.
constexpr uint32_t PAD_LAUNCH_WINDOW_MS = 120;
constexpr uint32_t PAD_LAUNCH_CHECK_DELAY_MS = 20;
constexpr float PAD_LAUNCH_ARM_ACCEL = 11.5f;  // accel magnitude (includes 1g)
constexpr int PAD_LAUNCH_ARM_SAMPLES = 4;      // consecutive samples at 500 Hz
constexpr int PAD_PREROLL_SAMPLES = 150;       // 0.30 s at 500 Hz
constexpr float PAD_LOOP_DT = 1.0f / 500.0f;

float padPrerollAccel[PAD_PREROLL_SAMPLES];
int padPrerollHead = 0;
int padPrerollCount = 0;
int padLaunchConsecutive = 0;

static void resetPadLaunchTracking() {
  padPrerollHead = 0;
  padPrerollCount = 0;
  padLaunchConsecutive = 0;
  for (int i = 0; i < PAD_PREROLL_SAMPLES; ++i) {
    padPrerollAccel[i] = 0.0f;
  }
}

static void pushPadPreroll(float accelMag) {
  // Convert accel magnitude to approximate specific force ahead of launch.
  float specific = accelMag - G;
  if (specific > 120.0f) {
    specific = 120.0f;
  }
  if (specific < -20.0f) {
    specific = -20.0f;
  }

  padPrerollAccel[padPrerollHead] = specific;
  padPrerollHead = (padPrerollHead + 1) % PAD_PREROLL_SAMPLES;
  if (padPrerollCount < PAD_PREROLL_SAMPLES) {
    padPrerollCount++;
  }
}

static float computePadPrerollDv() {
  float dv = 0.0f;
  for (int i = 0; i < padPrerollCount; ++i) {
    int idx =
        (padPrerollHead - 1 - i + PAD_PREROLL_SAMPLES) % PAD_PREROLL_SAMPLES;
    if (padPrerollAccel[idx] > 0.0f) {
      dv += padPrerollAccel[idx] * PAD_LOOP_DT;
    }
  }
  return dv;
}

void stateUpdate() {
  float prog;
  switch (currentState) {
    case STATE_IDLE:
      // Estimator update handles it all, just show color
      prog = (millis() - lastNonIdleTime) / 10000.0f;
      // LED indicator: yellow if low storage, green otherwise
      if (checkStorageWarning()) {
        ledWrite(0.1, 0.1, 0.0);  // Yellow warning
      } else {
        ledWrite(0.0, prog, 1.0f - prog);  // Blue to Green (normal)
      }
      debugPrintf("STATE: IDLE\n");
      break;

    case STATE_PAD:
      if (millis() - lastNonIdleTime < PAD_LAUNCH_WINDOW_MS) {
        uint32_t launchElapsed = millis() - lastNonIdleTime;
        // During potential launch, fade to solid red
        prog = launchElapsed / (float)PAD_LAUNCH_WINDOW_MS;
        ledWrite(prog, 0.0, 0.0);
        debugPrintf("STATE: PAD (LAUNCH WINDOW)\n");
        // Check for launch after brief settle. If accel pulse already decayed,
        // allow velocity-only confirmation in later window half.
        if (launchElapsed > PAD_LAUNCH_CHECK_DELAY_MS) {
          if (x[1] > LAUNCH_VEL &&
              (rawSensorData[0] > LAUNCH_ACCEL ||
               launchElapsed > (PAD_LAUNCH_WINDOW_MS / 2))) {
            currentState = STATE_BOOST;
            resetPadLaunchTracking();
          }
        }
        break;
      }
      // Indicate bias calibration using LEDs when in PAD and not in launch
      // window. Cyan-ish when calibrating, dim green otherwise.
      if (biasActive) {
        ledWrite(0.0, 0.5, 0.5);  // Calibrating (cyan)
      } else {
        ledWrite(0.0, 0.1, 0.0);  // Dim green
      }
      debugPrintf("STATE: PAD\n");
      break;

    case STATE_BOOST:
      ledWrite(1.0, 0.0, 0.0);  // Solid red
      debugPrintf("STATE: BOOST\n");

      // Log all data (GetOrientation called internally at 100Hz)
      logFlightData(x[0], x[1], x[2], rawSensorData[0], rawBaroData, motorpos,
                    motorvel, Cd, Cd, motorcurrent, axisError);

      // See if time for control (look at vertical vel)
      if (x[1] < VEL_CONTROL_START && x[0] > ALT_LANDED &&
          rawSensorData[0] < 0.0f) {
        currentState = STATE_CONTROL;
      }
      break;

    case STATE_CONTROL:
      ledWrite(1.0, 1.0, 0.0);  // Solid yellow
      debugPrintf("STATE: CONTROL\n");

      // Log all data (GetOrientation called internally at 100Hz)
      logFlightData(x[0], x[1], x[2], rawSensorData[0], rawBaroData, motorpos,
                    motorvel, Cd, desiredCd, motorcurrent, axisError);

      controlUpdate();

      // See if apogee reached
      if (x[1] < VEL_DESCENT) {
        currentState = STATE_DESCENT;
      }
      break;

    case STATE_DESCENT:
      ledWrite(1.0, 1.0, 1.0);  // Solid white
      debugPrintf("STATE: DESCENT\n");
      // Log all data (GetOrientation called internally at 100Hz)
      logFlightData(x[0], x[1], x[2], rawSensorData[0], rawBaroData, motorpos,
                    motorvel, Cd, Cd, motorcurrent, axisError);
      odrvPosition(0.0f);  // Closed

      if (x[0] < ALT_LANDED) {
        currentState = STATE_LANDED;
        // Flush log buffer
        flushLogBuffer();
      }
      break;

    case STATE_LANDED:
      debugPrintf("STATE: LANDED\n");
      ledWrite(1.0, 0.0, 1.0);  // Solid purple
      break;
  }
}

void stateInit() {
  currentState = STATE_IDLE;
  lastNonIdleTime = millis();
  resetPadLaunchTracking();
}

void estimatorUpdate() {
  switch (currentState) {
    case STATE_IDLE:
      ReadIMU();
      float aZ;
      float tmp;
      mpu.getAccel(tmp, tmp, aZ);  // Read Z accel
      if (aZ < 9.0f || aZ > 11.0f) {
        // Movement, not idle
        lastNonIdleTime = millis();
      }

      if (millis() - lastNonIdleTime > 10000) {
        // Idle vertical for 10s, go to PAD state
        currentState = STATE_PAD;
        initFlash();           // Initialize flight data file
        hp.startConversion();  // For bias calibration
        lastNonIdleTime = 0;   // Reset for bias calibration
        resetPadLaunchTracking();
      }
      break;

    case STATE_PAD: {
      // Update filter during hypothetical launch window instead of bias
      // calibrating
      if (millis() - lastNonIdleTime < PAD_LAUNCH_WINDOW_MS) {
        FilterUpdate();
        break;
      }

      // Enable odrive
      EnableOdrv();

      // Calibrate bias (mark active for LED indication)
      biasActive = true;
      float acc = BiasUpdate();
      biasActive = false;
      pushPadPreroll(acc);

      if (acc > PAD_LAUNCH_ARM_ACCEL) {
        if (padLaunchConsecutive < PAD_LAUNCH_ARM_SAMPLES) {
          padLaunchConsecutive++;
        }
      } else {
        padLaunchConsecutive = 0;
      }

      if (padLaunchConsecutive >= PAD_LAUNCH_ARM_SAMPLES) {
        // Potential launch: start launch window and seed missing pre-roll dv.
        lastNonIdleTime = millis();
        FilterReset();
        float dv = computePadPrerollDv();
        x[1] += dv;
        debugPrintf("PAD preroll dv injected: %.4f m/s\n", dv);
        padLaunchConsecutive = 0;
      }
      break;
    }

    case STATE_BOOST:
    case STATE_CONTROL:
    case STATE_DESCENT:
      FilterUpdate();
      break;

    case STATE_LANDED:
      // Nothing
      break;
  }
}

// Logging
uint32_t lastPrintTime = 0;
void debugPrintf(const char* format, ...) {
  if (millis() - lastPrintTime < 50) {
    return;  // Limit to 20Hz
  }
  lastPrintTime = millis();

  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);
}