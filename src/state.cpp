#include "state.h"

State currentState = STATE_IDLE;

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
      if (millis() - lastNonIdleTime < 100) {
        // During potential launch, fade to solid red
        prog = (millis() - lastNonIdleTime) / 100.0f;
        ledWrite(prog, 0.0, 0.0);
        debugPrintf("STATE: PAD (LAUNCH WINDOW)\n");
        // Check for launch, after 50ms cuz don't want any transients
        if (millis() - lastNonIdleTime > 50) {
          if (x[1] > LAUNCH_VEL && rawSensorData[0] > LAUNCH_ACCEL) {
            currentState = STATE_BOOST;
          }
        }
        break;
      }
      ledWrite(0.0, 0.1, 0.0);  // Dim green
      debugPrintf("STATE: PAD\n");
      break;

    case STATE_BOOST:
      ledWrite(1.0, 0.0, 0.0);  // Solid red
      debugPrintf("STATE: BOOST\n");

      // Log all data (GetOrientation called internally at 100Hz)
      logFlightData(x[0], x[1], x[2], rawSensorData[0], rawSensorData[1],
                    motorpos, motorvel, Cd);

      // See if time for control (look at vertical vel)
      if (x[1] < VEL_CONTROL_START) {
        currentState = STATE_CONTROL;
      }
      break;

    case STATE_CONTROL:
      ledWrite(1.0, 1.0, 0.0);  // Solid yellow
      debugPrintf("STATE: CONTROL\n");

      // Log all data (GetOrientation called internally at 100Hz)
      logFlightData(x[0], x[1], x[2], rawSensorData[0], rawSensorData[1],
                    motorpos, motorvel, Cd);

      controlUpdate();

      // See if apogee reached (vel < -0.5m/s)
      if (x[1] < VEL_DESCENT) {
        currentState = STATE_DESCENT;
      }
      break;

    case STATE_DESCENT:
      ledWrite(1.0, 1.0, 1.0);  // Solid white
      debugPrintf("STATE: DESCENT\n");
      // Log all data (GetOrientation called internally at 100Hz)
      logFlightData(x[0], x[1], x[2], rawSensorData[0], rawSensorData[1],
                    motorpos, motorvel, Cd);
      odrv.setPosition(0.0);  // Closed

      if (x[0] < ALT_LANDED && fabsf(x[1]) < VEL_LANDED) {
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
      }
      break;

    case STATE_PAD: {
      // Update filter during hypothetical launch window instead of bias
      // calibrating
      if (millis() - lastNonIdleTime < 100) {
        FilterUpdate();
        break;
      }

      // Enable odrive
      EnableOdrv();

      // Calibrate bias
      float acc = BiasUpdate();
      if (acc > 15.0f) {
        // Could be a launch, trigger 100ms timer before BOOST
        lastNonIdleTime = millis();
        FilterReset();
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