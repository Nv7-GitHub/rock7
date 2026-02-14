#include "control.h"

void controlUpdate() {
  // odrv.setPosition(24.0f);  // FULL DEPLOYMENT!
  odrv.setPosition(sinf(millis() / 1000.0f * 2.0f * M_PI) * 12.0f +
                   12.0f);  // Sinewave deployment
}