#include <Arduino.h>
#include <Wire.h>

#include "state.h"

// Make setup blocking
semaphore_t setup_done;

void setup() {
  sem_init(&setup_done, 0, 1);

  setupHardware();
  hp.startConversion();
  while (true) {
    if (hp.isConversionReady()) {
      hp.readAllData();
      debugPrintf("Baro: %.2f m, pres: %.2f Pa, temp: %.2f C\n",
                  hp.hp_sensorData.A, hp.hp_sensorData.P, hp.hp_sensorData.T);
      hp.startConversion();
    }
  }

  // Flash logging will be initialized when entering STATE_PAD

  // Indicate setup done
  sem_release(&setup_done);
}

void loop() {
  handleFlashCommands();

  // Update state machine
  stateUpdate();
}

void setup1() {
  // wait for core0 setup to finish
  sem_acquire_blocking(&setup_done);
}

void loop1() {
  // Update filter
  estimatorUpdate();
}