#include <Arduino.h>
#include <Wire.h>

#include "state.h"

// Make setup blocking
semaphore_t setup_done;

void setup() {
  sem_init(&setup_done, 0, 1);

  setupHardware();

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