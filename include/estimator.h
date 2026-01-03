#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "hardware.h"

void FilterReset();
void FilterUpdate();
void BiasUpdate();

extern float aGlob[3];
extern float x[3];
extern float rawSensorData[2];

#endif  // ESTIMATOR_H