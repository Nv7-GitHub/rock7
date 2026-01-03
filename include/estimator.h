#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "hardware.h"

void FilterReset();
void FilterUpdate();
void OrientationBiasUpdate();

extern float aGlob[3];
extern float x[3];

#endif  // ESTIMATOR_H