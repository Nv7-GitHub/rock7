#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "hardware.h"

void FilterReset();
void FilterUpdate();
void BiasUpdate();
void GetOrientation(float* roll, float* pitch, float* yaw);

extern float aGlob[3];
extern float x[3];
extern float rawSensorData[2];
extern float C[3][3];

#endif  // ESTIMATOR_H