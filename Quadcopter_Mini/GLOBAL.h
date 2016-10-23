#ifndef GLOBAL_H
#define GLOBAL_H

#define UPPER_LIMIT_THRUST_CMD 50
#define LOWER_LIMIT_THRUST_CMD -100
#define UPPER_LIMIT_Q_CMD 50
#define LOWER_LIMIT_Q_CMD -50



#include "COMM.h"
#include "IMU.h"
#include "CTRL.h"

void process_command();
void safe_mode();

#endif
