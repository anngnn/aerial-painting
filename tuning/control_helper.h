#ifndef HELPER_H
#define HELPER_H
#include <stdint.h>



#define NS_PER_SECOND 1000000000

// Function declarations
void quaternion_to_euler(float *q1, float *q2, float *q3, float *q4, float *roll, float *pitch, float *yaw);
float parse_a_float(char *buffer, uint16_t j);
void sub_timespec(struct timespec t1, struct timespec t2, struct timespec *td);





#endif