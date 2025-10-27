#define _USE_MATH_DEFINES //This may be windows only.
#include <math.h>
#include <stdint.h>
#include <time.h>
#include <string.h> //needed for memcpy
#include "control_helper.h"



void quaternion_to_euler(float *q1, float *q2, float *q3, float *q4, float *roll, float *pitch, float *yaw){
  /*
  Takes in pointers to the 4 parts of a quaternion (q1, q2, q3, and q4) and computes roll, pitch, yaw
  Writes to the addresses of roll, pitch, and yaw given by the pointers.
  */

  *yaw = atan2f(2.0*(*q3 * *q4 + *q1 * *q2), (*q1 * *q1) - (*q2 * *q2) - (*q3 * *q3) + (*q4 * *q4));
  *pitch = asinf(-2.0*((*q2 * *q4) - (*q1 * *q3)));
  *roll = atan2f(2.0*(*q2 * *q3 + *q1 * *q4), (*q1 * *q1) + (*q2 * *q2) - (*q3 * *q3) - (*q4 * *q4)); 

  // //pi/2 phase shift, mod pi, phase sift back
  if (*roll >= 0){
    *roll = *roll - M_PI;
  }
  else
  {
    *roll = *roll + M_PI;
  }

  return;
}

float parse_a_float(char *buffer, uint16_t j){
  /*
    Takes in a pointer to an array of bytes (buffer) and an index for that array.
    Starting at the byte pointed to by the index, it transforms 4 bytes from the buffer into a floating point value.
    Returns the floating point value.
  */
  
  float f = 0.0;
  char x_bytes[4];

  x_bytes[0] = buffer[j];
  x_bytes[1] = buffer[j+1];
  x_bytes[2] = buffer[j+2];
  x_bytes[3] = buffer[j+3];

  memcpy(&f, x_bytes, sizeof(f));

  return f;
}

void sub_timespec(struct timespec t1, struct timespec t2, struct timespec *td)
/*
    Find the difference between timespec t2 and timespec t1, and store them in td.
    Time difference is both in SECONDS and NANOSECONDS (td.sec and td.nsec, respectively)
*/
{
    td->tv_nsec = t2.tv_nsec - t1.tv_nsec;
    td->tv_sec  = t2.tv_sec - t1.tv_sec;
    if (td->tv_sec > 0 && td->tv_nsec < 0)
    {
        td->tv_nsec += NS_PER_SECOND;
        td->tv_sec--;
    }
    else if (td->tv_sec < 0 && td->tv_nsec > 0)
    {
        td->tv_nsec -= NS_PER_SECOND;
        td->tv_sec++;
    }
}
