#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "zf_common_headfile.h"
typedef struct encoder
{
  int16 l_speed;
  int16 r_speed;
  uint8 flag;
  int32 sum;
}Encoder;
extern Encoder encoder;
void get_encoder(void);
void encoder_sum_up(void);
#endif