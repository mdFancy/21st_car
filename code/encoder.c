#include "zf_common_headfile.h"
Encoder encoder;
void get_encoder(void)
{
  encoder.r_speed = -encoder_get_count(TC_CH09_ENCODER);
  encoder.l_speed = encoder_get_count(TC_CH07_ENCODER);
  encoder_clear_count(TC_CH09_ENCODER);
  encoder_clear_count(TC_CH07_ENCODER);
}    

void encoder_sum_up(void)
{
  if (encoder.flag)
  {
    encoder.sum += (encoder.l_speed + encoder.r_speed)/2;
  }
  else
  {
    encoder.sum = 0;
  }
}