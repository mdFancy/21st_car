#ifndef _PID_H_
#define _PID_H_
#include "zf_common_headfile.h"
typedef struct PID_Parma
{
  int16 l_speed;
  int16 r_speed;
  int16 base_speed;
  float motor_kp;
  float motor_ki;
  float error_kp;
  float error_kd;
  float nav_kp;
  float nav_kd;
  float gyro_kd;
  float angle_kp;
  float angle_ki;
  float aim_error;
  float aim_angular_speed;
  int16 l_motor_pid_out;
  int16 r_motor_pid_out;
  uint8 stop_flag;
  uint8 bldc_flag;
  uint16 bldc_duty;
  int16 normal_speed;
  int16 circle_speed;
  uint16 nav_speed; // 速度
}PID_Parma;
typedef struct PID_Parma_KEY
{
  uint16 l_speed;
  uint16 r_speed;
  uint16 base_speed;
  uint16 motor_kp;
  uint16 motor_ki;
  uint16 error_kp;
  uint16 error_kd;
    uint16 nav_kp;
  uint16 nav_kd;
  uint16 gyro_kd;
  uint16 angle_kp;
  uint16 angle_ki;
  uint16 aim_error;
  uint16 aim_angular_speed;
  uint16 l_motor_pid_out;
  uint16 r_motor_pid_out;
  uint16 nav_speed; // 速度
  uint16 bldc_duty;
}PID_Parma_KEY;
extern float error;
extern PID_Parma PID;
extern PID_Parma_KEY PID_KEY;
extern PID_Parma PID_C;
extern PID_Parma_KEY PID_C_KEY;
extern PID_Parma PID_C_R;
extern PID_Parma_KEY PID_C_R_KEY;
extern int speed_max2;
void angle_pid(void);
void outside_error_pd(void);
void inside_angle_pi(void);
void para_pid(void);
void speed_pi(void);
void speed_pid_init(void);
void motor_pwm_out(void);
void PID_KEY_INIT(void);
void increase_speed_pid(void);
#endif