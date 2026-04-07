#include "zf_common_headfile.h"
PID_Parma PID;
PID_Parma PID_C;
PID_Parma PID_C_R;
uint8 time_counter= 0;
uint8 fast_step = 0; // 增加速度的步长
uint8 err_step = 0; // 错误步长
int speed_max = 0; // 最大速度
int speed_max2 = 0;
uint8 change_flag=0;
void speed_pid_init(void)//实锟斤拷使锟斤拷锟斤拷锟斤拷
{
   pwm_init(TCPWM_CH13_P00_3,10000,0);
   pwm_init(TCPWM_CH50_P18_7,10000,0);
   gpio_init(P00_2, GPO, 0, GPO_PUSH_PULL);
   gpio_init(P18_6, GPO, 0, GPO_PUSH_PULL);
   encoder_dir_init(TC_CH09_ENCODER,TC_CH09_ENCODER_CH1_P05_0,TC_CH09_ENCODER_CH2_P05_1);                 //锟斤拷锟斤拷锟斤拷锟斤拷锟绞硷拷锟�,P5.0锟斤拷锟斤拷锟斤拷P5.1锟斤拷锟斤拷
   encoder_dir_init(TC_CH07_ENCODER,TC_CH07_ENCODER_CH1_P02_0,TC_CH07_ENCODER_CH2_P02_1); 
   encoder.flag = 0;
   encoder.sum = 0;                //锟揭憋拷锟斤拷锟斤拷锟斤拷始锟斤拷,P2.0锟斤拷锟斤拷锟斤拷P2.1锟斤拷锟斤拷
   PID.motor_kp = PID_KEY.motor_kp*0.01;//5.4;                  ////
   PID.motor_ki = PID_KEY.motor_ki*0.01;//2.15;                PID_KEY.motor_kp*0.01; ////
   PID.error_kp = PID_KEY.error_kp*0.01;//1.2;                 PID_KEY.motor_ki*0.01; ////
   PID.error_kd = PID_KEY.error_kd*0.01;//0.85;                PID_KEY.error_kp*0.01; ////
   PID.nav_kp = PID_KEY.nav_kp*0.01;//0.8;                  PID_KEY.error_kp*0.01; ////
  PID.nav_kd = PID_KEY.nav_kd*0.01;//0.85;   
   PID.angle_kp = PID_KEY.angle_kp*0.01;//0.8;                 PID_KEY.error_kd*0.01; ////
   PID.angle_ki = PID_KEY.angle_ki*0.01;//0.02;                PID_KEY.angle_kp*0.01; ////
   PID.aim_error = 0;//0                                       PID_KEY.angle_ki*0.01;
   PID.gyro_kd = PID_KEY.gyro_kd;//40
   PID.aim_angular_speed = 0;//0
   PID.l_speed = 0;
   PID.r_speed = 0;
   PID.base_speed =PID_KEY.base_speed;//35
   PID.l_motor_pid_out = 0;
   PID.r_motor_pid_out = 0;
   PID.bldc_duty = PID_KEY.bldc_duty;
   PID.normal_speed = PID.base_speed;

   PID.circle_speed = 0;

  PID.nav_speed = PID_KEY.nav_speed; // 速度

   PID_C.error_kp = PID_C_KEY.error_kp*0.01;//1.2;                 PID_KEY.motor_kp*0.01; ////
   PID_C.error_kd = PID_C_KEY.error_kd*0.01;//0.85;                PID_KEY.motor_ki*0.01; ////
   PID_C.angle_kp = PID_C_KEY.angle_kp*0.01;//0.8;                 PID_KEY.error_kp*0.01; ////
   PID_C.angle_ki = PID_C_KEY.angle_ki*0.01;//0.02;                PID_KEY.error_kd*0.01; ////
   PID_C.gyro_kd = PID_C_KEY.gyro_kd;//40
   PID_C.base_speed = PID_C_KEY.base_speed;//35


}

void PID_KEY_INIT(void)
{
  PID_KEY.l_speed=0;
  PID_KEY.r_speed =0;
  PID_KEY.base_speed=0;
  PID_KEY.motor_kp=0;
  PID_KEY.motor_ki=0;
  PID_KEY.error_kp=0;
  PID_KEY.error_kd=0;
  PID_KEY.nav_kp=0;
  PID_KEY.nav_kd=0;
  PID_KEY.gyro_kd=0;
  PID_KEY.angle_kp=0;
  PID_KEY.angle_ki=0;
  PID_KEY.aim_error=0;
  PID_KEY.aim_angular_speed=0;
  PID_KEY.l_motor_pid_out=0;
  PID_KEY.r_motor_pid_out=0;

  PID_C_KEY.error_kp = 0;
  PID_C_KEY.error_kd = 0;
  PID_C_KEY.angle_kp = 0;
  PID_C_KEY.angle_ki = 0;
  PID_C_KEY.gyro_kd = 0;
  PID_C_KEY.base_speed = 0;

  PID_KEY.nav_speed = 0;

}
void outside_error_pd(void)
{
  
  static float last_error = 0;
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  error = oriimg_error - PID.aim_error;
  if(road_status == cut)
  {
    error = N.Final_Out - PID.aim_error;
    PID.aim_angular_speed = PID.error_kp * error + PID.error_kd * (error - last_error);
  }
  else if(road_status==lcircle||road_status==rcircle)
  {
    PID.aim_angular_speed = PID_C.error_kp * error + PID_C.error_kd * (error - last_error);
  }
  else
  {
    PID.aim_angular_speed = PID.error_kp * error + PID.error_kd * (error - last_error);
  }
  
  //PID.aim_angular_speed = 30;
  last_error = error;
}


void inside_angle_pi(void)
{
   static float error = 0;
   static float error_sum = 0;
   error =  (PID.aim_angular_speed - tranced_gyro_z);
   error_sum += error;
   if (abs(error_sum) >= 50)
   {
     if (error_sum < 0)
     {

      error_sum = -50;
      
     }
     else
     {
       error_sum= 50;
     }
   }
   if(road_status==cut)
   {
      PID.l_speed = (int16)(PID.nav_speed + (PID.angle_kp * error + PID.angle_ki * error_sum) - PID.gyro_kd * tranced_gyro_z);
      PID.r_speed = (int16)(PID.nav_speed - (PID.angle_kp * error + PID.angle_ki * error_sum) + PID.gyro_kd * tranced_gyro_z);
   }
   else if(road_status==lcircle||road_status==rcircle)
   {
      PID.circle_speed  =  PID_C.base_speed;
      PID.l_speed = (int16)(PID.circle_speed + (PID_C.angle_kp * error + PID_C.angle_ki * error_sum) - PID_C.gyro_kd * tranced_gyro_z);
      PID.r_speed = (int16)(PID.circle_speed - (PID_C.angle_kp * error + PID_C.angle_ki * error_sum) + PID_C.gyro_kd * tranced_gyro_z);
   }
 
   else
   {
    
      if (abs(first_error) <= err_step)
      {

        PID.normal_speed += fast_step;
      }
      else
      {
        PID.normal_speed = PID.base_speed;
      }
      


      if (PID.normal_speed >= speed_max)
      {
        PID.normal_speed = speed_max;
      }
      




      if(speedup_flag&&speedstart)
      {
        
        PID.normal_speed = speed_max2;
        //gpio_set_level(P00_1, 1); 
        
      }

      if (speedup_flag)
      {
        if (slowstart)
        {

          if(change_flag)
          {
            if(choose_plan==1||choose_plan==3)
            {
              turn_planD();
            }
          }
          

          //PID.normal_speed = speed_min;
          // gpio_set_level(P00_1, 1);
          
        }
        else
        {
          if(change_flag)
          {
            if(choose_plan==1)
            {
              turn_planA();
            }
            else if(choose_plan==3)
            {
              turn_planC();
            }
          }
          
        }
      }

      PID.l_speed = (int16)(PID.normal_speed + (PID.angle_kp * error + PID.angle_ki * error_sum) - PID.gyro_kd * tranced_gyro_z);
      PID.r_speed = (int16)(PID.normal_speed - (PID.angle_kp * error + PID.angle_ki * error_sum) + PID.gyro_kd * tranced_gyro_z);
   }

   //涓插彛鍋滆溅
  if (PID.stop_flag )
  {
    PID.base_speed = 0;
    oriimg_error = 0;
    PID.l_speed=0;
    PID.r_speed=0;
    road_status = normal;
    if(PID.bldc_duty <= 0)
    {
      PID.bldc_duty = 0;
    }
    else
    {
      PID.bldc_duty -= 5;
    }
    pwm_set_duty(TCPWM_CH18_P00_0,PID.bldc_duty);

  }
}


 void speed_pi(void)
{
  static int16 l_error_sum = 0;
  static int16 l_error = 0;
  static int16 r_error_sum = 0;
  static int16 r_error = 0;
  
  l_error = PID.l_speed - encoder.l_speed;
  l_error_sum += l_error;
  r_error = PID.r_speed - encoder.r_speed;
  r_error_sum += r_error; 
  
  if (abs(l_error_sum) >= 2000)
  {
    if(l_error_sum <= 0)
    {
      l_error_sum = -2000;
    }
    else
    {
      l_error_sum = 2000;
    }
  }
  
  if (abs(r_error_sum) >= 2000)
  {
    if(r_error_sum <= 0)
    {
      r_error_sum = -2000;
    }
    else
    {
      r_error_sum = 2000;
    }
  }
  PID.l_motor_pid_out = (int16) (PID.motor_kp * l_error + PID.motor_ki * l_error_sum);
  PID.r_motor_pid_out = (int16) (PID.motor_kp * r_error + PID.motor_ki * r_error_sum);
  if(PID.l_motor_pid_out >= 8000)
  {
    PID.l_motor_pid_out = 8000;
  }
  if(PID.r_motor_pid_out >= 8000)
  {
    PID.r_motor_pid_out = 8000;
  }
  motor_pwm_out();
}

void para_pid(void)
{
  static float last_error;
  PID.l_speed= PID.base_speed+ (PID.error_kp* oriimg_error+ PID.error_kd* (oriimg_error - last_error));
  PID.r_speed= PID.base_speed- (PID.error_kp* oriimg_error+ PID.error_kd* (oriimg_error - last_error));
  last_error = oriimg_error;
}

void motor_pwm_out(void)
{
  if (PID.l_motor_pid_out < 0 && PID.r_motor_pid_out < 0)
  {
      gpio_set_level(P00_2, 1);
      gpio_set_level(P18_6, 1);
      pwm_set_duty(TCPWM_CH13_P00_3,abs(PID.r_motor_pid_out));
      pwm_set_duty(TCPWM_CH50_P18_7,abs(PID.l_motor_pid_out));
  }
  else if (PID.l_motor_pid_out < 0 && PID.r_motor_pid_out >= 0)
  {
      gpio_set_level(P00_2, 1);
      gpio_set_level(P18_6, 0);
      pwm_set_duty(TCPWM_CH13_P00_3,abs(PID.r_motor_pid_out));
      pwm_set_duty(TCPWM_CH50_P18_7,abs(PID.l_motor_pid_out));    
  }
  else if (PID.l_motor_pid_out >= 0 && PID.r_motor_pid_out < 0)
  {
      gpio_set_level(P00_2, 0);
      gpio_set_level(P18_6, 1);
      pwm_set_duty(TCPWM_CH13_P00_3,abs(PID.r_motor_pid_out));
      pwm_set_duty(TCPWM_CH50_P18_7,abs(PID.l_motor_pid_out));    
  }
  else if (PID.l_motor_pid_out >= 0 && PID.r_motor_pid_out >= 0)
  {
      gpio_set_level(P00_2, 0);
      gpio_set_level(P18_6, 0);
      pwm_set_duty(TCPWM_CH13_P00_3,abs(PID.r_motor_pid_out));
      pwm_set_duty(TCPWM_CH50_P18_7,abs(PID.l_motor_pid_out));    
  }
}

void increase_speed_pid(void)
{
  static int16 l_error_1 = 0;
  static int16 l_error_2 = 0;
  static int16 r_error_1 = 0;
  static int16 r_error_2 = 0;
  float inc_l= 0;
  float inc_r = 0;

  l_error_1 = PID.l_speed - encoder.l_speed;
  r_error_1 = PID.r_speed - encoder.r_speed;
  inc_l = PID.motor_kp * (l_error_1 - l_error_2) + PID.motor_ki  * l_error_1 ;  //缁撴灉杈撳嚭
  inc_r = PID.motor_kp * (r_error_1 - r_error_2) + PID.motor_ki  * r_error_1 ;  //缁撴灉杈撳嚭
  PID.l_motor_pid_out  += inc_l;
  PID.r_motor_pid_out  += inc_r;
  l_error_2 = l_error_1;
  r_error_2 = r_error_1;
  if (PID.l_motor_pid_out  >= 3000)
  {
    PID.l_motor_pid_out  = 3000;
  }
  if (PID.r_motor_pid_out  >= 3000)
  {
    PID.r_motor_pid_out  = 3000;
  }
  motor_pwm_out();
}