#ifndef __CHASSIS_CTRL_H
#define __CHASSIS_CTRL_H
#include "stdlib.h"
#include "string.h"
#include "sys_config.h"

#define CHASSIS_CTRL_PERIOD (5)
/*this struct is for chassis contrl*/
typedef struct
{
  float vx;
  float vy;
  float vw;
} rc_ctrl_t;

typedef struct
{
  float           vx; // forward/back
  float           vy; // left/right
  float           vw; // 
//  int16_t         rotate_x_offset;
//  int16_t         rotate_y_offset;
//  
//  chassis_mode_e  ctrl_mode;
//  chassis_mode_e  last_ctrl_mode;

//  float           gyro_angle;
//  float           gyro_palstance;

  int16_t         wheel_spd_fdb[4];
  int16_t         wheel_spd_ref[4];
  int16_t         current[4];
//  
//  int16_t         position_ref;
//  uint8_t         follow_gimbal;
} chassis_t;

void chassis_operation_func(int16_t forward_back, int16_t rotate);
//void chassis_stop_handler(void);
void speed_calc(float vx, float vw);
void chassis_param_init(void);
void Start_chassis_ctrl_task(void const * argument);

#endif
