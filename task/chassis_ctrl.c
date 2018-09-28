#include "chassis_ctrl.h"
#include "sys_config.h"
#include "stdlib.h"
#include "string.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "pid.h"
#include "cmsis_os.h"


rc_ctrl_t   rm;
chassis_t chassis;
uint16_t test_given[4];
void chassis_operation_func(int16_t forward_back, int16_t rotate)
{
  rm.vx =  forward_back / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X;
//  rm.vy = -left_right / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y;
  rm.vw = -rotate / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_R;
 
  chassis.vx = rm.vx * CHASSIS_RC_MOVE_RATIO_X; 
//  chassis.vy = rm.vy * CHASSIS_RC_MOVE_RATIO_Y;
  chassis.vw = rm.vw * CHASSIS_RC_MOVE_RATIO_R;
}

//void chassis_stop_handler(void)
//{
//  chassis.vy = 0;
//  chassis.vx = 0;
//  chassis.vw = 0;
//}
	int16_t wheel_rpm[4];
void speed_calc(float vx, float vw)
{

	
	VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
//	VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
	VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s
	
	wheel_rpm[0] = (vx / RADIUS) / CHASSIS_DECELE_RATIO + ((vw * (WHEELTRACK / 2.0)) / RADIUS) / CHASSIS_DECELE_RATIO;
	wheel_rpm[1] = (vx / RADIUS) / CHASSIS_DECELE_RATIO - ((vw * (WHEELTRACK / 2.0)) / RADIUS) / CHASSIS_DECELE_RATIO;
	wheel_rpm[2] = (vx / RADIUS) / CHASSIS_DECELE_RATIO - ((vw * (WHEELTRACK / 2.0)) / RADIUS) / CHASSIS_DECELE_RATIO;
	wheel_rpm[3] = (vx / RADIUS) / CHASSIS_DECELE_RATIO + ((vw * (WHEELTRACK / 2.0)) / RADIUS) / CHASSIS_DECELE_RATIO;
	
	VAL_LIMIT(wheel_rpm[0], -MAX_WHEEL_RPM, MAX_WHEEL_RPM);  //rpm
	VAL_LIMIT(wheel_rpm[1], -MAX_WHEEL_RPM, MAX_WHEEL_RPM);  //rpm
	VAL_LIMIT(wheel_rpm[2], -MAX_WHEEL_RPM, MAX_WHEEL_RPM);  //rpm
	VAL_LIMIT(wheel_rpm[3], -MAX_WHEEL_RPM, MAX_WHEEL_RPM);  //rpm
	
  for (int i = 0; i < 4; i++)
  {
	if(rc.sw1 == 2)
		chassis.wheel_spd_ref[i] = 0;
	else
		chassis.wheel_spd_ref[i] = wheel_rpm[i];
		
		chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm;
    chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
  }
}
  
/**
  * @brief  nitialize chassis motor pid parameter
  * @usage  before chassis loop use this function
  */
void chassis_param_init(void)
{
  memset(&chassis, 0, sizeof(chassis_t));
  
//  chassis.ctrl_mode      = CHASSIS_STOP;
//  chassis.last_ctrl_mode = CHASSIS_RELAX;
  
  for (int k = 0; k < 4; k++)
  {
    PID_struct_init(&pid_spd[k], POSITION_PID, 10000, 500, 20.5f, 0.05, 0);
  }
  
  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 50, 14.0f, 0.0f, 50.0f);
  
//  glb_struct.chassis_config = NO_CONFIG;
//  glb_struct.gimbal_config  = NO_CONFIG;
}

void Start_chassis_ctrl_task(void const * argument)
{
	chassis_param_init();
  HAL_GPIO_WritePin(GPIOH, PW24V_1_Pin|PW24V_2_Pin|PW24V_3_Pin|PW24V_4_Pin, GPIO_PIN_SET);
	uint32_t chassis_ctrl_wake_time = osKernelSysTick();
  while(1)
  {
    chassis_operation_func(rc.ch2, rc.ch3);
		speed_calc(chassis.vx, chassis.vw);
    send_chassis_cur(chassis.current);
		osDelayUntil(&chassis_ctrl_wake_time, CHASSIS_CTRL_PERIOD);
  }
  
}
