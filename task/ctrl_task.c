#include "sys_config.h"
/*
该工程基于国际开发板，imu部分使用模拟IIC，内部上拉，参考原子获取imu数据，参考平衡小车之家实现控制
*/
rc_ctrl_t   rm;
chassis_t chassis;
uint16_t test_given[4];
#if FOUR_WHEELS_ON
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
	
	wheel_rpm[0] = (vx / RADIUS) / CHASSIS_DECELE_RATIO - ((vw * (WHEELTRACK / 2.0)) / RADIUS) / CHASSIS_DECELE_RATIO;
	wheel_rpm[1] = (vx / RADIUS) / CHASSIS_DECELE_RATIO - ((vw * (WHEELTRACK / 2.0)) / RADIUS) / CHASSIS_DECELE_RATIO;
	wheel_rpm[2] = -(vx / RADIUS) / CHASSIS_DECELE_RATIO - ((vw * (WHEELTRACK / 2.0)) / RADIUS) / CHASSIS_DECELE_RATIO;
	wheel_rpm[3] = -(vx / RADIUS) / CHASSIS_DECELE_RATIO - ((vw * (WHEELTRACK / 2.0)) / RADIUS) / CHASSIS_DECELE_RATIO;
	
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
#endif

/**
  * @brief  nitialize chassis motor pid parameter
  * @usage  before chassis loop use this function
  */
void chassis_param_init(void)
{
  memset(&chassis, 0, sizeof(chassis_t));
  
	/*four motors pid parameter init*/
  for (int k = 0; k < 4; k++)
  {
    PID_struct_init(&pid_spd[k], POSITION_PID, 10000, 500, 20.5f, 0.05, 0);
  }
	/*chassis tilt angle ctrl pid parameter init*/
  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 50, 14.0f, 0.0f, 50.0f);
	/*imu temperature pid parameter init*/
	PID_struct_init(&pid_imu_tmp, POSITION_PID, MAX_CHASSIS_VR_SPEED, 50, 14.0f, 0.0f, 50.0f);
	/*motor power on*/
	HAL_GPIO_WritePin(GPIOH, PW24V_1_Pin|PW24V_2_Pin|PW24V_3_Pin|PW24V_4_Pin, GPIO_PIN_SET);
}

float Balance_Kp=100,Balance_Kd=0.4,Velocity_Kp=50,Velocity_Ki=0.25;//PID参数
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
#define ZHONGZHI 3  //===求出平衡的角度中值 和机械相关
/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balance(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-ZHONGZHI;       //===求出平衡的角度中值 和机械相关
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}

///**************************************************************************
//函数功能：速度PI控制 修改前进后退遥控速度，请修Target_Velocity，比如，改成60就比较慢了
//入口参数：左轮编码器、右轮编码器
//返回  值：速度控制PWM
//作    者：平衡小车之家
//**************************************************************************/
//int velocity(int encoder_left,int encoder_right)
//{  
//     static float Velocity,Encoder_Least,Encoder,Movement;
//	  static float Encoder_Integral,Target_Velocity=130;
//	  //=============遥控前进后退部分=======================// 
//		if(1==Flag_Qian)    	Movement=-Target_Velocity;	      //===前进标志位置1 
//		else if(1==Flag_Hou)	Movement=Target_Velocity;         //===后退标志位置1
//	  else  Movement=0;	
//	  if(Bi_zhang==1&&Distance<500&&Flag_Left!=1&&Flag_Right!=1)        //避障标志位置1且非遥控转弯的时候，进入避障模式
//	  Movement=Target_Velocity;
//   //=============速度PI控制器=======================//	
//		Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
//		Encoder *= 0.7;		                                                //===一阶低通滤波器       
//		Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
//		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
//		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
//		if(Encoder_Integral>15000)  	Encoder_Integral=15000;             //===积分限幅
//		if(Encoder_Integral<-15000)	Encoder_Integral=-15000;              //===积分限幅	
//		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;                          //===速度控制	
//		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      //===电机关闭后清除积分
//	  return Velocity;
//}

///**************************************************************************
//函数功能：转向控制  修改转向速度，请修改Turn_Amplitude即可
//入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
//返回  值：转向控制PWM
//作    者：平衡小车之家
//**************************************************************************/
//int turn(int encoder_left,int encoder_right,float gyro)//转向控制
//{
//    static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.7,Turn_Count,Kp=42,Kd=0;
//	  float Turn_Amplitude=50/Flag_sudu;    
//	  //=============遥控左右旋转部分=======================//
//  	if(1==Flag_Left||1==Flag_Right)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
//		{
//			if(++Turn_Count==1)
//			Encoder_temp=myabs(encoder_left+encoder_right);
//			Turn_Convert=50/Encoder_temp;
//			if(Turn_Convert<0.4)Turn_Convert=0.4;
//			if(Turn_Convert>1)Turn_Convert=1;
//		}	
//	  else
//		{
//			Turn_Convert=0.7;
//			Turn_Count=0;
//			Encoder_temp=0;
//		}		
//		if(1==Flag_Left)	           Turn_Target-=Turn_Convert;        //===接收转向遥控数据
//		else if(1==Flag_Right)	     Turn_Target+=Turn_Convert;        //===接收转向遥控数据
//		else Turn_Target=0;                                            //===接收转向遥控数据
//    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
//	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;   //===转向速度限幅
//		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.6;                         //===接收转向遥控数据直立行走的时候增加陀螺仪就纠正    
//		else Kd=0;                                   
//  	//=============转向PD控制器=======================//
//		Turn=Turn_Target*Kp+gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
//	  return Turn;
//}

int16_t forward_back_value;
int16_t rotat_value;
float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;					//温度

void Start_chassis_ctrl_task(void const * argument)
{
	/*imu init*/
#if SPI_DMP_ON || IIC_DMP_ON
MPU_Init();
while(mpu_dmp_init());	
#elif RM_LIB
	mpu_device_init();
	init_quaternion();		
#endif

	/*power on beep*/
	
  chassis_param_init();
  uint32_t chassis_ctrl_wake_time = osKernelSysTick();
  while(1)
  {

#if SPI_DMP_ON || IIC_DMP_ON
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	temp=MPU_Get_Temperature();	              //得到温度值
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
#elif RM_LIB
	mpu_get_data();
	imu_ahrs_update();
	imu_attitude_update(); 
#endif


#if TWO_WHEELS_ON
		/*banlance circle*/
 			chassis.current[1] = chassis.current[0] =balance(imu.pit,imu.ay);                   //===平衡PID控制	
//		  Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);                  //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
//  	  Turn_Pwm    =turn(Encoder_Left,Encoder_Right,Gyro_Turn);            //===转向环PID控制     
			VAL_LIMIT(chassis.current[0], -MAX_WHEEL_RPM, MAX_WHEEL_RPM);  //rpm
			VAL_LIMIT(chassis.current[1], -MAX_WHEEL_RPM, MAX_WHEEL_RPM);  //rpm
#elif FOUR_WHEELS_ON
chassis_operation_func(forward_back_value, rotat_value);
speed_calc(chassis.vx, chassis.vw);
#endif

    send_chassis_cur(chassis.current);
    osDelayUntil(&chassis_ctrl_wake_time, CTRL_TASK_PERIOD);
  }
  
}
