#include "sys_config.h"
/*
�ù��̻��ڹ��ʿ����壬imu����ʹ��ģ��IIC���ڲ��������ο�ԭ�ӻ�ȡimu���ݣ��ο�ƽ��С��֮��ʵ�ֿ���
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

float Balance_Kp=100,Balance_Kd=0.4,Velocity_Kp=50,Velocity_Ki=0.25;//PID����
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
#define ZHONGZHI 3  //===���ƽ��ĽǶ���ֵ �ͻ�е���
/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int balance(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-ZHONGZHI;       //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}

///**************************************************************************
//�������ܣ��ٶ�PI���� �޸�ǰ������ң���ٶȣ�����Target_Velocity�����磬�ĳ�60�ͱȽ�����
//��ڲ��������ֱ����������ֱ�����
//����  ֵ���ٶȿ���PWM
//��    �ߣ�ƽ��С��֮��
//**************************************************************************/
//int velocity(int encoder_left,int encoder_right)
//{  
//     static float Velocity,Encoder_Least,Encoder,Movement;
//	  static float Encoder_Integral,Target_Velocity=130;
//	  //=============ң��ǰ�����˲���=======================// 
//		if(1==Flag_Qian)    	Movement=-Target_Velocity;	      //===ǰ����־λ��1 
//		else if(1==Flag_Hou)	Movement=Target_Velocity;         //===���˱�־λ��1
//	  else  Movement=0;	
//	  if(Bi_zhang==1&&Distance<500&&Flag_Left!=1&&Flag_Right!=1)        //���ϱ�־λ��1�ҷ�ң��ת���ʱ�򣬽������ģʽ
//	  Movement=Target_Velocity;
//   //=============�ٶ�PI������=======================//	
//		Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
//		Encoder *= 0.7;		                                                //===һ�׵�ͨ�˲���       
//		Encoder += Encoder_Least*0.3;	                                    //===һ�׵�ͨ�˲���    
//		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
//		Encoder_Integral=Encoder_Integral-Movement;                       //===����ң�������ݣ�����ǰ������
//		if(Encoder_Integral>15000)  	Encoder_Integral=15000;             //===�����޷�
//		if(Encoder_Integral<-15000)	Encoder_Integral=-15000;              //===�����޷�	
//		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;                          //===�ٶȿ���	
//		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      //===����رպ��������
//	  return Velocity;
//}

///**************************************************************************
//�������ܣ�ת�����  �޸�ת���ٶȣ����޸�Turn_Amplitude����
//��ڲ��������ֱ����������ֱ�������Z��������
//����  ֵ��ת�����PWM
//��    �ߣ�ƽ��С��֮��
//**************************************************************************/
//int turn(int encoder_left,int encoder_right,float gyro)//ת�����
//{
//    static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.7,Turn_Count,Kp=42,Kd=0;
//	  float Turn_Amplitude=50/Flag_sudu;    
//	  //=============ң��������ת����=======================//
//  	if(1==Flag_Left||1==Flag_Right)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
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
//		if(1==Flag_Left)	           Turn_Target-=Turn_Convert;        //===����ת��ң������
//		else if(1==Flag_Right)	     Turn_Target+=Turn_Convert;        //===����ת��ң������
//		else Turn_Target=0;                                            //===����ת��ң������
//    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
//	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;   //===ת���ٶ��޷�
//		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.6;                         //===����ת��ң������ֱ�����ߵ�ʱ�����������Ǿ;���    
//		else Kd=0;                                   
//  	//=============ת��PD������=======================//
//		Turn=Turn_Target*Kp+gyro*Kd;                 //===���Z�������ǽ���PD����
//	  return Turn;
//}

int16_t forward_back_value;
int16_t rotat_value;
float pitch,roll,yaw; 		//ŷ����
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;	//������ԭʼ����
short temp;					//�¶�

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
	temp=MPU_Get_Temperature();	              //�õ��¶�ֵ
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
#elif RM_LIB
	mpu_get_data();
	imu_ahrs_update();
	imu_attitude_update(); 
#endif


#if TWO_WHEELS_ON
		/*banlance circle*/
 			chassis.current[1] = chassis.current[0] =balance(imu.pit,imu.ay);                   //===ƽ��PID����	
//		  Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);                  //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
//  	  Turn_Pwm    =turn(Encoder_Left,Encoder_Right,Gyro_Turn);            //===ת��PID����     
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
