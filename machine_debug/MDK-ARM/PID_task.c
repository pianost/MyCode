#include "PID_task.h"
#include "motor.h"
#include "tim.h"        
#include "ramp.h"
#include "math.h"             
#include "VCOMCOMM.h"
/*
SPEED_MODE下，速度环单环控制3510和3508，Ex_v1,Ex_v2,Ex_v3,Ex_v4分别控制电机速度  注意id
RM3150_MODE下，位置环双环控制3508和2006，
		3508位置控制 expect_angle_3508.expect_1-4分别控制四个电机的位置，    expect_wheel_ramp_3508：斜坡
		2006位置控制 expect_angle_2006.expect_1-4分别控制四个电机的位置，    expect_wheel_ramp_2006：斜坡
*/
float Ex_v1=0; 
float Ex_v2=0;
float Ex_v3=0;
float Ex_v4=0;
float Ex_v5=0;
float Ex_v6=0;
float Ex_v7=0;
float Ex_v8=0;
int16_t LV53_distance;
int16_t CAN1_txdata[4];
int16_t CAN2_txdata[4];
Data_Typedef rubbish = {.id = 12, };  //数据接收
float count = 0;
Expect_Speed_Typedef Expect_Speed;
Chassis_Motor_expect expect_wheel_3508;
Chassis_Motor_expect expect_angle_3508;
Chassis_Motor_expect expect_wheel_2006;
Chassis_Motor_expect expect_angle_2006;
int32_t expect_wheel_ramp_3508[4]={200,200,200,200};   //电机斜坡
int32_t expect_wheel_ramp_2006[4]={367,367,367,367};   //电机斜坡
RM3510_TypeDef motor_wheel_3510[2];      
RM3508_TypeDef motor_wheel_3508[4];
M2006_TypeDef motor_wheel_2006[4];

Robotic_arm arm = {.l1=290.0f,.l2=150.0f,.l3=100.0f,.d=150.0f};
float x=400,h=190;

int32_t wheel_offset[4];
int32_t M2006_offset[4];
#if SPEED_MODE
PID PID_wheel_speed[4]={{.Kp=4.3, .Ki=0.004, .Kd=4.5,.limit=1000000},
						{.Kp=4.3, .Ki=0.001, .Kd=4.5,.limit=5000},
                        {.Kp=4.3, .Ki=0.001, .Kd=4.5,.limit=5000},
						{.Kp=4.3, .Ki=0.001, .Kd=4.5,.limit=5000},};
#else 
PID PID_wheel_speed[4]={{.Kp=2, .Ki=0, .Kd=2,.limit=5000},
						{.Kp=2, .Ki=0, .Kd=2,.limit=5000},
                        {.Kp=2, .Ki=0, .Kd=2,.limit=5000},
						{.Kp=2, .Ki=0, .Kd=2,.limit=5000},};

#endif

PID_Smis PID_wheel_position[4]={{.Kp=2, .Ki=0, .Kd=-2.5, .limit=5000},   //5 0 -6
								{.Kp=2, .Ki=0, .Kd=-2.5, .limit=5000},    //3 0 -5
                                {.Kp=2, .Ki=0, .Kd=-2.5, .limit=2000},    //1 0 -2
                                {.Kp=2, .Ki=0, .Kd=-2.5, .limit=2000}};

PID M2006_Speed[4]={{.Kp=2,.Ki=0,.Kd=2,.limit = 5000},
					{.Kp=2,.Ki=0,.Kd=2,.limit = 5000},
					{.Kp=2,.Ki=0,.Kd=2,.limit = 5000},
					{.Kp=2,.Ki=0,.Kd=2,.limit = 5000}};

PID_Smis M2006_Angle[4]={{.Kp=3,.Ki=0,.Kd=-3,.limit = 5000},
						 {.Kp=3,.Ki=0,.Kd=-3,.limit = 5000},
						 {.Kp=3,.Ki=0,.Kd=-3,.limit = 5000},
						 {.Kp=3,.Ki=0,.Kd=-3,.limit = 5000}};							
								
TaskHandle_t PID_Handler;	
								
#if SPEED_MODE							
void PID_Task(void *pvParameters){ //电机pid控制任务    
    
	portTickType xLastWakeTime = xTaskGetTickCount();
	
	for(;;){
		//3508电机速度控制
		PID_Control(motor_wheel_3508[0].Speed,Ex_v1,&PID_wheel_speed[0]);
		limit(PID_wheel_speed[0].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[0]=PID_wheel_speed[0].pid_out;
		
		PID_Control(motor_wheel_3508[1].Speed,Ex_v2,&PID_wheel_speed[1]);
		limit(PID_wheel_speed[1].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[1]=PID_wheel_speed[1].pid_out;
		
		PID_Control(motor_wheel_3508[2].Speed,Ex_v3,&PID_wheel_speed[2]);
		limit(PID_wheel_speed[2].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[2]=PID_wheel_speed[2].pid_out;
		
		PID_Control(motor_wheel_3508[3].Speed,Ex_v4,&PID_wheel_speed[3]);
		limit(PID_wheel_speed[3].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[3]=PID_wheel_speed[3].pid_out;
		 
#if RM3510_MODE
		
		//3510电机速度控制		 			
		PID_Control(motor_wheel_3510[0].Speed,Ex_v1,&PID_wheel_speed[0]);
		limit(PID_wheel_speed[0].pid_out,RM3510_LIMIT,-RM3510_LIMIT);
		CAN1_txdata[0]=PID_wheel_speed[0].pid_out;
		
		PID_Control(motor_wheel_3510[1].Speed,Ex_v2,&PID_wheel_speed[1]);
		limit(PID_wheel_speed[1].pid_out,RM3510_LIMIT,-RM3510_LIMIT);
		CAN1_txdata[1]=PID_wheel_speed[1].pid_out;
//		//3508电机速度控制
//		PID_Control(motor_wheel_3508[0].Speed,Ex_v3,&PID_wheel_speed[2]);
//		limit(PID_wheel_speed[2].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
//		CAN1_txdata[2]=PID_wheel_speed[2].pid_out;
//		
//		PID_Control(motor_wheel_3508[1].Speed,Ex_v4,&PID_wheel_speed[3]);
//		limit(PID_wheel_speed[3].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
//		CAN1_txdata[3]=PID_wheel_speed[3].pid_out;
#endif		
		MotorSend(&hcan1,0x200,CAN1_txdata);
		
		//2006速度模式控制		
//		PID_Control(motor_wheel_2006[0].Speed,Ex_v5,&M2006_Speed[0]);
//		limit(M2006_Speed[0].pid_out,M2006_LIMIT,-M2006_LIMIT);
//		CAN2_txdata[0]=M2006_Speed[0].pid_out;
		
		PID_Control(motor_wheel_2006[1].Speed,Ex_v6,&M2006_Speed[1]);
		limit(M2006_Speed[1].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[1]=M2006_Speed[1].pid_out;
			
		PID_Control(motor_wheel_2006[2].Speed,Ex_v7,&M2006_Speed[2]);
		limit(M2006_Speed[2].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[2]=M2006_Speed[2].pid_out;
		
		PID_Control(motor_wheel_2006[3].Speed,Ex_v8,&M2006_Speed[3]);
		limit(M2006_Speed[3].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[3]=M2006_Speed[3].pid_out;
		MotorSend(&hcan2, 0x200, CAN2_txdata);// ID:201~204
		//单独的2006位置模式
		expect_wheel_2006.expect_1 = RAMP_self(expect_angle_2006.expect_1,expect_wheel_2006.expect_1,expect_wheel_ramp_2006[0]);
		PID_Control_Smis(motor_wheel_2006[0].Angle - M2006_offset[0],expect_wheel_2006.expect_1,&M2006_Angle[0],motor_wheel_2006[0].Speed);
 		limit(M2006_Angle[0].pid_out,M2006_LIMIT,-M2006_LIMIT);		
		PID_Control(motor_wheel_2006[0].Speed,M2006_Angle[0].pid_out,&M2006_Speed[0]);
		limit(M2006_Speed[0].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[0]=M2006_Speed[0].pid_out;
		vTaskDelay(1);
		vTaskDelayUntil(&xLastWakeTime,2);
  }
}
#else
void PID_Task(void *pvParameters){ //电机pid控制任务
	
    wheel_offset[0] = motor_wheel_3508[0].Angle;//   纠偏
    wheel_offset[1] = motor_wheel_3508[1].Angle;
    wheel_offset[2] = motor_wheel_3508[2].Angle;
	wheel_offset[3] = motor_wheel_3508[3].Angle;
    
	M2006_offset[0] = motor_wheel_2006[0].Angle;//   纠偏
    M2006_offset[1] = motor_wheel_2006[1].Angle;
	M2006_offset[2] = motor_wheel_2006[2].Angle;
	M2006_offset[3] = motor_wheel_2006[3].Angle;
    
	portTickType xLastWakeTime = xTaskGetTickCount();
	
	for(;;){
#if RM3510_MODE
		//3510位置模式控制
		PID_Control_Smis(motor_wheel_3510[0].Angle - wheel_offset[0],expect_wheel_3508.expect_1,&PID_wheel_position[0],motor_wheel_3510[0].Speed);
		PID_Control(motor_wheel_3510[0].Speed,PID_wheel_position[0].pid_out,&PID_wheel_speed[0]);
		limit(PID_wheel_speed[0].pid_out,RM3510_LIMIT,-RM3510_LIMIT);
		CAN1_txdata[0]=PID_wheel_speed[0].pid_out;
		
		PID_Control_Smis(motor_wheel_3510[1].Angle - wheel_offset[1],expect_wheel_3508.expect_2,&PID_wheel_position[1],motor_wheel_3510[1].Speed);
		PID_Control(motor_wheel_3510[1].Speed,PID_wheel_position[1].pid_out,&PID_wheel_speed[1]);
		limit(PID_wheel_speed[1].pid_out,RM3510_LIMIT,-RM3510_LIMIT);
		CAN1_txdata[1]=PID_wheel_speed[1].pid_out;
		
		PID_Control_Smis(motor_wheel_3508[2].Angle - wheel_offset[2],expect_wheel_3508.expect_3,&PID_wheel_position[2],motor_wheel_3508[2].Speed);
		PID_Control(motor_wheel_3508[2].Speed,PID_wheel_position[2].pid_out,&PID_wheel_speed[2]);
		limit(PID_wheel_speed[2].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[2]=PID_wheel_speed[2].pid_out;
#else
		//3508位置模式控制
		expect_wheel_3508.expect_1 = RAMP_self(expect_angle_3508.expect_1,expect_wheel_3508.expect_1,expect_wheel_ramp_3508[0]);
		PID_Control_Smis(motor_wheel_3508[0].Angle - wheel_offset[0],expect_wheel_3508.expect_1,&PID_wheel_position[0],motor_wheel_3508[0].Speed);
		PID_Control(motor_wheel_3508[0].Speed,PID_wheel_position[0].pid_out,&PID_wheel_speed[0]);
		limit(PID_wheel_speed[0].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[0]=PID_wheel_speed[0].pid_out;
		
		expect_wheel_3508.expect_2 = RAMP_self(expect_angle_3508.expect_2,expect_wheel_3508.expect_2,expect_wheel_ramp_3508[1]);
		PID_Control_Smis(motor_wheel_3508[1].Angle - wheel_offset[1],expect_wheel_3508.expect_2,&PID_wheel_position[1],motor_wheel_3508[1].Speed);
		PID_Control(motor_wheel_3508[1].Speed,PID_wheel_position[1].pid_out,&PID_wheel_speed[1]);
		limit(PID_wheel_speed[1].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[1]=PID_wheel_speed[1].pid_out;
		
		expect_wheel_3508.expect_3 = RAMP_self(expect_angle_3508.expect_3,expect_wheel_3508.expect_3,expect_wheel_ramp_3508[2]);
		PID_Control_Smis(motor_wheel_3508[2].Angle - wheel_offset[2],expect_wheel_3508.expect_3,&PID_wheel_position[2],motor_wheel_3508[2].Speed);
		PID_Control(motor_wheel_3508[2].Speed,PID_wheel_position[2].pid_out,&PID_wheel_speed[2]);
		limit(PID_wheel_speed[2].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[2]=PID_wheel_speed[2].pid_out;
		
		expect_wheel_3508.expect_4 = RAMP_self(expect_angle_3508.expect_4,expect_wheel_3508.expect_4,expect_wheel_ramp_3508[3]);
		PID_Control_Smis(motor_wheel_3508[3].Angle - wheel_offset[3],expect_wheel_3508.expect_4,&PID_wheel_position[3],motor_wheel_3508[3].Speed);
		PID_Control(motor_wheel_3508[3].Speed,PID_wheel_position[3].pid_out,&PID_wheel_speed[3]);
		limit(PID_wheel_speed[3].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[3]=PID_wheel_speed[3].pid_out;
#endif		
		MotorSend(&hcan1,0x200,CAN1_txdata);
		
		//2006位置模式控制
		expect_wheel_2006.expect_1 = RAMP_self(expect_angle_2006.expect_1,expect_wheel_2006.expect_1,expect_wheel_ramp_2006[0]);
		PID_Control_Smis(motor_wheel_2006[0].Angle - M2006_offset[0],expect_wheel_2006.expect_1,&M2006_Angle[0],motor_wheel_2006[0].Speed);
 		limit(M2006_Angle[0].pid_out,M2006_LIMIT,-M2006_LIMIT);		
		PID_Control(motor_wheel_2006[0].Speed,M2006_Angle[0].pid_out,&M2006_Speed[0]);
		limit(M2006_Speed[0].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[0]=M2006_Speed[0].pid_out;
		
		expect_wheel_2006.expect_2 = RAMP_self(expect_angle_2006.expect_2,expect_wheel_2006.expect_2,expect_wheel_ramp_2006[1]);
		PID_Control_Smis(motor_wheel_2006[1].Angle - M2006_offset[1],expect_wheel_2006.expect_2,&M2006_Angle[1],motor_wheel_2006[1].Speed);
 		limit(M2006_Angle[1].pid_out,M2006_LIMIT,-M2006_LIMIT);		
		PID_Control(motor_wheel_2006[1].Speed,M2006_Angle[0].pid_out,&M2006_Speed[1]);
		limit(M2006_Speed[1].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[1]=M2006_Speed[1].pid_out;
		
		expect_wheel_2006.expect_3 = RAMP_self(expect_angle_2006.expect_3,expect_wheel_2006.expect_3,expect_wheel_ramp_2006[2]);
		PID_Control_Smis(motor_wheel_2006[2].Angle - M2006_offset[2],expect_wheel_2006.expect_3,&M2006_Angle[2],motor_wheel_2006[2].Speed);
 		limit(M2006_Angle[2].pid_out,M2006_LIMIT,-M2006_LIMIT);		
		PID_Control(motor_wheel_2006[2].Speed,M2006_Angle[2].pid_out,&M2006_Speed[2]);
		limit(M2006_Speed[2].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[2]=M2006_Speed[2].pid_out;
		
		expect_wheel_2006.expect_4 = RAMP_self(expect_angle_2006.expect_4,expect_wheel_2006.expect_4,expect_wheel_ramp_2006[3]);
		PID_Control_Smis(motor_wheel_2006[3].Angle - M2006_offset[3],expect_wheel_2006.expect_4,&M2006_Angle[3],motor_wheel_2006[3].Speed);
 		limit(M2006_Angle[3].pid_out,M2006_LIMIT,-M2006_LIMIT);		
		PID_Control(motor_wheel_2006[3].Speed,M2006_Angle[3].pid_out,&M2006_Speed[3]);
		limit(M2006_Speed[3].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[3]=M2006_Speed[3].pid_out;
		MotorSend(&hcan2, 0x200, CAN2_txdata);// ID:201~204
		
		vTaskDelay(1);
		vTaskDelayUntil(&xLastWakeTime,2);
  }
}
#endif
							
TaskHandle_t Angle_Handler;
int32_t row_angle =0;    
int32_t col_angle =0;
int16_t high = 0;    //电推杆位置
int16_t put_flag = 2;
int16_t pos_flag = 0;
int16_t pep_flag = 0;
int w1 = 1; 
int put_ball = 0;	
int duoji_1 = 0;
int duoji_2 = 0;
int duoji_3 = 1465;
int duoji_4 = 0;
int flag1 = 1;
int flag2 = 0;
int jiao_a = 91;
int jiao_b = 88;
int duoji = 0;
int Mode0 = 0;
int time = 350;
int flag_miao = 0;
int ab = 0;
float count1 = 1;
int test_count = 1;
void Angle_Task(void *pvParameters){
	//put_flag = 2;
	portTickType xLastWakeTime = xTaskGetTickCount();
	for(;;){
		if(pep_flag == 0)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		}
		if(pep_flag == 1)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
		}
		
		
		//电推杆伸缩调试 high控制电推杆伸缩
		if(put_flag == 1)
		{
			textexp = high;
			vTaskDelay(2);
		}

		//四路pwm波 duoji_1 duoji_2 duoji_3 duoji_4分别对应tim1-4
		if(put_flag == 2)//取苗
		{
			//左1 右2
//			duoji_1 = 2000 - duoji;
//			duoji_1 = duoji;
//			duoji_2 = duoji;
//			duoji_3 = 2000 - duoji;
//			duoji_4 = duoji;
			if(Mode0 == 0)//取苗
			{
			//	ab = HAL_GPIO_ReadPin(GuangDianMenInPut_GPIO_Port,GuangDianMenInPut_Pin);
				duoji_3 = 775;flag2 = 1;
			if(LV53_distance >=45 && LV53_distance <=160 && flag_miao == 0) //苗到了
			{
				flag_miao = 1;
				osDelay(pdMS_TO_TICKS(300));
				flag1 = 0;
				jiaqu();
				osDelay(pdMS_TO_TICKS(500));
				flag1 = 1;
				jiaqu();
				osDelay(pdMS_TO_TICKS(time));
				count +=count1;
				jiaqu();
			  osDelay(pdMS_TO_TICKS(500));
			
			expect_angle_2006.expect_1 = ((8192 * 5 * 36) / 2 / 7.2)*count*test_count;
				 osDelay(pdMS_TO_TICKS(450));
			while(w1)
			{
				
				count +=0.02;
				osDelay(pdMS_TO_TICKS(10));
				expect_angle_2006.expect_1 = ((8192 * 5 * 36) / 2 / 7.2)*count;
			  osDelay(pdMS_TO_TICKS(10));
				if(HAL_GPIO_ReadPin(GuangDianMenInPut_GPIO_Port,GuangDianMenInPut_Pin) == 0){w1 = 0;count1 = count;break;}
	
			}
			/*
			while(HAL_GPIO_ReadPin(GuangDianMenInPut_GPIO_Port,GuangDianMenInPut_Pin) == 0)
			{
				count +=0.05;
				osDelay(pdMS_TO_TICKS(200));
				expect_angle_2006.expect_1 = ((8192 * 5 * 36) / 2 / 7.2)*count;
				count1 = count;
			}
			
			*/
			flag_miao = 0;
			LV53_distance = 0;			
			}
					
		 }
			else if(Mode0 == 1)//放苗
			{
				if(flag2 == 1)
				{
					duoji_3 = -50;
					jiaqu();
					count +=count1;
					flag2 = 0;
				}
				osDelay(pdMS_TO_TICKS(750));
			  expect_angle_2006.expect_1 = ((8192 * 5 * 36) / 2 / 6)*count;
				
			}
	
			
		}
		if(put_flag == 5)//放苗
		{
			
		}
		//凯乐的取苗
		if(put_flag == 3)
		{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,duoji);
			vTaskDelay(10);
			jiaqu();
		}
		//通胜的夹取
		if(put_flag == 4)
		{
			
			if(pos_flag == 0)
			{
				duoji_1 = 0;
				expect_angle_2006.expect_1 = 0;
				expect_angle_3508.expect_3 = 0;
			}
			if(pos_flag == 1)
			{
				expect_angle_2006.expect_1 = 120000;  // 10000
				expect_angle_3508.expect_3 = -70000; // -60000
				duoji_1 = 400;
			}
			if(pos_flag == 2)
			{                        
				expect_angle_2006.expect_1 = (int)(8192 * 36 * 2.2 * ((jiao_a - arm.a) / 360)); 
				expect_angle_3508.expect_3 = (int)(8192 * 19 * 2.27 * ((jiao_b - arm.a - arm.b) / 360));
				duoji_1 = 224 + (int)(arm.b / 300 * 2000);		
			}
			
			vTaskDelay(10);
			jiaqu();
			Get_angle(arm.l3+h-arm.d,x);
//			vTaskDelay(1000);
//			h = 150;
		}
		vTaskDelay(2);
		vTaskDelayUntil(&xLastWakeTime,2);
  }
}

TaskHandle_t vN5055_Handle;
int myflag = 0;
float speed_n5055_1 = 5,speed_n5055_2 = 5;//5055速度控制
//5055电机控制任务
void vN5055_task(void *parameters)
{
    odrive_set_axis0.axis_node_id = 0x010 ;
    odrive_set_axis1.axis_node_id = 0x020 ;
    for(;;)
    {   
			if(myflag == 1) {
			odrive_set_axis0.input_vel = speed_n5055_1;
			odrive_set_axis1.input_vel = speed_n5055_2; 
			}
			else if(myflag == 2) {      
				odrive_set_axis0.input_vel = 0;
				odrive_set_axis1.input_vel = 0;
				myflag = 0;
			}
			osDelay(5);
			odrive_can_write(AXIS_0,MSG_SET_INPUT_VEL);
			osDelay(5);
			odrive_can_write(AXIS_1,MSG_SET_INPUT_VEL);
    }
}

void Get_angle(float h,float x)
{
	//printf("%.2f ",h);
	float r_lc=sqrt(h*h+x*x); //直角边
	float c_ac = acos((arm.l1*arm.l1+arm.l2*arm.l2-r_lc*r_lc)/(2*arm.l1*arm.l2))/PI * 180; 
	float r_ab = asin(x/r_lc)/PI*180;
	if(h < 0) r_ab = 180 - r_ab;
	float c_ab = acos((r_lc*r_lc+arm.l2*arm.l2-arm.l1*arm.l1)/(2*r_lc*arm.l2))/PI * 180;

	arm.a = 180 - r_ab - c_ab;
	arm.b = c_ac - arm.a;
	//return arm;
}


void setPos(int32_t *m, int32_t pos, int32_t ramp) {//电推杆位置控制
    int32_t  i = 0;
    i = (abs(pos - *m) / ramp) + 1; 
    while(i--){
          *m = RAMP_self(pos ,*m,ramp);
          vTaskDelay(35);
    }
	
//    vTaskDelay(timmm);
//    xSemaphoreGive(semaphore_flash);
}


//四路pwm波 duoji_1 duoji_2 duoji_3 duoji_4分别对应tim1-4
void jiaqu()     //舵机控制控制 
{					
	//DUOJI1 1350 - 300   DUOJI2 200 - 1450
	if(flag1 == 0)
	{
		duoji_1 = 1850; //500 + 1850
		duoji_2 = 700;//500 + 700
	}
	else if(flag1 == 1)
	{
		duoji_1 = 650;//500 + 800
		duoji_2 = 1950;//500 + 
	}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,500+duoji_1); //0-200 1160+
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,500+duoji_2); //0-1350
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,500+duoji_3);  //0-775
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,500+duoji_4);  //350 - 1050
}

int32_t RAMP_self( int32_t final, int32_t now, int32_t ramp ) //斜坡函数
{
    float buffer = final - now;
    
    if (buffer > 0)
    {
        if (buffer > ramp)  
                now += ramp;  
        else
                now += buffer;
    }		
    else
    {
        if (buffer < -ramp)
                now += -ramp;
        else
                now += buffer;
    }
    return now;
    
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){     //can回调
	
	if (hcan->Instance == CAN1) {
		uint16_t ID1 = CAN_Receive_DataFrame(&hcan1,CAN1_buff);     
		switch(ID1){
#if RM3510_MODE
			case 0x201: 
				RM3510_Receive(&motor_wheel_3510[0],CAN1_buff);
				break;
			case 0x202: 
				RM3510_Receive(&motor_wheel_3510[1],CAN1_buff);
				break;
			case 0x203: 
				RM3508_Receive(&motor_wheel_3508[0],CAN1_buff);
				break;
			case 0x204: 
				RM3508_Receive(&motor_wheel_3508[1],CAN1_buff);
				break;
		
#else		
			case 0x201: 
				RM3508_Receive(&motor_wheel_3508[0],CAN1_buff);
				break;
			case 0x202: 
				RM3508_Receive(&motor_wheel_3508[1],CAN1_buff);
				break;
			case 0x203: 
				RM3508_Receive(&motor_wheel_3508[2],CAN1_buff);
				break;
			case 0x204: 
				RM3508_Receive(&motor_wheel_3508[3],CAN1_buff);
				break;  
#endif
			default: break;
		}			
	
        while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0))
		{
			CAN_RxHeaderTypeDef header;
			if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
			{
				CanMessage_t rxmsg;
				HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, rxmsg.buf);

				rxmsg.id = header.StdId;
				rxmsg.len = header.DLC;
				rxmsg.rtr = header.RTR;
				if(rxmsg.id == AXIS0_NODE_ID + getmode)
				{
					memcpy(&rx_msg_0,&rxmsg,sizeof(CanMessage_t));
				}
				else if(rxmsg.id == AXIS1_NODE_ID + getmode)
				{
					memcpy(&rx_msg_1,&rxmsg,sizeof(CanMessage_t));
				}
			}
		}
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	
	uint16_t ID1 = CAN_Receive_DataFrame(&hcan2,CAN2_buff);     
	switch(ID1){             
		case 0x11B:
			CAN_RoboModule_DRV_Feedback(&putter,CAN2_buff); //电推杆数据接收
			break;
		case 0x201: 
			M2006_Receive(&motor_wheel_2006[0], CAN2_buff);
			break;
		case 0x202: 
			M2006_Receive(&motor_wheel_2006[1], CAN2_buff);
			break;
		case 0x203: 
			M2006_Receive(&motor_wheel_2006[2], CAN2_buff);
			break;
		case 0x204: 
			M2006_Receive(&motor_wheel_2006[3], CAN2_buff);
			break;       
		default: break;
	}
	
				
}

void VCOMM_CallBack(uint8_t fun_code, uint16_t id, uint8_t *data, uint8_t len)        //接收pc端数据，详细见队库
{
	memcpy(&Expect_Speed, data, sizeof(Expect_Speed_Typedef));
	//VCOMM_Transmit(fun_code, id, data, len);
}



