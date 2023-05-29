#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"



//??�����õ�??
#define CHASSIS_DISABLE 0

#define MecanumWheel 0

//in the beginning of task ,wait a time
#define CHASSIS_TASK_INIT_TIME 357


#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Y_CHANNEL 0
#define CHASSIS_WZ_CHANNEL 2
#define CHASSIS_MODE_CHANNEL 0
//rocker value (max 660) change to vertial speed (m/s) 
#define CHASSIS_VX_RC_SEN 0.006f
//rocker value (max 660) change to horizontal speed (m/s)
#define CHASSIS_VY_RC_SEN 0.005f
//in following yaw angle mode, rocker value add to angle 
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//in not following yaw angle mode, rocker value change to rotation speed
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f 

//rocker value deadline
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

#define CHASSIS_CONTROL_TIME_MS 2
#define CHASSIS_CONTROL_TIME 0.002f
/*chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;*/
#define CHASSIS_CONTROL_FREQUENCE 500.0f
#define MAX_MOTOR_CAN_CURRENT 15000.0f
//����ҡ��
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//W,A,S,D
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

/*TOP*/ 
#define CHASSIS_TOP_KEY KEY_PRESSED_OFFSET_F

/*45°*/
#define CHASSIS_ROTATION_ANG KEY_PRESSED_OFFSET_Z

//m3508 rmp change to chassis speed
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

////single chassis motor max speed
//#define MAX_WHEEL_SPEED 4.0f
//chassis forward or back max speed
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//chassis left or right max speed
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

//#define RAD_2_ANGLE 57.2957795f    // 180/pi
//#define ANGLE_2_RAD 0.01745329252f // pi/180
//#define WHEEL_BASE 300              // 纵向轴距(前进后退方向)
//#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
//#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
//#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
//#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * ANGLE_2_RAD)
//#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * ANGLE_2_RAD)
//#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * ANGLE_2_RAD)
//#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * ANGLE_2_RAD)
#define CHASSIS_WZ_SET_SCALE 0.15f


#define SWING_NO_MOVE_ANGLE 0.7f
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f



#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 0.01f
#define M3505_MOTOR_SPEED_PID_KD 2.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 12.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f	
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 10.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.0f


 



#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f
#define Fast_Run_flag KEY_PRESSED_OFFSET_SHIFT
#define CLASSIS_CHONGNEN_MODE KEY_PRESSED_OFFSET_Q

#define MAX_WHEEL_SPEED_40 0.7f
#define NORMAL_MAX_CHASSIS_SPEED_X_40 0.7f
#define NORMAL_MAX_CHASSIS_SPEED_X_40_t 1.5f

#define MAX_WHEEL_SPEED_40_t 1.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y_40 0.7f
#define NORMAL_MAX_CHASSIS_SPEED_Y_40_t 1.2f
#define NORMAL_MAX_CHASSIS_SPEED_Z_40 10.0f

//�����˶��������ǰ���ٶ�   45
#define MAX_WHEEL_SPEED_45 0.9f
#define NORMAL_MAX_CHASSIS_SPEED_X_45 0.9f
#define NORMAL_MAX_CHASSIS_SPEED_X_45_t 1.7f
//�����˶��������ƽ���ٶ�   45
#define MAX_WHEEL_SPEED_45_t 1.7f
#define NORMAL_MAX_CHASSIS_SPEED_Y_45 0.9f
#define NORMAL_MAX_CHASSIS_SPEED_Y_45_t 1.4f
#define NORMAL_MAX_CHASSIS_SPEED_Z_45 10.0f

//�����˶��������ǰ���ٶ�   50
#define MAX_WHEEL_SPEED_50 1.2f
#define NORMAL_MAX_CHASSIS_SPEED_X_50 1.1f
#define NORMAL_MAX_CHASSIS_SPEED_X_50_t 1.5f
//�����˶��������ƽ���ٶ�   50
#define MAX_WHEEL_SPEED_50_t 1.6f
#define NORMAL_MAX_CHASSIS_SPEED_Y_50 1.2f
#define NORMAL_MAX_CHASSIS_SPEED_Y_50_t 1.4f
#define NORMAL_MAX_CHASSIS_SPEED_Z_50 10.0f

//�����˶��������ǰ���ٶ�   55
#define MAX_WHEEL_SPEED_55 1.2f
#define NORMAL_MAX_CHASSIS_SPEED_X_55 1.2f
#define NORMAL_MAX_CHASSIS_SPEED_X_55_t 2.0f
//�����˶��������ƽ���ٶ�   55
#define MAX_WHEEL_SPEED_55_t 2.0f
#define NORMAL_MAX_CHASSIS_SPEED_Y_55 1.3f
#define NORMAL_MAX_CHASSIS_SPEED_Y_55_t 1.7f
#define NORMAL_MAX_CHASSIS_SPEED_Z_55 10.0f

//�����˶��������ǰ���ٶ�  60
#define MAX_WHEEL_SPEED_60 2.0f
#define NORMAL_MAX_CHASSIS_SPEED_X_60 1.8f
#define NORMAL_MAX_CHASSIS_SPEED_X_60_t 2.5f
//�����˶��������ƽ���ٶ�  60
#define MAX_WHEEL_SPEED_60_t 1.9f
#define NORMAL_MAX_CHASSIS_SPEED_Y_60 1.2f
#define NORMAL_MAX_CHASSIS_SPEED_Y_60_t 1.5f
#define NORMAL_MAX_CHASSIS_SPEED_Z_60 20.0f

//�����˶��������ǰ���ٶ�   80
#define MAX_WHEEL_SPEED_80 1.7f
#define NORMAL_MAX_CHASSIS_SPEED_X_80 1.6f
#define NORMAL_MAX_CHASSIS_SPEED_X_80_t 2.0f
//�����˶��������ƽ���ٶ�   80
#define MAX_WHEEL_SPEED_80_t 2.1f
#define NORMAL_MAX_CHASSIS_SPEED_Y_80 1.4f
#define NORMAL_MAX_CHASSIS_SPEED_Y_80_t 1.6f
#define NORMAL_MAX_CHASSIS_SPEED_Z_80 20.0f
//�����˶��������ƽ���ٶ�   80����
#define MAX_WHEEL_SPEED_80_FLY 4.7f
#define NORMAL_MAX_CHASSIS_SPEED_X_80_FLY 4.7f
#define NORMAL_MAX_CHASSIS_SPEED_Y_80_FLY 4.7f



//�����˶��������ǰ���ٶ�   100
#define MAX_WHEEL_SPEED_100 1.8f
#define NORMAL_MAX_CHASSIS_SPEED_X_100 1.7f
#define NORMAL_MAX_CHASSIS_SPEED_X_100_t 2.2f
//�����˶��������ƽ���ٶ�  100
#define MAX_WHEEL_SPEED_100_t 2.3f
#define NORMAL_MAX_CHASSIS_SPEED_Y_100 1.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y_100_t 1.9f
#define NORMAL_MAX_CHASSIS_SPEED_Z_100 20.0f
//�����˶��������ƽ���ٶ�   100����
#define MAX_WHEEL_SPEED_100_FLY 3.5f
#define NORMAL_MAX_CHASSIS_SPEED_X_100_FLY 3.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y_100_FLY 3.5f

//�����˶��������ǰ���ٶ�   120
#define MAX_WHEEL_SPEED_120 4.0f
#define NORMAL_MAX_CHASSIS_SPEED_X_120 4.0f
#define NORMAL_MAX_CHASSIS_SPEED_X_120_t 4.0f
//�����˶��������ƽ���ٶ�  120
#define MAX_WHEEL_SPEED_120_t 3.5f
#define NORMAL_MAX_CHASSIS_SPEED_Y_120 3.4f
#define NORMAL_MAX_CHASSIS_SPEED_Y_120_t 2.8f
#define NORMAL_MAX_CHASSIS_SPEED_Z_120 10.0f
//�����˶��������ƽ���ٶ�   120����
#define MAX_WHEEL_SPEED_120_FLY 4.7f
#define NORMAL_MAX_CHASSIS_SPEED_X_120_FLY 4.7f
#define NORMAL_MAX_CHASSIS_SPEED_Y_120_FLY 4.0f


typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       
  CHASSIS_VECTOR_RAW,            
  CHASSIS_VECTOR_TOP,
  CHASSIS_VECTOR_ROT,
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;



typedef struct
{
  const RC_ctrl_t *chassis_RC;             
  const gimbal_motor_t *chassis_yaw_motor;  
  const gimbal_motor_t *chassis_pitch_motor; 
  const fp32 *chassis_INS_angle;
  chassis_mode_e chassis_mode;          
  chassis_mode_e last_chassis_mode;      
  chassis_motor_t motor_chassis[4];         
  pid_type_def motor_speed_pid[4];        
  pid_type_def chassis_angle_pid;      

  first_order_filter_type_t chassis_cmd_slow_set_vx; 
  first_order_filter_type_t chassis_cmd_slow_set_vy;  

  fp32 vx;                         
  fp32 vy;                  
  fp32 wz;                      
  fp32 vx_set;                 
  fp32 vy_set;                 
  fp32 wz_set;                 
  fp32 chassis_relative_angle;     
  fp32 chassis_relative_angle_set; 
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  
  fp32 vx_min_speed;  
  fp32 vy_max_speed; 
  fp32 vy_min_speed; 
  fp32 vz_max_speed;  //���ҷ�������ٶ� ��λm/s
  fp32 vz_min_speed;  //���ҷ�����С�ٶ� ��λm/s
  fp32 chassis_yaw;  
  fp32 chassis_pitch; 
  fp32 chassis_roll; 

  ramp_function_source_t classis_ramp;
  

} chassis_move_t;




static void chassis_jude_mode(chassis_move_t *chassis_move_update);
void chassis_power_control(chassis_move_t *chassis_power_control);
float TOP_Priority(void);
/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval          none
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
extern void omni_wheels_speed(float vx_set, float vy_set, float wz_set, float wheel_speed[4]);
void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);

extern  chassis_move_t *GetChassisData(void);

#endif
