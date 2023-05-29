#include "Vision_Task.h"
#include "my_pid.h"
VisionData_t VisionData;


static float PitchANglePID[3] = {13.5, 0, 0};
static float PitchSpeedPID[3] = {7500.0, 50, 50};
static float YawANglePID[3] = {12, 0.0, 0};
static float YawSpeedPID[3] = {6000.0, 50, 50};
void VisionPIDInit(void); 



 pid_type_def VisionPitchSpeedPID;
 gimbal_PID_t VisionPitchAnglePID, VisionYawAnglePID;
// pid_type_def VisionPitchAnglePID, VisionYawAnglePID;
 pid_type_def VisionYawSpeedPID;
void Vision_task(void const *pvParameters)
{
//	const TickType_t xFrequency = pdMS_TO_TICKS(20); // 设置任务执行周期为100ms
//	vTaskDelay(201);
	MY_UART_DMA_Init( );
	Vision_Kalman_Init();
	VisionPIDInit();
	while (1)
	{
		vision_send();
		AutoDataUpdate();
		vTaskDelay(2);
		
//	if(	LastPitchPrediction == VisionRecvData.pitch_angle || LastYawPrediction == VisionRecvData.yaw_angle)
//	{
//		time++;
//	}
//	else 
//	{
//		time = 0;
//	}

//	if(time >= 30)
//	{
//		VisionRecvData.pitch_angle = 0;
//		VisionRecvData.yaw_angle = 0;
//		time = 0;
//	}
//	LastPitchPrediction = VisionRecvData.pitch_angle;
//	LastYawPrediction = VisionRecvData.yaw_angle;	
		
	}
}
 
float SetYaw = 0, SetPitch = 0;

void AutoDataUpdate(void)
{


	VisionData.DataStruct = AimDataUpdate();

	VisionData.get_angle_point = get_INS_angle_point();
	VisionData.get_gyro_point  = get_gyro_data_point();
	VisionData.RcCtrl          = get_remote_control_point();
	VisionData.GetGimbalData = get_gimlal_data();
//	LoopQueueYaw(50, VisionData.DataStruct->yaw_angle);
//	LoopQueuePitch(50, VisionData.DataStruct->pitch_angle);
		VisionData.PredictionData.PredictionPitch = VisionData.DataStruct->pitch_angle * PI / 180.f;
		VisionData.PredictionData.PredictionYaw = VisionData.DataStruct->yaw_angle * PI / 180.0f;


	if(UseVision() && VisionData.DataStruct->IsFindTarget)
	{

		VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle_set = VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle - VisionData.PredictionData.PredictionPitch;
		VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle_set = VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle - VisionData.PredictionData.PredictionYaw;
//		VAL_LIMIT(&SetPitch, VisionData.GetGimbalData->gimbal_pitch_motor.min_relative_angle, VisionData.GetGimbalData->gimbal_pitch_motor.max_relative_angle);
	}
//	else 
//	{
//		VisionData.PredictionData.PredictionPitch = 0;
//		VisionData.PredictionData.PredictionYaw = 0;
//	}
	VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&VisionPitchAnglePID, VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle, VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle_set, VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro);
	// PID_calc(&VisionPitchAnglePID, VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle, SetPitch);
	PID_calc(&VisionPitchSpeedPID, VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro, VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro_set);
	// PID_calc(&VisionPitchSpeedPID, VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro, VisionPitchAnglePID.out);
	VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&VisionYawAnglePID, VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle, VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle_set, VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro);
	PID_calc(&VisionYawSpeedPID, VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro, VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro_set);
	VisionData.GetGimbalData->gimbal_pitch_motor.current_set = (int16_t)VisionPitchSpeedPID.out;
	VisionData.GetGimbalData->gimbal_yaw_motor.current_set = (int16_t)VisionYawSpeedPID.out;

//	gimbal_motor_absolute_angle_control(&VisionData.GetGimbalData->gimbal_pitch_motor);
//	gimbal_motor_absolute_angle_control(&VisionData.GetGimbalData->gimbal_yaw_motor);
	
	
}

void VisionPIDInit(void)
{
	gimbal_PID_init(&VisionPitchAnglePID, 15.0, 0, PitchANglePID[0], PitchANglePID[1], PitchANglePID[2]);
	PID_init(&VisionPitchSpeedPID, PID_POSITION, PitchSpeedPID, 30000.0f, 5000.0f);
	// PID_init(&VisionPitchAnglePID, PID_POSITION, PitchANglePID, 10.0f, 0.0f);
	gimbal_PID_init(&VisionYawAnglePID, 20.0, 0, YawANglePID[0], YawANglePID[1], YawANglePID[2]);
	PID_init(&VisionYawSpeedPID, PID_POSITION, YawSpeedPID, 30000.0f, 5000.0f);
	// PID_init(&VisionYawAnglePID, PID_POSITION, YawANglePID, 18.0f, 0.0f);
}



