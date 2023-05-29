#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_rng.h"
#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define get_motor_measure(ptr, data)                                 \
  {                                                                  \
    (ptr)->last_ecd = (ptr)->ecd;                                    \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);             \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);       \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);   \
    (ptr)->temperate = (data)[6];                                    \
    if ((ptr)->ecd - (ptr)->last_ecd > 4096)                         \
      (ptr)->round_cnt--;                                            \
    else if ((ptr)->ecd - (ptr)->last_ecd < -4096)                   \
      (ptr)->round_cnt++;                                            \
    (ptr)->total_angle = ((ptr)->round_cnt + 1) * 8192 + (ptr)->ecd; \
  }

#define get_board_info(ptr, data)    \
  {                                  \
    CharUFloat.byte[0] = (data)[0];  \
    CharUFloat.byte[1] = (data)[1];  \
    CharUFloat.byte[0] = (data)[3];  \
    CharUFloat.byte[1] = (data)[4];  \
    (ptr)->INFO = CharUFloat.f_data; \
  }

motor_measure_t motor_chassis[6];

motor_measure_t motor_shoot[3];


CharUFloat_t CharUFloat; 
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
  if (hcan->Instance == CAN1)
  {
    switch (rx_header.StdId)
    {
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    case CAN_YAW_MOTOR_ID:
    case CAN_PIT_MOTOR_ID:
    {
      static uint8_t i = 0;
      i = rx_header.StdId - CAN_3508_M1_ID;
      get_motor_measure(&motor_chassis[i], rx_data);
      detect_hook(CHASSIS_MOTOR1_TOE + i);
      break;
    }

    default:
    {
      break;
    }
    }
  }
  else
  {
    switch (rx_header.StdId)
    {
    case CAN_FIRE_LEFT_ID:
      get_motor_measure(&motor_shoot[0], rx_data);
      break;
    case CAN_FIRE_RIGHT_ID:
      get_motor_measure(&motor_shoot[1], rx_data);
      break;
    case CAN_TRIGGER_MOTOR_ID:
      get_motor_measure(&motor_shoot[2], rx_data);
      break;

    default:
      break;
    }
  }
}


void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t NO, int16_t rev)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = (NO >> 8);
  gimbal_can_send_data[5] = NO;
  gimbal_can_send_data[6] = (rev >> 8);
  gimbal_can_send_data[7] = rev;
  HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


void CAN_cmd_chassis_reset_ID(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_shoot(int16_t shoot, int16_t left, int16_t right, int16_t rev)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x1ff;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = shoot >> 8;
  chassis_can_send_data[1] = shoot;
  chassis_can_send_data[2] = left >> 8;
  chassis_can_send_data[3] = left;
  chassis_can_send_data[4] = right >> 8;
  chassis_can_send_data[5] = right;
  chassis_can_send_data[6] = rev >> 8;
  chassis_can_send_data[7] = rev;

  HAL_CAN_AddTxMessage(&SHOOT_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_shoot2(int16_t shoot, int16_t left, int16_t right, int16_t rev)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x200;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = shoot >> 8;
  chassis_can_send_data[1] = shoot;
  chassis_can_send_data[2] = left >> 8;
  chassis_can_send_data[3] = left;
  chassis_can_send_data[4] = right >> 8;
  chassis_can_send_data[5] = right;
  chassis_can_send_data[6] = rev >> 8;
  chassis_can_send_data[7] = rev;

  HAL_CAN_AddTxMessage(&SHOOT_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_chassis[4];
}


const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_chassis[5];
}


const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}

const motor_measure_t *get_shoot_trigger_measure_point(void)
{
  return &motor_shoot[2];
}

const motor_measure_t *get_shoot_fire_left_point(void)
{
  return &motor_shoot[0];
}

const motor_measure_t *get_shoot_fire_right_point(void)
{
  return &motor_shoot[1];
}
