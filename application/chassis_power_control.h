#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "chassis_task.h"
#include "main.h"
#include "remote_control.h"

#define AccAble KEY_PRESSED_OFFSET_F

typedef struct
{
    float Limit_k;
    float Real_PowerBuffer;
    float Max_PowerBuffer;
    float CHAS_TotalOutput;
    float CHAS_LimitOutput;
} CHASSIS_PowerLimit_t;

typedef struct
{
    const motor_measure_t *ChassisMotorData[4];
    const RC_ctrl_t* RCData;
    CHASSIS_PowerLimit_t CHASSIS_PowerLimit;
    chassis_move_t *ChassisData;

}ChassisPowerControl_t;




void Chassis_VAL_LIMIT(int X);

extern void ChassisPowerLimit(void);
extern void PowerLimit(void);


#endif
