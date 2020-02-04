#ifndef LLDRIVERS_H
#define LLDRIVERS_H

#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

#include <stdbool.h>

#define STR_DELIM   ';'
#define ENTER_ASCII 0x0D


#define OMEGA_MAX   3
#define STEPPER_MODE 2
//#define LINEARISE(X)  100/(200 * 0.00002)
#define ARR(X)  100/(200 *STEPPER_MODE* 0.00002 * OMEGA_MAX * X)

typedef enum
{
Enable,
Disable
}E_State;

typedef enum
{
Forward,
Backwards
}E_Direction;

typedef enum
{
Full_S,
Half_S,
Quarter_S,
Eigth_S,
Sixteenth_S
}E_StepperMode;


typedef struct 
{
    float angleOffset;
    int KP;
    int KI;
    int KD;
    int speed;
}ST_CommParam;



typedef struct{
    uint8_t * pu8_BufISR;
    //uint8_t  u16_SizeISR ;
}ST_UART1_ISR;


void init_PWMTimers(void);
void LLDriverCliMenu(uint8_t* Buf,ST_CommParam *stCommParam);

void calculatePID(ST_CommParam *receivedCMD,float CMD_Angle);

void setMotorCmd(ST_CommParam *receivedCMD);



//left motor functions
void enableLeftMDriver(E_State b_State);
void setLeftStepperMode(E_StepperMode E_Mode);
void setLeftStepperDir(E_Direction E_Dir);

//right motor functions
void enableRightMDriver(E_State b_State);
void setRightStepperMode(E_StepperMode E_Mode);
void setRightStepperDir(E_Direction E_Dir);

void setStepperMotorMode(E_StepperMode E_Mode);

//osMailQDef (object_pool_qCMD, 4*sizeof(ST_CommParam), ST_CommParam);  // Declare mail queue
//osMailQId  (object_pool_q_idCMD);                 // Mail queue ID

#endif //LLDRIVERS_H

