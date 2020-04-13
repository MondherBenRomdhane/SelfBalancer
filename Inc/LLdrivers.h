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


#define ABS(X)     ((X>=0)?X:-X)
#define GYROSCOPE_DRIFT         40

//when the object is static we notice that the raw values are 
//around -550 these results in significant Drift! this vlue is to compensate that 
//after testing this is the value we got to nullify gyroscopic drift
#define X_AXIS_CALIB            620 

//this is the limit of wich the robot cannot recover from it's tilt better fall and hope for the best XD !!!
//the motors sould never operate past this limit
#define DEAD_ANGLE              24

//#define DEAD_ANGLE              45

#define BALANCE_RANGE           (0.2)

#define ALPHA                   (0.999)

#define ALPHA_CALIB             (0.9)

#define INTEGRATE_DELAY(X)     (X*0.000001)

#define ACTIVE_DELAY_MS            10 

#define ANGLE_OFFSET            (-6.4) // meaning at equilibrium point the sensor reading is 2.6

#define CALIBRATION_CYCLE        20

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
    float KP;
    float KI;
    float KD;
    int speed;
}ST_CommParam;



typedef struct{
    uint8_t * pu8_BufISR;
    //uint8_t  u16_SizeISR ;
}ST_UART1_ISR;


void init_PWMTimers(void);

float calibrateIMU(void);

float getAccelAngle(void);

void LLDriverCliMenu(uint8_t* Buf,ST_CommParam *stCommParam);

void calculatePID(ST_CommParam *receivedCMD,float CMD_Angle);

void setMotorCmd(ST_CommParam *receivedCMD);

void toggleAllLeds(char delay);

void AllLedSetState(GPIO_PinState STATE);

//void debugPrint(float arg_CMD_Angle, float speed);
void debugPrint(float arg_CMD_Angle, int speed);


//left motor functions
void enableLeftMDriver(E_State b_State);
void setLeftStepperMode(E_StepperMode E_Mode);
void setLeftStepperDir(E_Direction E_Dir);

//right motor functions
void enableRightMDriver(E_State b_State);
void setRightStepperMode(E_StepperMode E_Mode);
void setRightStepperDir(E_Direction E_Dir);

void setStepperMotorMode(E_StepperMode E_Mode);
void setStepperAngleDir(float argCMD_Angle);

void initialiseParam(ST_CommParam *stArgCommParam);

void stateManage(float arg_CMD_Angle, ST_CommParam *stArgCommParam);

void GanttDebug(char idx);

//osMailQDef (object_pool_qCMD, 4*sizeof(ST_CommParam), ST_CommParam);  // Declare mail queue
//osMailQId  (object_pool_q_idCMD);                 // Mail queue ID

#endif //LLDRIVERS_H

