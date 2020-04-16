#include "stm32f3xx_hal.h"
#include "LLdrivers.h"


#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <stdbool.h>

#include "stm32f3_discovery_accelerometer.h"
#include "stm32f3_discovery_gyroscope.h"


#define ABS(X)     ((X>=0)?X:-X)
//#define MOTOR(N)   (54 * N + 60000)
//#define TF(N)       (long)trunc(1/(N^2 *0.00002))

bool b_DebugEnabled ;
bool b_Reeinitialise ;

extern float Alpha;

extern char enableMotors;
//timers declared
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;


double TF(double N)
{
    return (1/(N*N *0.00002));
    //return (1/(N*N *0.0016));
}

void init_PWMTimers()
{
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
}

//this function manages the abstract motor cmd
//ship enable
//rototion sense
//speed throuough PWM square timer

void setMotorCmd(ST_CommParam *receivedCMD)
{
    //in the case of implementing a transfert funcion it should not be here !!!!
    //uint16_t = TF(receivedCMD->speed);
    
    if(enableMotors==false) //overrite whaterver value the speed is by 0
    {
        receivedCMD->speed = 0;
    }
    
    if(receivedCMD)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim4, ARR(receivedCMD->speed));//D15
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,__HAL_TIM_GET_AUTORELOAD(&htim4)/2);
        TIM4->EGR|=TIM_EGR_UG;
        
        __HAL_TIM_SET_AUTORELOAD(&htim16,ARR(receivedCMD->speed));//b4
        __HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,__HAL_TIM_GET_AUTORELOAD(&htim16)/2);
        TIM16->EGR|=TIM_EGR_UG;
    }
}

// to do add the return typedef struct
void LLDriverCliMenu(uint8_t* Buf,ST_CommParam *stCommParam)
{
    if (Buf[0]=='T')
    {
        char TBuf[]="D19;P3800;I40;O0;S19;";
        memcpy(Buf,TBuf,sizeof(TBuf));
        printf("test sequence !!\n\r");
        osDelay(10);
    }
    
    //check if it is a frame input or a shor command input
    if (strlen((char*)Buf)<=3)
    {
       if (Buf[0]=='?')
        {
            //printf("\n\rO : Offset\n\rP : KP\n\rI : KI\n\rD : KD\n\rS : Speed\n\r");
            
            printf("\n\rO : Offset %f\n\rD : KD %f \n\rI : KI %f\n\rP : KP %f\n\rS : Speed %d\n\r : Alpha %f\r\n",
              stCommParam->angleOffset,
              stCommParam->KD,
              stCommParam->KI,
              stCommParam->KP,
              stCommParam->speed,
              Alpha
            );
            osDelay(10);
        }
        
        else if (Buf[0] == ENTER_ASCII)
        {
            printf("\n\rCMD> ");
            osDelay(10);
        }
        else if(Buf[0] == 'd')
        {
            printf("\n\rDebug ON !!! ");
            b_DebugEnabled = true;
            osDelay(10);
        }
        
        else if(Buf[0] == '-')
        {
            printf("\n\rDebug OFF !!! ");
            b_DebugEnabled = false;
            osDelay(10);
        }
        //else //could only be used in inteerrupt mode
        //{
        //    printf("%d\r\n",strlen((const char *)Buf));
        //}
    }
    //set command
    else
    {
        char Value[10]="";
        char* pBuf = (char*)Buf;
        char i;
        
        for (i=0;i<strlen((char*)Buf)-1;i++)
        {
            if (pBuf[i] == STR_DELIM)
            {
                memcpy(Value,pBuf,i);
                pBuf=&pBuf[i]+1;
                
                if (Value[0]=='P')
                {
                    stCommParam->KP = atof((const char *)Value+1);
                    printf("KP set OK %f\r\n",stCommParam->KP);
                    osDelay(10);
                }
                
                else if (Value[0]=='I')
                {
                    stCommParam->KI = atof((const char *)Value+1);
                    printf("KI set OK %f\r\n",stCommParam->KI);
                    osDelay(10);
                }
                
                else if (Value[0]=='D')
                {
                    stCommParam->KD = atof((const char *)Value+1);
                    printf("KD set OK %f\r\n",stCommParam->KD);
                    osDelay(10);
                }
                
                else if (Value[0]=='O')
                {
                    stCommParam->angleOffset = atof((const char *)Value+1);
                    printf("angleOffset set OK %f\r\n",stCommParam->angleOffset);
                    osDelay(10);
                }
                
                else if (Value[0]=='S')
                {
                    stCommParam->speed = atoi((const char *)Value+1);
                    //printf("speed set OK %d\r\n",stCommParam->speed);
                    osDelay(10);
                }
                else if (Value[0]=='A')
                {
                    Alpha = atof((const char *)Value+1);
                    printf("Alpha set OK %f\r\n", Alpha);
                    osDelay(10);
                }
                else if (Value[0]=='M')
                {
                    char tempvar;
                    tempvar = atoi((const char *)Value+1);
                    if (tempvar == 1)
                    {
                        enableMotors = true;
                    }
                    else if(tempvar == 0)
                    {
                        enableMotors = false;
                    }
                    else
                    {
                        printf("KO! \r\n");
                        osDelay(10);
                    }
                    //printf("speed set OK %d\r\n",stCommParam->speed);
                    osDelay(10);
                }
                else
                {
                    printf("KO! \r\n");
                    osDelay(10);
                }
                //printf("\n\rCMD>\n\r");
                i=0;
            }
            memset(Value,'\0',strlen(Value));
        }
    }

    //flush buffer
    memset(Buf,'\0',strlen((char*)Buf));
    osDelay(10);
    //*Buf='\0';

}

void calculatePID(ST_CommParam *receivedCMD ,float sensorValue) 
{
    //#define PID_SCALER 0.01
    #define PID_SCALER 1
    
    if (receivedCMD)
    {
        static float previousError = 0; //(used by derivative control)
        const char setPoint = 0; //radians, set by the user
        static float proportional, derivative, integral;
        static float error =0;
        
        error = ABS(sensorValue )- setPoint;
        
        proportional = error;
        integral += error * 0.00001f; 
        derivative = (error - previousError);///0.0001f;
        previousError = error;
        receivedCMD->speed = (proportional*receivedCMD->KP) + (integral*receivedCMD->KI*(PID_SCALER)) + (derivative*receivedCMD->KD*(PID_SCALER));
    }
}

void enableLeftMDriver(E_State b_State)
{
    if (b_State == Enable)
    {
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);
    }
}

void enableRightMDriver(E_State b_State)
{
    if (b_State == Enable)
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);// check if GPIO is defined !!!!
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
    }
}

//MS1------MS2------MS3------MicrostepResolution
//Low------Low------Low------Full step
//High-----Low------Low------Half step
//Low------High-----Low------Quarter step
//High-----High-----Low------Eighth step
//High-----High-----High-----Sixteenth step

void setLeftStepperMode(E_StepperMode E_Mode)
{ 
    //ms1 --> PF1
    //ms2 --> PC15
    //ms3 --> PC14
    switch(E_Mode)
    {
        case(Full_S):
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET); //--->
            break;
        
        case(Half_S):
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET);   //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET); //--->
            break;
            
        case(Quarter_S):
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);   //--->
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET); //--->
            break;
        default:
            break;
    }
    
}

void setRightStepperMode(E_StepperMode E_Mode)
{
    //ms1 --> PF2
    //ms2 --> PA1
    //ms3 --> PC0
    switch(E_Mode)
    {
        case(Full_S):
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET); //--->
            break;
        
        case(Half_S):
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET);   //--->
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET); //--->
            break;
            
        case(Quarter_S):
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET); //--->
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);   //--->
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET); //--->
            break;
        default:
            break;
    }
}

void initialiseParam(ST_CommParam *stArgCommParam)
{
    if (stArgCommParam)
    {
        stArgCommParam->angleOffset=ANGLE_OFFSET; //this is a mechanical constraint XD !
        stArgCommParam->KD =0;
        stArgCommParam->KI =0;
        stArgCommParam->KP =20 ;
        stArgCommParam->speed =0;
        b_DebugEnabled = false;
        b_Reeinitialise = true;
        enableMotors = true;
    }
    else
    {
        Error_Handler();
    }
}

//extern float Ref_ACCELAngle;
//extern float Angle;

void stateManage(float arg_CMD_Angle, ST_CommParam *stArgCommParam)
{
    if (stArgCommParam)
    {
        //outside the balance range
        if (arg_CMD_Angle > DEAD_ANGLE ) 
        {
            b_Reeinitialise = true; // to wait until put back at the balancing point
            stArgCommParam->speed=0; //overriding the pid value
        }
        
        // at equilibrium point
        if (arg_CMD_Angle < BALANCE_RANGE )
        {
            b_Reeinitialise = false; // start the balancing process only when put at equilibrium point
            stArgCommParam->speed=0; //overriding the pid value
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);
            
            //this is palying with fire !!
            //Angle = Ref_ACCELAngle;
        }
        else
        {
            //what the ??? (is the led working to indicate that the rob is at equilibrium pt.)
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
        }
    }
    else
    {
        Error_Handler();
    }
    
}

#if (DEBUG_VAL == DEBUG_SPEED)
void debugPrint(float arg_CMD_Angle, int speed)
#elif (DEBUG_VAL == DEBUG_ACCEL)
 void debugPrint(float arg_CMD_Angle, float speed)
#endif
{
    if (b_DebugEnabled==true)
        {
            //do not remove this line it is for debug purposes !!
            //printf("%06.2f;%06.2f;\n\r",arg_CMD_Angle,speed); //leading zeros for the sign serial print the output is on 8
#if (DEBUG_VAL == DEBUG_SPEED)
            printf("%06.2f;%d;\n\r",arg_CMD_Angle,speed); //leading zeros for the sign serial print the output is on 8
#elif (DEBUG_VAL == DEBUG_ACCEL)
            printf("%06.2f;%06.2f;\n\r",arg_CMD_Angle,speed); //leading zeros for the sign serial print the output is on 8
#endif
        }
}

void setStepperMotorMode(E_StepperMode E_Mode)
{
    setLeftStepperMode(E_Mode);
    setRightStepperMode(E_Mode);
}

void setLeftStepperDir(E_Direction E_Dir)
{
    if (E_Dir==Forward)
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
    }
}

void setRightStepperDir(E_Direction E_Dir)
{
    if (E_Dir==Forward)
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
    }
}


void setStepperAngleDir(float argCMD_Angle)
{
    if (argCMD_Angle>0)
    {
        setLeftStepperDir(Forward);
        setRightStepperDir(Forward);
    }
    else
    {
        setLeftStepperDir(Backwards);
        setRightStepperDir(Backwards);
    }
}

void GanttDebug(char idx)
{
    //B11/13/15
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
    
    if (idx == 1)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
    }
    else if (idx == 2)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
    }
    else if (idx == 3)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
    }
}

    //float Buffer[1];
    //float CalibYaw = 90;
    //float calibAngle = 0;

float calibrateIMU(void)
{
    float Buffer[1];
    float CalibYaw = 90;
    float calibAngle = 0;
    char idx = 0;

    //for(char i = 0 ; i<CALIBRATION_CYCLE ; i++)
    while ((ABS(CalibYaw)-ABS(calibAngle)>0.5)||(idx<20))
    //while (1)
    {
        idx++;
        CalibYaw = getAccelAngle();
        
        BSP_GYRO_GetXYZ(Buffer);
        
        //calibAngle = (Buffer[0]) * 0.0001 + calibAngle; //pure gyro value 
        calibAngle = ALPHA_CALIB*((Buffer[0]) * 0.00001 + calibAngle)+(1-ALPHA_CALIB)*CalibYaw;
        
        osDelay(10);
        toggleAllLeds(0);
        //debugPrint(calibAngle,CalibYaw);
    }
    //calculating gyroscopic drift
    //return(calibAngle/CALIBRATION_CYCLE);
    AllLedSetState(GPIO_PIN_RESET);
    return(calibAngle);
}

float getAccelAngle(void)
{
    int16_t buffer[3] = {0};
    double xAcc , yAcc, zAcc = 0;
    
    BSP_ACCELERO_GetXYZ(buffer);
    
    xAcc=(double)buffer[0]/16384;
    yAcc=(double)buffer[1]/16384;
    zAcc=(double)buffer[2]/16384;

    return (atan(zAcc/sqrt(xAcc*xAcc+yAcc*yAcc))*57.32);
}

void toggleAllLeds(char delay)
{
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_15);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_14);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_13);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_12);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_10);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_9);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
        HAL_Delay(delay);
}

void AllLedSetState(GPIO_PinState STATE)
{
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9 ,STATE);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8 ,STATE);
}


