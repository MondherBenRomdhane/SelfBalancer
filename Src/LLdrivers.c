#include "stm32f3xx_hal.h"
#include "LLdrivers.h"


#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <stdbool.h>


#define ABS(X)     ((X>=0)?X:-X)
//#define MOTOR(N)   (54 * N + 60000)
//#define TF(N)       (long)trunc(1/(N^2 *0.00002))

bool b_DebugEnabled ;
bool b_Reeinitialise ;

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
double gotValue =1;
void setMotorCmd(ST_CommParam *receivedCMD)
{
    static int i=0;
    i++;
    //uint16_t = TF(receivedCMD->speed);
    if(receivedCMD)
    {
        //gotValue = TF(receivedCMD->speed);
        //gotValue*=receivedCMD->speed;
        //// ship enable
        //if (i==20)
        //{
        //    //printf("ARR%f , speed %d\r\n",gotValue,receivedCMD->speed);
        //    i=0;
        //}
        
        //rotation sense
        //__NOP();
        //speed rotation // as tested the tolerable speed ranges go from [0 to 1000]
        
        __HAL_TIM_SET_AUTORELOAD(&htim4, /*ABS(round(gotValue))*/ARR(receivedCMD->speed));//D15
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,__HAL_TIM_GET_AUTORELOAD(&htim4)/2);
        TIM4->EGR|=TIM_EGR_UG;
        //osDelay(1);
        __HAL_TIM_SET_AUTORELOAD(&htim16,/*ABS(round(gotValue))*/ARR(receivedCMD->speed));//b4
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
            
            printf("\n\rO : Offset %f\n\rD : KD %d \n\rI : KI %d\n\rP : KP %d\n\rS : Speed %d\n\r",
              stCommParam->angleOffset,
              stCommParam->KD,
              stCommParam->KI,
              stCommParam->KP,
              stCommParam->speed);
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
                    stCommParam->KP = atoi((const char *)Value+1);
                    printf("KP set OK %d\r\n",stCommParam->KP);
                    osDelay(10);
                }
                
                else if (Value[0]=='I')
                {
                    stCommParam->KI = atoi((const char *)Value+1);
                    printf("KI set OK %d\r\n",stCommParam->KI);
                    osDelay(10);
                }
                
                else if (Value[0]=='D')
                {
                    stCommParam->KD = atoi((const char *)Value+1);
                    printf("KD set OK %d\r\n",stCommParam->KD);
                    osDelay(10);
                }
                
                else if (Value[0]=='O')
                {
                    stCommParam->angleOffset = atoi((const char *)Value+1);
                    printf("angleOffset set OK %f\r\n",stCommParam->angleOffset);
                    osDelay(10);
                }
                
                else if (Value[0]=='S')
                {
                    stCommParam->speed = atoi((const char *)Value+1);
                    printf("speed set OK %d\r\n",stCommParam->speed);
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
    if (receivedCMD)
    {
        static float previousError = 0; //(used by derivative control)
        const char setPoint = 0; //radians, set by the user
        //float sensorValue = 0; //radians, from the IMU
        //float speed; //output of calculatePID()
        
        
        static float proportional, derivative, integral;
        static float error =0;
        error = ABS(sensorValue )- setPoint;
        
        proportional = error;
        //integral += error * 0.0001f; 
        //derivative = (error - previousError)/0.0001f;
        //previousError = error;
        receivedCMD->speed = (proportional*receivedCMD->KP) + (integral*receivedCMD->KI) + (derivative*receivedCMD->KD);
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

//void SDebug(char* string)
//{
//    if(string)
//    {
//        static uint8_t counter = 0;
//        
//        if 
//    }
//
//}


