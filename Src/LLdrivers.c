#include "stm32f3xx_hal.h"
#include "LLdrivers.h"


#include <string.h>
#include <stdlib.h>


//timers declared
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;


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
void setMotorCmd()
{
    // ship enable
    
    //rotation sense
    
    //speed rotation // as tested the tolerable speed ranges go from 
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
            
            printf("\n\rO : Offset %f\n\rP : KP %d \n\rI : KI %d\n\rD : KD %d\n\rS : Speed %d\n\r",
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


