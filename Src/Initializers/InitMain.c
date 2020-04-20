//
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * This notice applies to any and all portions of this file
//  * that are not between comment pairs USER CODE BEGIN and
//  * USER CODE END. Other portions of this file, whether 
//  * inserted by the user or by software development tools
//  * are owned by their respective copyright owners.
//  *
//  * Copyright (c) 2019 STMicroelectronics International N.V. 
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without 
//  * modification, are permitted, provided that the following conditions are met:
//  *
//  * 1. Redistribution of source code must retain the above copyright notice, 
//  *    this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright notice,
//  *    this list of conditions and the following disclaimer in the documentation
//  *    and/or other materials provided with the distribution.
//  * 3. Neither the name of STMicroelectronics nor the names of other 
//  *    contributors to this software may be used to endorse or promote products 
//  *    derived from this software without specific written permission.
//  * 4. This software, including modifications and/or derivative works of this 
//  *    software, must execute solely and exclusively on microcontroller or
//  *    microprocessor devices manufactured by or for STMicroelectronics.
//  * 5. Redistribution and use of this software other than as permitted under 
//  *    this license is void and will automatically terminate your rights under 
//  *    this license. 
//  *
//  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
//  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
//  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
//  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
//  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
//  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
//  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *
//  ******************************************************************************
//  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "stm32f3_discovery.h"
#include "stm32f3_discovery_accelerometer.h"
#include "stm32f3_discovery_gyroscope.h"
#include "usbd_cdc_if.h"
#include "LLdrivers.h"
#include <math.h>
#include <stdbool.h>

//static double globAccelangle = 0;

//
//
///* USER CODE END Includes */
//
///* Private variables ---------------------------------------------------------*/
//I2C_HandleTypeDef hi2c1;
//
//SPI_HandleTypeDef hspi1;
//
//
//TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef htim4;
//
//UART_HandleTypeDef huart4;
//
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
//
///* USER CODE BEGIN PV */
///* Private variables ---------------------------------------------------------*/
//
///* USER CODE END PV */
//

//
///* USER CODE BEGIN PFP */
///* Private function prototypes -----------------------------------------------*/
//
///* USER CODE END PFP */
//
///* USER CODE BEGIN 0 */

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
//extern uint8_t Buf[50];
uint8_t DataBuf[20];
uint8_t RXBuf[20] ;

osThreadId PrintserialID;
osThreadId defaultTaskID;

float Alpha = ALPHA;
float GyroDrift = GYROSCOPE_DRIFT;

    typedef struct {
      double D_Angle;
    } properties_t;


    osMailQDef (object_pool_qCMD, 2, ST_CommParam);  // Declare mail queue
    osMailQId  (object_pool_q_idCMD);                 // Mail queue ID
    
    volatile int16_t xval, yval ,zval= 0x00; // accel val
    
    float yaw = 0;
    float AVGYaw = 0;
    
    extern bool b_DebugEnabled ;
    extern bool b_Reeinitialise;
    volatile float Xval,Yval,Yval1,Zval = 0x00; //gyro val
    
    ST_CommParam stCurrentState;
    float CMD_Angle=0;
    float Angle;
//    float Ref_ACCELAngle;

    bool enableMotors;
///* USER CODE END 0 */

////add Timer 4 and timer 15 for square PWM command for the drivers of Steppers

////Timer4  channel4 -> PD15 stepper
////Timer15 channel1 -> PB4  stepper
////Timer16 channel1 -> PF9  servo
////Timer17 channel1 -> PB5  servo

//extern TIM_HandleTypeDef htim4;

float imuVAL =0;
void AngleCalcTask(void const * argument)
{
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    init_PWMTimers();
    
    int16_t buffer[3] = {0};
    float Buffer[3];
    
    //this variable allows that the gyroscope value could be updated from the accelerometer
    bool b_GyroInit = true;
    bool b_GyroCalib = true;
    extern long CalibGyrovalue ;
    //this variable prevents the gyroscopic drift
    uint8_t samplingCounter = 0;
    
    imuVAL = calibrateIMU();
    
    osDelay(10);//this is a time bomb! the queue pointer is null if this task starts first that's why we delay XD !
    
    //TickType_t xLastWakeTime;
    //xLastWakeTime = osKernelSysTick();
    
    //osThreadTerminate(PrintserialID);
    //osThreadTerminate(defaultTaskID);
    for(;;)
    {
        GanttDebug(3);
        
        yaw = getAccelAngle();
        // gyro & final angle read////////////////////////////////////
        BSP_GYRO_GetXYZ(Buffer);

        Xval = Buffer[0];
        Yval = Buffer[1];
        Zval = Buffer[2];
        
        if (b_GyroCalib == true)
        {
            Angle = imuVAL;
            b_GyroCalib = false;
        }
        
        //highpass filter (Manual :( !)
        if (ABS(Xval)<1000)
        { 
            // filter values lower than 1000 wich are basically just noise
            Xval = 0;
        }
        //else
        //{
        //    // because noise is still inside the values
        //    Xval = Xval - CalibGyrovalue;
        //}
        AVGYaw = AVG(yaw);
        //Angle = Alpha*((Xval-GyroDrift) * 0.00001 + Angle)+(1-Alpha)*trunc(yaw);
        Angle = Alpha*((Xval-GyroDrift) * 0.00001 + Angle)+(1-Alpha)*AVGYaw;
        //Angle = (Xval) * 0.00001 + Angle;
        
        ////begin TASK2//////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        ////offset for the angle
        CMD_Angle = Angle - stCurrentState.angleOffset - GyroDrift;
        //Ref_ACCELAngle = yaw - stCurrentState.angleOffset;
        
        //setStepperAngleDir(CMD_Angle);
        
        if (b_Reeinitialise == false)
        {
            //magic!!
            calculatePID(&stCurrentState,CMD_Angle);
        }
        
        //this function is responsible for the handling of the balance state and the fail  state case
        stateManage(ABS(CMD_Angle), &stCurrentState);
        
        setMotorCmd(&stCurrentState);
        
        //osDelay(ACTIVE_DELAY_MS);
        osDelay(10);
        //HAL_Delay(9);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
        
        //end TASK2 //////////////////////////////////////////////////////////////////////////////////////////////////////
    }
}
unsigned char bfr[10] ;
void printSerial(void const * argument)
{
    
    HAL_UART_Receive_IT(&huart3,bfr,sizeof(bfr));
    PrintserialID = osThreadGetId();
    
    for(;;)
    {
        GanttDebug(1);
#if (DEBUG_VAL == DEBUG_SPEED)
        debugPrint(CMD_Angle, stCurrentState.speed);
#elif (DEBUG_VAL == DEBUG_ACCEL)
        debugPrint(CMD_Angle, yaw);
#endif
        
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_14);
        //HAL_UART_Receive(&huart3,bfr,8,100);
        // printf("--> got :<%s>",bfr);
        
        osDelay(10);
        //setMotorCmd(&stCurrentState);
        
    }
}

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
    defaultTaskID = osThreadGetId();
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    enableLeftMDriver(Enable);
    enableRightMDriver(Enable);
    
    setStepperMotorMode(Quarter_S);
    
    //initialize PID & State machine VAR
    initialiseParam(&stCurrentState);
    HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
    
    /* Infinite loop */
    for(;;)
    {
        GanttDebug(2);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_10);//Orange
        LLDriverCliMenu(DataBuf,&stCurrentState);
        //setMotorCmd(&stCurrentState);
        osDelay(100);
    }
}

int i=0;
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
    //HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
    i=strlen((const char *)RXBuf);
    if ((RXBuf[i-1]== 0x0D)||(RXBuf[i-1]== 0x0A))//carriage return or line feed
    {
        HAL_UART_RxCpltCallback(&huart1);
        HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
    }
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
    //HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
    //i=strlen((const char *)RXBuf);
    //if ((RXBuf[i-1]== 0x0D)||(RXBuf[i-1]== 0x0A))//carriage return or line feed
    {
        HAL_UART_RxCpltCallback(&huart3);
        HAL_UART_Receive_IT(&huart2,bfr,sizeof(bfr));
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart==&huart1)
    {
        memcpy(DataBuf,RXBuf,sizeof(DataBuf));
        //printf("HAL_UART_RxCpltCallback\r\n");
        HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
        //printf("DB%s\r\n",DataBuf);
        memset(RXBuf,'\0',sizeof(RXBuf));
        huart->RxState = HAL_UART_STATE_READY;
    }
    else if (huart==&huart3)
    {
    
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart==&huart1)
    {
        //errors could be overflow or overrun
        __NOP();
        // printf("HAL_UART_ErrorCallback\r\n");
        // printf("DB:%s\r\n",DataBuf);
        //clear the data
        memset(RXBuf,'\0',sizeof(RXBuf));
        HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
    }
    else if (huart==&huart3)
    {
        
    }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
