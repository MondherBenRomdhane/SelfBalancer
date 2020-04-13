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
//extern uint8_t Buf[50];
uint8_t DataBuf[20];
uint8_t RXBuf[20] ;
uint8_t RXCounter = 0;

    typedef struct {
      double D_Angle;
    } properties_t;


    osMailQDef (object_pool_qCMD, 2, ST_CommParam);  // Declare mail queue
    osMailQId  (object_pool_q_idCMD);                 // Mail queue ID
    
    volatile int16_t xval, yval ,zval= 0x00; // accel val
    
    
    
    float yaw = 0;
    
    extern bool b_DebugEnabled ;
    extern bool b_Reeinitialise;
    volatile float Xval,Yval,Yval1,Zval = 0x00; //gyro val
    
    ST_CommParam stCurrentState;
    float CMD_Angle=0;
    float Angle;

    bool enableMotors;
///* USER CODE END 0 */

////add Timer 4 and timer 15 for square PWM command for the drivers of Steppers

////Timer4  channel4 -> PD15 stepper
////Timer15 channel1 -> PB4  stepper
////Timer16 channel1 -> PF9  servo
////Timer17 channel1 -> PB5  servo

//extern TIM_HandleTypeDef htim4;
//void MotorCmdTask(void const * argument)
//{
//    init_PWMTimers();
//    
//    //HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);  // 
//    //HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_RESET); //TO DO PF9 ship enable is defective !!!!! is a ctually the pin of the tim 15 !!!!!
//    //
//    //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);  // ship enable is low
//    //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
//    float CMD_Angle=0;
//    char isBalanced ;
//    for(;;)
//    {
//        isBalanced = ' ';
//        osEvent event = osMailGet(object_pool_q_id , osWaitForever);
//        properties_t *received = (properties_t *)event.value.p;// ".p" indic ates that the message is a pointer
//        
//        osEvent eventCMD = osMailGet(object_pool_q_idCMD , 201);
//        ST_CommParam *receivedCMD = (ST_CommParam *)eventCMD.value.p;
//        
//        //offset for the angle
//        CMD_Angle = received->D_Angle - receivedCMD->angleOffset;
//        
//        if (b_Reeinitialise == false)
//        {
//            if (CMD_Angle>0)
//            {
//                setLeftStepperDir(Forward);
//                setRightStepperDir(Forward);
//            }
//            else
//            {
//                setLeftStepperDir(Backwards);
//                setRightStepperDir(Backwards);
//            }
//            
//            HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
//            
//            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
//            
//            //magic!!
//            ////////////////////////////////////////////////////
//            calculatePID(receivedCMD,CMD_Angle);
//            ////////////////////////////////////////////////////
//            
//            osMailFree(object_pool_q_id, received);
//            osMailFree(object_pool_q_idCMD, receivedCMD);
//            osDelay(100);
//        }
//        
//        if (ABS(CMD_Angle) > DEAD_ANGLE ) //outside the balance range
//        {
//            b_Reeinitialise = true; // to wait until put back at the balancing point
//            receivedCMD->speed=0; //overriding the pid value
//        }
//        
//        if (ABS(CMD_Angle) < BALANCE_RANGE ) // at equilibrium point
//        {
//            b_Reeinitialise = false; // start the balancing process only when put at equilibrium point
//            receivedCMD->speed=0; //overriding the pid value
//            isBalanced = '*';
//        }
//        else
//        {
//            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
//        }
//        
//        if (b_DebugEnabled==true)
//        {
//            //do not remove this line it is for debug purposes !!
//            printf("%06.2f;%d;\n\r",CMD_Angle,receivedCMD->speed/*,isBalanced*/); //leading zeros for the sign serial print the output is on 8
//        }
//        //else //while debuging motors should never run !!!
//        //{
//          //receivedCMD->speed*=0.01;
//          setMotorCmd(receivedCMD);
//        //}
//        
//        
//    }
//}
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
    //this variable prevents the gyroscopic drift
    uint8_t samplingCounter = 0;
    
    imuVAL = calibrateIMU();
    
    osDelay(10);//this is a time bomb! the queue pointer is null if this task starts first that's why we delay XD !
    
    //TickType_t xLastWakeTime;
    //xLastWakeTime = osKernelSysTick();
    
    
    for(;;)
    {
        GanttDebug(3);
        //read accelero Values///////////////////////////////////////
        //BSP_ACCELERO_GetXYZ(buffer);
        //xval = buffer[0];
        //yval = buffer[1];
        //zval = buffer[2];
        //
        //xAcc=(double)xval/16384;
        //yAcc=(double)yval/16384;
        //zAcc=(double)zval/16384;
        //
        //yaw = atan(zAcc/sqrt(xAcc*xAcc+yAcc*yAcc))*57.32;
        yaw = getAccelAngle();
        // gyro & final angle read////////////////////////////////////
        BSP_GYRO_GetXYZ(Buffer);

        Xval = Buffer[0];
        Yval = Buffer[1];
        Zval = Buffer[2];

        // this is poorly written !!
        //this code lets the angle being read the first time only from the accelerometer assuming that the 
        //robot is not moved at startup time
        //there should be instead a calibration period in wich the a led blinks and the angle is 
        //calculated from the RMS of the accel angle
        //add also the auto couter-action for the gyro drift !!!
        //if (b_GyroInit == true)
        //{
        //    //angle = yaw;
        //    b_GyroInit = false;
        //    if (b_GyroCalib == true)
        //    {
        //        Angle = yaw;
        //        b_GyroCalib = false;
        //    }
        //}
        //
        //samplingCounter++;
        //if (samplingCounter>GYROSCOPE_DRIFT)
        //{
        //    samplingCounter=0;
        //    b_GyroInit = true;
        //}

        //Angle = ALPHA*((Xval+X_AXIS_CALIB) *0.0001+Angle)+(1-ALPHA)*yaw; //0.0001 for 100ms!!
        //Angle = ALPHA*((Xval+X_AXIS_CALIB) * INTEGRATE_DELAY(ACTIVE_DELAY_MS) + Angle)+(1-ALPHA)*yaw;
        if (b_GyroCalib == true)
        {
            Angle = imuVAL;
            b_GyroCalib = false;
        }
        
        Angle = ALPHA*((Xval/*+X_AXIS_CALIB*/) * 0.00001 + Angle)+(1-ALPHA)*yaw;
        //Angle = (Xval) * 0.00001 + Angle;
        
        ////begin TASK2//////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        ////offset for the angle
        CMD_Angle = Angle - stCurrentState.angleOffset;
        setStepperAngleDir(CMD_Angle);
        
        if (b_Reeinitialise == false)
        {
            //magic!!
            calculatePID(&stCurrentState,CMD_Angle);
        }
        
        //this function is responsible for the handling of the balance state and the fail  state case
        stateManage(ABS(CMD_Angle), &stCurrentState);
        
        //setMotorCmd(&stCurrentState);
        
        //osDelay(ACTIVE_DELAY_MS);
        osDelay(10);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
        
        //end TASK2 //////////////////////////////////////////////////////////////////////////////////////////////////////
    }
}

void printSerial(void const * argument)
{
    for(;;)
    {
        GanttDebug(1);
        debugPrint(CMD_Angle, stCurrentState.speed);
        //debugPrint(yaw, stCurrentState.speed);
        
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_14);
        osDelay(10);
        setMotorCmd(&stCurrentState);
        
    }
}

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    memcpy(DataBuf,RXBuf,sizeof(DataBuf));
    //printf("HAL_UART_RxCpltCallback\r\n");
    HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
    //printf("DB%s\r\n",DataBuf);
    RXCounter=0;
    memset(RXBuf,'\0',sizeof(RXBuf));
    huart->RxState = HAL_UART_STATE_READY;
    
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    //errors could be overflow or overrun
    __NOP();
   // printf("HAL_UART_ErrorCallback\r\n");
   // printf("DB:%s\r\n",DataBuf);
    //clear the data
    memset(RXBuf,'\0',sizeof(RXBuf));
    HAL_UART_Receive_IT(&huart1,RXBuf,sizeof(RXBuf));
    //RXCounter=0;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
