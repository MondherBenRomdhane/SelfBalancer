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
#define GYROSCOPE_DRIFT         40
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

    typedef struct {
      double D_Angle;
    } properties_t;

    osMailQDef (object_pool_q, 2, properties_t);  // Declare mail queue
    osMailQId  (object_pool_q_id);                 // Mail queue ID
    
    osMailQDef (object_pool_qCMD, 2, ST_CommParam);  // Declare mail queue
    osMailQId  (object_pool_q_idCMD);                 // Mail queue ID
    
    osSemaphoreDef (my_semaphore);    // Declare semaphore
    osSemaphoreId  (my_semaphore_id);
    
    volatile int16_t xval, yval ,zval= 0x00; // accel val
    
    double xAcc , yAcc, zAcc = 0;
    
    float yaw = 0;
    
    
    volatile float Xval,Yval,Yval1,Zval = 0x00; //gyro val

///* USER CODE END 0 */

////add Timer 4 and timer 15 for square PWM command for the drivers of Steppers

////Timer4  channel4 -> PD15 stepper
////Timer15 channel1 -> PB4  stepper
////Timer16 channel1 -> PF9  servo
////Timer17 channel1 -> PB5  servo

//extern TIM_HandleTypeDef htim4;
void MotorCmdTask(void const * argument)
{
    //todo not yet tested !!!!
    init_PWMTimers();
    
    //HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
    //HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
    //HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
    //HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
    
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);  // 
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_RESET); //TO DO PF9 ship enable is defective !!!!! is a ctually the pin of the tim 15 !!!!!
    
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);  // ship enable is low
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
    
    for(;;)
    {
        osEvent event = osMailGet(object_pool_q_id , osWaitForever);
        properties_t *received = (properties_t *)event.value.p;// ".p" indic ates that the message is a pointer
        
        osEvent eventCMD = osMailGet(object_pool_q_idCMD , osWaitForever);
        ST_CommParam *receivedCMD = (ST_CommParam *)eventCMD.value.p;

        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);

        __HAL_TIM_SET_AUTORELOAD(&htim4,receivedCMD->speed);//D15
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,__HAL_TIM_GET_AUTORELOAD(&htim4)/2);
        osDelay(1);
        __HAL_TIM_SET_AUTORELOAD(&htim16,receivedCMD->speed);//b4
        __HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,__HAL_TIM_GET_AUTORELOAD(&htim16)/2);    
      

        osMailFree(object_pool_q_id, received);
        osMailFree(object_pool_q_idCMD, receivedCMD);
        osDelay(100);
    }

}

void AngleCalcTask(void const * argument)
{
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    
    object_pool_q_id = osMailCreate(osMailQ(object_pool_q), NULL);
    properties_t *object_data;
    object_data = (properties_t *) osMailAlloc(object_pool_q_id, osWaitForever);
    
    
    int16_t buffer[3] = {0};
    float Buffer[3];
    
    float angle;
    //uint8_t UserTxBufferFS[100] = "";
    
    //this variable allows that the gyroscope value could be updated from the accelerometer
    bool b_GyroInit = true;
    //this variable prevents the gyroscopic drift
    uint8_t samplingCounter = 0;
    
    for(;;)
    {
        
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
        osDelay(100);
        BSP_ACCELERO_GetXYZ(buffer);
        xval = buffer[0];
        yval = buffer[1];
        zval = buffer[2];
        
        //printf("X = %u , Y = %u , Z = %u \r\n",xval,yval,zval);
        //sprintf((char*)UserTxBufferFS ,"<-- X = %d , Y = %d , Z = %d --> \r\n",xval,yval,zval);
        //CDC_Transmit_FS(UserTxBufferFS , strlen((char*)UserTxBufferFS));
        //memset(UserTxBufferFS,'0',sizeof(UserTxBufferFS));
        
        xAcc=(double)xval/16384;
        yAcc=(double)yval/16384;
        zAcc=(double)zval/16384;
        
        yaw  =atan(zAcc/sqrt(xAcc*xAcc+yAcc*yAcc))*57.32;
        
        //globAccelangle = yaw;
        
        //sprintf((char*)UserTxBufferFS ,"------------- \r\n");
        //CDC_Transmit_FS(UserTxBufferFS , strlen((char*)UserTxBufferFS));
        //memset(UserTxBufferFS,'0',sizeof(UserTxBufferFS));
        //
        //sprintf((char*)UserTxBufferFS ,"acel Z angle = %f \r\n",yaw);
        //CDC_Transmit_FS(UserTxBufferFS , strlen((char*)UserTxBufferFS));
        //memset(UserTxBufferFS,'0',sizeof(UserTxBufferFS));
        
        BSP_GYRO_GetXYZ(Buffer);
        
        Xval = Buffer[0];
        Yval = Buffer[1];
        Zval = Buffer[2];
        
        if (b_GyroInit == true)
        {
            angle = yaw;
            b_GyroInit = false;
        }

        angle =angle - Xval *0.0001;
        
        //sprintf((char*)UserTxBufferFS ,"gyro Z angle = %f \r\n",angle);
        //CDC_Transmit_FS(UserTxBufferFS , strlen((char*)UserTxBufferFS));
        //memset(UserTxBufferFS,'0',sizeof(UserTxBufferFS));
        
        samplingCounter++;
        if (samplingCounter>GYROSCOPE_DRIFT)
        {
            samplingCounter=0;
            b_GyroInit = true;
        }
        
        //printf("accel angle %f gyro angle %f  \r\n",yaw,angle);
        object_data->D_Angle = angle;
        osMailPut(object_pool_q_id, object_data);
    }
}
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
  my_semaphore_id = osSemaphoreCreate(osSemaphore(my_semaphore), 1);  // Create semaphore with 4 tokens
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  
  // create buffer
  uint8_t Buf[50];
  
  ST_CommParam stCommParam ;
  
  stCommParam.angleOffset=0;
  stCommParam.KD =0;
  stCommParam.KI =0;
  stCommParam.KP =0;
  stCommParam.speed =0;
  
  /* USER CODE BEGIN 5 */
    object_pool_q_idCMD = osMailCreate(osMailQ(object_pool_qCMD), NULL);
    ST_CommParam *object_dataCMD;
    object_dataCMD = (ST_CommParam *) osMailAlloc(object_pool_q_idCMD, osWaitForever);
    HAL_UART_Receive_IT(&huart1,Buf,sizeof(Buf));
    
  /* Infinite loop */
  for(;;)
  {
      //printf("%u",k);
      HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_10);
    // read values form uart buffer
    // call uart read fcn
    //HAL_UART_Receive(&huart1,Buf,sizeof(Buf),500); // 500 is for the blocking mode it the recive fcn is interrupted by the OS (too short taimeout or too long message)the uart fails
     
    //printf("%s",Buf);
    // parse the message 
    //LLDriverCliMenu(Buf,&stCommParam);
      if (Buf[0]=='?')
      {printf("IS ?\r\n");}
      else if(Buf[0]=='D')
      {printf("IS D\r\n");}
      //memset(Buf,'\0',strlen((const char *)Buf));
      Buf[0]=0;
    //dispatche it to the mailing queue
    memcpy(object_dataCMD,&stCommParam,sizeof(ST_CommParam));
    //object_dataCMD->angleOffset=200;
    osMailPut(object_pool_q_idCMD, object_dataCMD);
    osDelay(200);
    
  }
  /* USER CODE END 5 */ 
}

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  // try the semaphore Take
    osSemaphoreWait(my_semaphore_id, osWaitForever);
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  
  // try the semaphore give fromISR
  //:;,
    osSemaphoreRelease(my_semaphore_id);
  
  /* USER CODE END USART1_IRQn 1 */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
