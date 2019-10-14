#ifndef LLDRIVERS_H
#define LLDRIVERS_H

#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

#define STR_DELIM   ';'
#define ENTER_ASCII 0x0D

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




//osMailQDef (object_pool_qCMD, 4*sizeof(ST_CommParam), ST_CommParam);  // Declare mail queue
//osMailQId  (object_pool_q_idCMD);                 // Mail queue ID

#endif //LLDRIVERS_H

