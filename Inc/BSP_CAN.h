/**
  ******************************************************************************
  * @FileName:		BSP_CAN.h
  * @Author:		Frank
  * @Version:		V1.0.0
  * @Date:			
  * @Description:   
  * @Others:
  ******************************************************************************
  * @Copyright(C), 2020, Smokie Robotics, Inc
  ******************************************************************************
  * @History:
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_CAN_H
#define __BSP_CAN_H



/* Private Includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private Typedefs ----------------------------------------------------------*/


/* Macros --------------------------------------------------------------------*/
#define CAN_ID_BASE    	  0x001

/* Private Variables ---------------------------------------------------------*/

/* External Variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;


/* Exported Functions --------------------------------------------------------*/
uint8_t BSP_CAN_Init(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
uint8_t CANx_SendNormalData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);


#endif
/********************** (C) COPYRIGHT 2020 Smokie Robotics *****END OF FILE****/

