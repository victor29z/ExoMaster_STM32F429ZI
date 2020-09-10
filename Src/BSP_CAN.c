/**
  ******************************************************************************
  * @FileName:	Can_User.c
  * @Author:		Victor
  * @Version:		V1.0.0
  * @Date:			06-08-2020
  * @Description:   
  * @Others:
  ******************************************************************************
  * @FunctionsList:	
  *
  *
  ******************************************************************************
  * @Copyright(C), 2020 Smokie Robotics, Inc
  ******************************************************************************
  * @History:
  *
  ******************************************************************************
  */


/* Private Includes ----------------------------------------------------------*/
#include "BSP_CAN.h"
#include <joint_data_type.h>
#include "stdbool.h"

/* Private Includes ----------------------------------------------------------*/

/* Private Typedefs ----------------------------------------------------------*/

/* Macros --------------------------------------------------------------------*/

/* Private Variables ---------------------------------------------------------*/
CAN_RxHeaderTypeDef     RxMsg;
unsigned char new_can_msg = 0;
JOINT_DAT_TYPE joint_data;
const unsigned int cfg_UploadID_list[16]={
	370,371,372,373,374,375,376,401,	//L
	380,381,382,383,384,385,386,400		//R
};
bool joint_data_updated[16] = {false};

/* External Variables --------------------------------------------------------*/

/* Exported Functions --------------------------------------------------------*/

/**
  * @brief  initialize can filter and interrupt
  * @param  hcan: can handler
  * @retval None
  */

 
uint8_t BSP_CAN_Init(CAN_HandleTypeDef* hcan )   //�û���ʼ������
{
	CAN_FilterTypeDef  sFilterConfig;
	HAL_StatusTypeDef  HAL_Status;
	

	
	sFilterConfig.FilterBank = 0;                       //use filter0
	sFilterConfig.FilterMode =  CAN_FILTERMODE_IDMASK;  //use id list mode
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		
	sFilterConfig.FilterIdHigh = 0;   //BASE_ID
	sFilterConfig.FilterIdLow  = 0;  //BASE_ID+1
		
	sFilterConfig.FilterMaskIdHigh =0;//BASE_ID+2
	sFilterConfig.FilterMaskIdLow  =0; //BASE_ID+3
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;	// use can_rx_fifo0    

	sFilterConfig.FilterActivation = ENABLE;  	//enable filter
	sFilterConfig.SlaveStartFilterBank  = 0; 

	HAL_Status=HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
	HAL_Status=HAL_CAN_Start(hcan);  //enable CAN
		
	HAL_Status=HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // enable interrupt
	

	return 0;
		
 }

/**
  * @brief  overrided can receive pending interrupt callback
  * @param  hcan: can handler
  * @retval None
  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //���ջص�����
{
  uint8_t  Data[8];
  char i;
  HAL_StatusTypeDef	HAL_RetVal;
  
  if(hcan ==&hcan1){	
    HAL_RetVal=HAL_CAN_GetRxMessage(hcan,  CAN_RX_FIFO0, &RxMsg,  Data);
    if ( HAL_OK==HAL_RetVal){                              			
      //�������������
    	new_can_msg = 1;
    	int dat;
		int keyvalue;
		// use highest byte for key value, and low 3 bytes for encoder value
		dat = (unsigned char)Data[1] *65536 + (unsigned char)Data[2] * 256 + (unsigned char)Data[3];
		keyvalue = (unsigned char)Data[0];
		unsigned int canid = RxMsg.StdId;
		for(i = 0; i < 16; i++){
			if(canid == cfg_UploadID_list[i]){
			joint_data_updated[i] = true;
					//joint_data.joint_online[i] = true;
			joint_data.joint_pos_raw[i] = dat;
			joint_data.joint_pos_valid[i] = true;
			joint_data.keyvalue[i] = keyvalue;
			}
		}
    }
  }
}

/**
  * @brief  send a can message
  * @param  hcan: can handler
  			ID: CAN ID
  			pData: data bytes
  			Len: Length of data
  * @retval Error number
  */

uint8_t CANx_SendNormalData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len)
{
	HAL_StatusTypeDef	HAL_RetVal;
    uint16_t SendTimes,SendCNT=0;
	uint8_t  FreeTxNum=0;
	static CAN_TxHeaderTypeDef     TxMsg;
	TxMsg.StdId=ID;
	if(!hcan || ! pData ||!Len)  return 1;
	SendTimes=Len/8+(Len%8?1:0);
	FreeTxNum=HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	TxMsg.DLC=8;
	while(SendTimes--){
		if(0==SendTimes){
			if(Len%8)
				TxMsg.DLC=Len%8;
		}
		while(0==FreeTxNum){
			FreeTxNum=HAL_CAN_GetTxMailboxesFreeLevel(hcan);
		}
		HAL_Delay(1);  
		HAL_RetVal=HAL_CAN_AddTxMessage(hcan,&TxMsg,pData+SendCNT,(uint32_t*)CAN_TX_MAILBOX0); 
		if(HAL_RetVal!=HAL_OK)
		{
			return 2;
		}
		SendCNT+=8;
	}
	
  return 0;
}


/********************** (C) COPYRIGHT 2020 Smokie Robotics *****END OF FILE****/


