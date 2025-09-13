#include "canlib.hpp"

namespace PSR{
		HAL_StatusTypeDef Canlib::Can_Init(){
			// pg 257
			HAL_StatusTypeDef status = HAL_ERROR;
			status = HAL_FDCAN_Init(fdcan_inter);
			if(status == HAL_OK) HAL_FDCAN_Start(fdcan_inter);
			return status;
			//look for start
		}
		HAL_StatusTypeDef Canlib::Can_Transmit(uint8_t id, uint8_t * payload){
			FDCAN_TxHeaderTypeDef frame;
			frame.Identifier = id;
			frame.IdType = FDCAN_EXTENDED_ID;
			frame.DataLength = 8;
			frame.FDFormat = FDCAN_FD_CAN;
			frame.TxFrameType = FDCAN_FRAME_FD_NO_BRS;
			frame.BitRateSwitch = FDCAN_BRS_OFF;
			frame.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
			frame.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
			frame.MessageMarker = 0;
			//look for transmission
			// pg 265
			// HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ (FDCAN_HandleTypeDef * hfdcan, FDCAN_TxHeaderTypeDef * pTxHeader, uint8_t * pTxData)
			HAL_StatusTypeDef status = HAL_ERROR;
			if(Can_Init() == HAL_OK)
				 status = HAL_FDCAN_AddMessageToTxFifoQ(fdcan_inter, &frame, payload);
			return status;
		}
}


