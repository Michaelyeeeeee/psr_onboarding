#include "run.h"
#include "canlib.hpp"

using namespace PSR;

extern "C"{
	extern FDCAN_HandleTypeDef hfdcan1;
}

Canlib can = Canlib(&hfdcan1);
extern "C" void run(){
	can.Can_Init();
	uint8_t id = 0x200;
	uint8_t payload[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	payload[4] = 1;

	HAL_StatusTypeDef status;
	int count = 0;
	while(count < 2){
		status = can.Can_Transmit(id, payload);
		if(status == HAL_OK) count++;
		HAL_Delay(50);
	}
}
