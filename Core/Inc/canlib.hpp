#ifndef INC_CANLIB_HPP_
#define INC_CANLIB_HPP_
#include "stm32g0xx_hal.h"

namespace PSR{

		class Canlib{
		public:
				// constructor and argument pointer * hfdcan
				// initializes fdcan_inter
				Canlib(FDCAN_HandleTypeDef * hfdcan): fdcan_inter(hfdcan){};
				HAL_StatusTypeDef Can_Init(); // implement this
				HAL_StatusTypeDef Can_Transmit(uint8_t id, uint8_t * payload); //implement this
		private:
				FDCAN_HandleTypeDef * fdcan_inter;
		};
}
#endif
