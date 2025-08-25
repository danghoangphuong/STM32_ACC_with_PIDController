#include "HC_SR04.h"

// Enable TIMER input capture
// Set default polarity to RISING EDGE
TIM_HandleTypeDef *htim;
HC_SR04_Typedef *sensor;

void Delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(htim, 0);
	while(__HAL_TIM_GET_COUNTER(htim) < us);
}

HAL_TIM_ActiveChannel HCSR04_Map_channel(uint32_t tim_channel)
{
	switch(tim_channel)
	{
		case TIM_CHANNEL_1:
			return HAL_TIM_ACTIVE_CHANNEL_1;
		case TIM_CHANNEL_2:
			return HAL_TIM_ACTIVE_CHANNEL_2;
		case TIM_CHANNEL_3:
			return HAL_TIM_ACTIVE_CHANNEL_3;
		case TIM_CHANNEL_4:
			return HAL_TIM_ACTIVE_CHANNEL_4;
		default:
			return HAL_TIM_ACTIVE_CHANNEL_1;
	}
}

uint32_t HCSR04_Map_CC_IT(uint32_t tim_channel)
{
	switch(tim_channel)
		{
			case TIM_CHANNEL_1:
				return TIM_IT_CC1;
			case TIM_CHANNEL_2:
				return TIM_IT_CC2;
			case TIM_CHANNEL_3:
				return TIM_IT_CC3;
			case TIM_CHANNEL_4:
				return TIM_IT_CC4;
			default:
				return TIM_IT_CC1;
		}
}

void HCSR04_Init(TIM_HandleTypeDef *hcsr04_htim, HC_SR04_Typedef *hcsr04_x, uint32_t tim_channel)
{
	sensor = hcsr04_x;
	htim = hcsr04_htim;
	htim->Instance->PSC = 63;
	htim->Instance->ARR = 65535;
	hcsr04_x->diff = 0;
	hcsr04_x->distance = 0;
	hcsr04_x->IC_val1 = 0;
	hcsr04_x->IC_val2 = 0;
	hcsr04_x->is_first_capture = 0;
	hcsr04_x->tim_channel = tim_channel;
	HAL_TIM_IC_Start_IT(htim, htim->Channel);
}

void HCSR04_Send_Trigger(HC_SR04_Typedef *hcsr04_x, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) // put in loop, 
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);
	Delay_us(10);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);
	__HAL_TIM_ENABLE_IT(htim, HCSR04_Map_CC_IT(hcsr04_x->tim_channel));
}


void HCSR04_handle(HC_SR04_Typedef *hcsr04_x, TIM_HandleTypeDef *htim) // put in call back
{
	if(htim->Channel == HCSR04_Map_channel(hcsr04_x->tim_channel))
	{
		if(hcsr04_x->is_first_capture == 0)
		{
			hcsr04_x->IC_val1 = HAL_TIM_ReadCapturedValue(htim, hcsr04_x->tim_channel);
			hcsr04_x->is_first_capture = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, hcsr04_x->tim_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if(hcsr04_x->is_first_capture == 1)
		{
			hcsr04_x->IC_val2 = HAL_TIM_ReadCapturedValue(htim, hcsr04_x->tim_channel);
			__HAL_TIM_SET_COUNTER(htim, 0);
			if(hcsr04_x->IC_val2 > hcsr04_x->IC_val1)
			{
				hcsr04_x->diff = hcsr04_x->IC_val2 - hcsr04_x->IC_val1;
			}
			else if(hcsr04_x->IC_val2 < hcsr04_x->IC_val1)
			{
				hcsr04_x->diff = 0xffff + hcsr04_x->IC_val2 - hcsr04_x->IC_val1;
			}
			hcsr04_x->distance = 0.017*hcsr04_x->diff;
			hcsr04_x->is_first_capture = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, hcsr04_x->tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, HCSR04_Map_CC_IT(hcsr04_x->tim_channel));
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//put call back here
	HCSR04_handle(sensor, htim);
}


