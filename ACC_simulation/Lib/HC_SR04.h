#ifndef __HC_SR04_H
#define __HC_SR04_H

#include "stm32f1xx.h"

typedef struct{
	uint32_t IC_val1;
	uint32_t IC_val2;
	uint32_t diff;
	uint16_t distance;
	uint8_t is_first_capture;
	uint32_t tim_channel;
}HC_SR04_Typedef;

void Delay_us(uint16_t us);
HAL_TIM_ActiveChannel HCSR04_Map_channel(uint32_t tim_channel);
uint32_t HCSR04_Map_CC_IT(uint32_t tim_channel);
void HCSR04_Init(TIM_HandleTypeDef *hcsr04_htim, HC_SR04_Typedef *hcsr04_x, uint32_t tim_channel);
void HCSR04_Send_Trigger(HC_SR04_Typedef *hcsr04_x, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HCSR04_handle(HC_SR04_Typedef *hcsr04_x, TIM_HandleTypeDef *htim);
#endif

