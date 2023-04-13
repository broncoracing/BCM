#include "pwm.h"

uint16_t pwm_frequency_01 = 1;
uint16_t pwm_frequency_2345 = 20;
uint16_t pwm_duties[6] = {0};


void init_pwm(void) {
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

}

void update_pwm(void) {
  // Channel 0/1
  TIM2->ARR = 0xFFFE / pwm_frequency_01;
  uint32_t arr01 = TIM2->ARR + 2;
  TIM2->CCR3 = (arr01 * (uint32_t)pwm_duties[0]) >> 16;
  TIM2->CCR4 = (arr01 * (uint32_t)pwm_duties[1]) >> 16; 

  // Remaining channels
  TIM1->ARR = 0xFFFE / pwm_frequency_2345;
  uint32_t arr2345 = TIM1->ARR + 2;
  TIM1->CCR1 = (arr2345 * (uint32_t)pwm_duties[2]) >> 16;
  TIM1->CCR2 = (arr2345 * (uint32_t)pwm_duties[3]) >> 16; 
  TIM1->CCR3 = (arr2345 * (uint32_t)pwm_duties[4]) >> 16;
  TIM1->CCR4 = (arr2345 * (uint32_t)pwm_duties[5]) >> 16;
}

