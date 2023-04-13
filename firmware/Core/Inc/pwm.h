#ifndef PWM_H
#define PWM_H
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;


extern uint16_t pwm_frequency_01;
extern uint16_t pwm_frequency_2345;
extern uint16_t pwm_duties[6];


void init_pwm(void);
void update_pwm(void);

#endif