#include "systime.h"
#include "stm32f746xx.h"
#include "delay.h"

// TIM2 has a 108 MHz clock

void systime_init()
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  for (volatile int i = 0; i < 10000; i++) { } // not sure why we need this
  TIM2->PSC = 108-1; // prescalar counts microseconds
  TIM2->ARR = 0xffffffff; // count as long as possible
  TIM2->EGR = TIM_EGR_UG; // load the PSC register immediately
  TIM2->CR1 = TIM_CR1_CEN; // start counter
}

