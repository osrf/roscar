#include "systime.h"
#include "stm32f103xb.h"
#include "delay.h"

// TIM2 and TIM3 have a 72 MHz clock coming into them

void systime_init()
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
  for (volatile int i = 0; i < 10000; i++) { }
  TIM2->PSC = 72-1; // prescalar counts microseconds
  TIM2->ARR = 0xffff; // count as long as possible
  TIM2->EGR = TIM_EGR_UG; // load the PSC register immediately
  TIM2->CR2 = TIM_CR2_MMS_1; // send update events
  TIM2->CR1 = TIM_CR1_CEN; // start counter
  // now, the high-order 16 bits using TIM3
  TIM3->PSC = 1;
  TIM3->ARR = 0xffff; // count as long as possible
  TIM3->SMCR |= 
    TIM_SMCR_MSM |
    TIM_SMCR_TS_0 | // pull trigger from TIM2 events
    TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0; // clock from trigger
  TIM3->CR1 = TIM_CR1_CEN; // start counter
}

uint32_t systime_usecs()
{
  // todo: not sure how we can handle the glitch if we're reading the timers
  // right at the instant of TIM2 wraparound :-(
  return TIM2->CNT + (TIM3->CNT << 16);
}
