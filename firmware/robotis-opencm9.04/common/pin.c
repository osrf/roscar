#include "pin.h"
#include <stdio.h>

void pin_enable_gpio(GPIO_TypeDef *gpio)
{
  if (gpio == GPIOA)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  else if (gpio == GPIOB)
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  else if (gpio == GPIOC)
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
  else if (gpio == GPIOD)
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
  else if (gpio == GPIOE)
    RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
}

#if 0
void pin_set_output_type(GPIO_TypeDef *gpio, 
                         const uint8_t pin_idx,
                         const uint8_t output_type)
{
  pin_enable_gpio(gpio);
  if (output_type == PIN_OUTPUT_TYPE_OPEN_DRAIN)
  {
    printf("setting pin %d to open-drain\r\n", pin_idx);
    gpio->OTYPER |= (1 << pin_idx);
  }
  else
  {
    printf("setting pin %d to push-pull\r\n", pin_idx);
    gpio->OTYPER &= ~(1 << pin_idx);
  }
}
#endif

void pin_set_alternate_function(GPIO_TypeDef *gpio,
                                const uint8_t pin_idx,
                                const bool is_output,
                                const pin_pull_t pull)
{
  if (pin_idx > 15)
    return; // adios amigo
  pin_enable_gpio(gpio);
  volatile uint32_t *pc = NULL;
  uint32_t shift = pin_idx * 4;
  if (pin_idx >= 8)
  {
    pc = &gpio->CRH;
    shift -= 8 * 4;
  }
  else
    pc = &gpio->CRL;
  *pc &= ~(0xf << shift); // wipe out whatever was there before
  uint32_t mode_bits = 0;
  if (is_output)
    mode_bits = 0xb; // high-speed AF output
  else // it's an input
  {
    if (pull == PIN_PULL_NONE)
      mode_bits = 0x4;
    else if (pull == PIN_PULL_UP)
    {
      mode_bits = 0x8;
      gpio->ODR |= 1 << pin_idx;
    }
    else if (pull == PIN_PULL_DOWN)
    {
      mode_bits = 0x8;
      gpio->ODR &= ~(1 << pin_idx);
    }
    else
      while (1) { }  // IT'S A TRAP!!!
  }
  *pc |= (mode_bits << shift);
}

void pin_set_output(GPIO_TypeDef *gpio, 
                    const uint8_t pin_idx, 
                    const uint8_t initial_state)
{
  if (pin_idx > 15)
    return; // adios amigo
  pin_enable_gpio(gpio);
  pin_set_output_state(gpio, pin_idx, initial_state);
  volatile uint32_t *pc = NULL;
  uint32_t shift = pin_idx * 4;
  if (pin_idx >= 8)
  {
    pc = &gpio->CRH;
    shift -= 8 * 4;
  }
  else
    pc = &gpio->CRL;
  *pc = (*pc & ~(0xf << shift)) | (1 << shift);
}

#if 0
void pin_set_analog(GPIO_TypeDef *gpio, const uint8_t pin_idx)
{
  if (pin_idx > 15)
    return; // adios amigo
  pin_enable_gpio(gpio);
  gpio->MODER |= 3 << (pin_idx * 2);
}
#endif

void pin_set_output_state(GPIO_TypeDef *gpio, 
                          const uint8_t pin_idx, 
                          const uint8_t state)
{
  if (state)
    gpio->BSRR = 1 << pin_idx;
  else
    gpio->BSRR = (1 << pin_idx) << 16;
}

#if 0
void pin_set_output_speed(GPIO_TypeDef *gpio,
                          const uint_fast8_t pin_idx,
                          const uint_fast8_t speed)
{
  pin_enable_gpio(gpio);
  if (pin_idx > 15)
    return;
  if (speed > 3)
    return;
  gpio->OSPEEDR &= ~(0x3 << (pin_idx * 2)); // wipe out the old setting
  gpio->OSPEEDR |= speed << (pin_idx * 2);  // stuff in the new one
}
#endif

void pin_toggle_state(GPIO_TypeDef *gpio, const uint8_t pin_idx)
{
  gpio->ODR ^= (1 << pin_idx);
}

