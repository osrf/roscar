#include "console.h"
#include "stm32f103xb.h"
#include "pin.h"

// pin connections
// PA2 = usart2 TX
// PA3 = usart2 RX

#define PORTA_TX_PIN 2
#define PORTA_RX_PIN 3

static volatile bool s_console_init_complete = false;
static volatile USART_TypeDef * const s_console_usart = USART2;

// USART2 sits on APB1, which is 36 MHz

void console_init()
{
  s_console_init_complete = true;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  pin_set_alternate_function(GPIOA, PORTA_TX_PIN, true, PIN_PULL_NONE);
  s_console_usart->CR1 &= ~USART_CR1_UE;
  s_console_usart->CR1 |=  USART_CR1_TE | USART_CR1_RE;
  s_console_usart->BRR  = (((uint16_t)1) << 4) | 2;
  s_console_usart->CR1 |=  USART_CR1_UE;
}

void console_send_block(const uint8_t *buf, uint32_t len)
{
  if (!s_console_init_complete)
    console_init();
  for (uint32_t i = 0; i < len; i++)
  {
    while (!(s_console_usart->SR & USART_SR_TXE)) { } // wait for tx to clear
    s_console_usart->DR = buf[i];
  }
  while (!(s_console_usart->SR & USART_SR_TC)) { } // wait for TX to finish
}

