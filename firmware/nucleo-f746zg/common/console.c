#include "console.h"
#include "stm32f746xx.h"
#include "pin.h"

// pin connections to send console through ST-LINK virtual serial port:
// PD8 = USART3 TX
// PD9 = USART3 RX

#define PORTD_TX_PIN 8
#define PORTD_RX_PIN 9

static volatile bool s_console_init_complete = false;
static volatile USART_TypeDef * const s_console_usart = USART3;

// USART3 sits on APB1, which is 54 MHz
// (used to be 36 mhz)

void console_init()
{
  s_console_init_complete = true;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  pin_set_alternate_function(GPIOD, PORTD_TX_PIN, 7); // USART3 is AF7
  s_console_usart->CR1 &= ~USART_CR1_UE;
  s_console_usart->CR1 |=  USART_CR1_TE | USART_CR1_RE;
  s_console_usart->BRR  = 144;
  s_console_usart->CR1 |=  USART_CR1_UE;
}

void console_send_block(const uint8_t *buf, uint32_t len)
{
  if (!s_console_init_complete)
    console_init();
  for (uint32_t i = 0; i < len; i++)
  {
    while (!(s_console_usart->ISR & USART_ISR_TXE)) { } // wait for tx to clear
    s_console_usart->TDR = buf[i];
  }
  while (!(s_console_usart->ISR & USART_ISR_TC)) { } // wait for TX to finish
}

