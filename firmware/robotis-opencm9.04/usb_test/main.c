#include "led.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"
#include "usb.h"
#include "stm32f103xb.h"

int main()
{
  led_init();
  led_on();
  console_init();
  printf("APP ENTRY\r\n");
  systime_init();
  usb_init();
  __enable_irq();
  while (1)
  {
    delay_ms(500);
    led_toggle();
    //printf("systime %d\r\n", (int)systime_usecs());
  }
  return 0;
}
