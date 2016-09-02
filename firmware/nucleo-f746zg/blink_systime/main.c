#include "led.h"
#include "delay.h"
#include "systime.h"
#include "console.h"
#include <stdio.h>

int main()
{
  led_init();
  led_on();
  systime_init();
  while (1)
  {
    delay_ms(500);
    led_toggle();
  }
  return 0;
}
