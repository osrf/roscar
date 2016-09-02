#include "led.h"
#include "delay.h"
#include "systime.h"
#include "console.h"
#include <stdio.h>

int main()
{
  led_init();
  led_on();
  while (1)
  {
    for (volatile int i = 0; i < 10000000; i++) { }
    led_toggle();
  }
  return 0;
}
