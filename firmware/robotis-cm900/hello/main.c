#include "led.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"

int main()
{
  led_init();
  led_on();
  console_init();
  printf("APP ENTRY\r\n");
  //systime_init();
  int i = 0;
  while (1)
  {
    //delay_ms(500);
    for (volatile int i = 0; i < 1000000; i++) { }
    led_toggle();
    printf("hello world %d\r\n", i);
    i++;
  }
  return 0;
}
