#include "led.h"
#include "delay.h"
#include "systime.h"
#include "console.h"
#include <stdio.h>

int main()
{
  led_init();
  led_on();
  console_init();
  printf("===== ENTRY =====\n");
  for (int loop_count = 0; ; loop_count++)
  {
    for (volatile int i = 0; i < 10000000; i++) { }
    led_toggle();
    printf("hello world %d\n", loop_count);
  }
  return 0;
}
