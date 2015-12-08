#include "led.h"
#include "delay.h"
#include "systime.h"

int main()
{
  led_init();
  led_on();
  //console_init();
  //puts("===== APP ENTRY =====\r\n");
  //systime_init();
  while (1)
  {
    //delay_ms(500);
    for (volatile int i = 0; i < 1000000; i++) { }
    led_toggle();
  }
  return 0;
}
