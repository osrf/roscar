#include "led.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"
#include "usb.h"
#include "stm32f103xb.h"
#include "dmxl.h"

int g_num_usb_sof = 0;
void usb_sof()
{
  g_num_usb_sof++;
}

typedef struct status_pkt
{
  uint32_t t;
} __attribute__((packed)) status_pkt_t;
struct status_pkt g_status_pkt;

void usb_rx(const uint8_t ep, const uint8_t *data, const uint8_t len)
{
  printf("rx %d bytes on ep%d\r\n", (int)len, (int)ep);
  if (data[0])
    led_on();
  else
    led_off();
}

int main()
{
  led_init();
  led_on();
  console_init();
  printf("********************************\r\nAPP ENTRY\r\n");
  systime_init();
  usb_init();
  dmxl_init();
  __enable_irq();
  while (1)
  {
    /*
    while (g_num_usb_sof < 500) { }
    g_num_usb_sof = 0;
    //led_toggle();
    g_status_pkt.t = systime_usecs();
    usb_tx(1, (const uint8_t *)&g_status_pkt, sizeof(g_status_pkt));
    //printf("systime %d\r\n", (int)systime_usecs());
    */
    dmxl_tick();
  }
  return 0;
}
