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

static uint32_t g_last_motion_cmd_time = 0;

void usb_rx(const uint8_t ep, const uint8_t *data, const uint8_t len)
{
  //printf("rx %d bytes on ep%d\r\n", (int)len, (int)ep);
  uint32_t cmd = *((uint32_t *)(&data[0]));
  if (cmd == 0) // set led
  {
    if (data[4] & 0x1)
      led_on();
    else
      led_off();
  }
  else if (cmd == 1) // DRIVE
  {
    uint16_t left = *((uint16_t *)(&data[4]));
    uint16_t right = *((uint16_t *)(&data[6]));
    printf("rx left %d right %d\r\n", (int)left, (int)right);
    dmxl_set_regs(1, 0x20, 2, (uint8_t *)&left);  // set left velocity
    delay_us(5);
    dmxl_set_regs(2, 0x20, 2, (uint8_t *)&right); // set right velocity
    g_last_motion_cmd_time = systime_usecs();
  }
  else if (cmd == 2) // set random dynamixel register
  {
    uint8_t id = data[4];
    uint8_t reg_idx = data[5];
    uint8_t reg_val = data[6];
    printf("setting dmxl %d reg 0x%d to %d\r\n",
           id, (unsigned)reg_idx, reg_val);
    dmxl_set_regs(id, reg_idx, 1, &reg_val);
  }
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
    dmxl_tick();
    // check for deadman timeout every 10 ms
    if (g_num_usb_sof >= 10)
    {
      g_num_usb_sof = 0;
      //led_toggle();
      uint32_t t = systime_usecs();
      if (t - g_last_motion_cmd_time > 150000)
      {
        printf("timeout\r\n");
        // it's been 150 ms since last motion command. stop the wheels plz
        uint16_t zero = 0;
        dmxl_set_regs(1, 0x20, 2, (uint8_t *)&zero); // set left velocity
        delay_us(5);
        dmxl_set_regs(2, 0x20, 2, (uint8_t *)&zero); // set right velocity
        g_last_motion_cmd_time = t; // keep sending stop commands every 150 ms
      }
      g_status_pkt.t = t;
      usb_tx(1, (const uint8_t *)&g_status_pkt, sizeof(g_status_pkt));
      //printf("systime %d\r\n", (int)systime_usecs());
    }
  }
  return 0;
}
