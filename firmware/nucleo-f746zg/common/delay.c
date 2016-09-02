#include "delay.h"
#include <stdint.h>
#include "systime.h"

// these functions assume that systime is running at microsecond resolution

void delay_ns(const uint32_t ns)
{
  // TODO: actually tune this better on an oscilloscope
  for (volatile uint32_t i = 0; i < ns/10; i++) { }
}

void delay_us(const uint32_t us)
{
  // todo: care about wraparound
  volatile uint32_t t_start = SYSTIME;
  while (t_start + us > SYSTIME);
}

void delay_ms(const uint32_t ms)
{
  // todo: care about wraparound
  volatile uint32_t t_start = SYSTIME;
  while (t_start + 1000 * ms > SYSTIME);
}

