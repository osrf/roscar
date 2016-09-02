#include "flash.h"
#include "stm32f746xx.h"

void flash_init(void)
{
  FLASH->ACR = 0; // ensure the caches are turned off, so we can reset them
  FLASH->ACR = FLASH_ACR_PRFTEN |     // enable flash prefetch
               FLASH_ACR_ARTEN  |     // enable ART (cache)
               FLASH_ACR_LATENCY_7WS; // set 7-wait-state 
}
