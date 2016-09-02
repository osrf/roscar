#include <stdint.h>
#include "stm32f746xx.h"
//#include "watchdog.h"
#include "stack.h"
#include "flash.h"
#include <led.h>

// crystal HSE_OSC is 25 MHz

// TODO: change this, because stm32f7 is way more awesome than this:
// we want SYSCLK, AHB, and APB2 to run at 72 MHz
// we want APB1 at 36 MHz
// we want USBCLK at 48 MHz using the /1.5 USB prescalar from 72 MHz PLLCLK

extern uint32_t _srelocate_flash, _srelocate, _erelocate, _ebss, _sbss;
extern int main();

void startup_clock_init_fail() { while (1) { } }

// where is this defined? I'm sure somebody else declares it somewhere.
void __libc_init_array();

void reset_vector()
{
  //watchdog_reset_counter();
  g_stack[0] = 0; // hit the stack to ensure the linker keeps it around...
  // set up data segment
  uint32_t *pSrc = &_srelocate_flash;
  uint32_t *pDest = &_srelocate;
  if (pSrc != pDest)
    for (; pDest < &_erelocate; )
      *pDest++ = *pSrc++;
  // set up bss segment
  for (pDest = &_sbss; pDest < &_ebss; )
    *pDest++ = 0;
  SCB->CPACR |= ((3UL << (10*2)) | (3UL << (11*2))); // activate the CM7 FPU
  __libc_init_array();
  flash_init(); // slow down flash, because we're about to go super fast
  // time to set up the clocking scheme. pun intended.
  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // spin up the power controller
  // for debugging: RCC->CR      is 0x40023800
  //                RCC->PLLCFGR is 0x40023804
  RCC->CR |= RCC_CR_HSION; // ensure the HSI (internal) oscillator is on
  RCC->CFGR = RCC_CFGR_SW_HSI; // ensure the HSI is the clock source
  RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON | RCC_CR_CSSON); // turn off Fancy
  //RCC->PLLCFGR = 0x24003010; // ensure PLLCFGR is at reset state
  RCC->CR &= ~RCC_CR_HSEBYP; // reset HSEBYP (i.e., HSE is *not* bypassed)
  RCC->CIR = 0x0; // disable all RCC interrupts
  RCC->CR |= RCC_CR_HSEON; // enable HSE oscillator (off-chip crystal)
  for (volatile uint32_t i = 0;
       i < 0x100000 /*HSE_STARTUP_TIMEOUT*/ && !(RCC->CR & RCC_CR_HSERDY); i++)
  { } // wait for either timeout or HSE to spin up
  if (!(RCC->CR & RCC_CR_HSERDY))
    startup_clock_init_fail(); // go there and spin forever. BUH BYE
  // let's go for max beef:
  //    216 MHz core CPU frequency
  //    216 MHz AHB bus (aka HCLK)
  //    108 MHz APB2 (high-speed bus, aka PCLK2)
  //     54 MHz APB1 ( low-speed bus, aka PCLK1)
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // set HCLK (AHB clock) to have no division
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // set APB high-speed clock to sysclock/2
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // set APB  low-speed clock to sysclock/4
  // PLL_M sets up an input frequency of 1 MHz for the PLL's, as per datasheet
  #define PLL_M (HSE_VALUE / 1000000)
  // PLL_N is the main multipler. this sets up a VCO frequency of 1 * N = 432
  //#define PLL_N 432
  #define PLL_N 432
  // SYSCLK = PLL_VCO / PLL_P = 432 / 2 = 216 MHz
  #define PLL_P   2
  // USB clock = PLL_VCO / PLL_Q = 432 / 9 = 48 MHz
  #define PLL_Q   9
  RCC->PLLCFGR = 0x20000000 | // top nibble is "reserved" in datasheet
                 PLL_M | (PLL_N << 6) | (((PLL_P >> 1)-1) << 16) |
                 (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);
  RCC->CR |= RCC_CR_PLLON; // start spinning up the PLL
  while (!(RCC->CR & RCC_CR_PLLRDY)) { } // wait until it's spun up
  PWR->CR1 |= PWR_CR1_ODEN;
  while (!(PWR->CSR1 & PWR_CSR1_ODRDY)); // wait for overdrive to be ready
  PWR->CR1 |= PWR_CR1_ODSWEN; // buckle your seat belts. switch to overdrive.
  while (!(PWR->CSR1 & PWR_CSR1_ODSWRDY)); // wait for overdrive to engage
  RCC->CFGR &= ~((uint32_t)RCC_CFGR_SW); // select internal oscillator
  RCC->CFGR |= RCC_CFGR_SW_PLL; // select PLL as clock source
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // wait for it...
  // hooray we're done! we're now running at 216 MHz VROOM VROOM VROOOOM
  SCB->CCR |= (1 << 18); // enable branch prediction
  __DSB();
  SCB_EnableICache();
  SCB_EnableDCache();
  main(); // jump to application code now
  while (1); // shouldn't ever get here...
}
