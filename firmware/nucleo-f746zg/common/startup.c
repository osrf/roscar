#include <stdint.h>
#include "stm32f103xb.h"
//#include "watchdog.h"
#include <led.h>

// crystal HSE_OSC is 25 MHz
// we want SYSCLK, AHB, and APB2 to run at 72 MHz
// we want APB1 at 36 MHz
// we want USBCLK at 48 MHz using the /1.5 USB prescalar from 72 MHz PLLCLK

extern uint32_t _srelocate_flash, _srelocate, _erelocate, _ebss, _sbss;
extern int main();

void startup_clock_init_fail() { while (1) { } }

void reset_vector()
{
  //watchdog_reset_counter();
  // set up data segment
  uint32_t *pSrc = &_srelocate_flash;
  uint32_t *pDest = &_srelocate;
  if (pSrc != pDest)
    for (; pDest < &_erelocate; )
      *pDest++ = *pSrc++;
  // set up bss segment
  for (pDest = &_sbss; pDest < &_ebss; )
    *pDest++ = 0;
  __libc_init_array();
  SCB->CPACR |= ((3UL << (10*2)) | (3UL << (11*2))); // activate the FPU
  //TODO used define provided if changes in future generations ? 
  // set up the clocking scheme
  RCC->CR |= 0x1; // ensure the HSI (internal) oscillator is on
//  RCC->CR |= RCC_CR_HSION;
  RCC->CFGR = 0; // ensure the HSI oscillator is the clock source
//RCC->CFGR = RCC_CFGR_SW_HSI;
  RCC->CR &= 0xfef6ffff; // turn off the main PLL and HSE oscillator
// RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON | RCC_CR_CSSON);
  RCC->PLLCFGR = 0x24003010; // ensure PLLCFGR is at reset state
  RCC->CR &= 0xfffbffff; // reset HSEBYP (i.e., HSE is *not* bypassed)
//  RCC->CR &= ~RCC_CR_HSEBYP;
  RCC->CIR = 0x0; // disable all RCC interrupts
  RCC->CR |= RCC_CR_HSEON; // enable HSE oscillator (off-chip crystal)
  for (volatile uint32_t i = 0;
       i < 0x5000 /*HSE_STARTUP_TIMEOUT*/ && !(RCC->CR & RCC_CR_HSERDY); i++)
  { } // wait for either timeout or HSE to spin up
  flash_init();
  if (!(RCC->CR & RCC_CR_HSERDY))
    startup_clock_init_fail(); // go there and spin forever. BUH BYE
  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // clock up the power controller
  //PWR->CR |= PWR_CR_VOS; // ensure the voltage regulator is at max beef
  //                       // this will let us run at 168 MHz without overdrive
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // set HCLK (AHB clock) to sysclock
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // set APB high-speed clock to sysclock/2
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // set APB  low-speed clock to sysclock/4
  // PLL_M sets up an input frequency of 1 MHz for the PLL's, as per DS p.141
  #define PLL_M (HSE_VALUE / 1000000)
  // PLL_N is the main multipler. this sets up a VCO frequency of 1 * N = 336
  #define PLL_N 336
  #define PLL_P   2
  // SYSCLK = PLL_VCO / PLL_P = 168 MHz
  #define PLL_Q   7
  // USB clock = PLL_VCO / PLL_Q = 48 MHz
  RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1)-1) << 16) |
                 (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);
  RCC->CR |= RCC_CR_PLLON; // start spinning up the PLL
  while (!(RCC->CR & RCC_CR_PLLRDY)) { } // wait until it's spun up
  RCC->CFGR &= ~((uint32_t)RCC_CFGR_SW); // select internal oscillator
  RCC->CFGR |= RCC_CFGR_SW_PLL; // select PLL as clock source
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { } // wait for it...

#if 0
  led_init();
  led_on();
  //for (volatile int i = 0; i < 1000000; i++) { } // save ourselves!
  led_off();
  RCC->CR |= 0x1; // ensure the HSI (internal) oscillator is on
  RCC->CFGR = 0; // ensure the HSI oscillator is the clock source
  RCC->CR &= 0xfef6ffff; // turn off the main PLL and HSE oscillator
  //RCC->PLLCFGR = 0x24003010; // ensure PLLCFGR is at reset state
  //RCC->CR &= 0xfffbffff; // reset HSEBYP (i.e., HSE is *not* bypassed)
  RCC->CIR = 0x0; // disable all RCC interrupts
  RCC->CR |= RCC_CR_HSEON; // enable HSE oscillator (off-chip crystal)
  for (volatile uint32_t i = 0; 
       i < 0x5000 /*HSE_STARTUP_TIMEOUT*/ && !(RCC->CR & RCC_CR_HSERDY); i++)
  { } // wait for either timeout or HSE to spin up
  FLASH->ACR = 0; // ensure the caches are turned off, so we can reset them
  FLASH->ACR = FLASH_ACR_PRFTBE | 2; // 2 wait states
  if (!(RCC->CR & RCC_CR_HSERDY))
    startup_clock_init_fail(); // go there and spin forever. BUH BYE
  // set up the clocking scheme
  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // clock up the power controller
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1   // set HCLK (AHB clock) to sysclock
             | RCC_CFGR_PPRE2_DIV1  // set APB high-speed clock to sysclock
             | RCC_CFGR_PPRE1_DIV2  // set APB  low-speed clock to sysclock/2
             | RCC_CFGR_PLLSRC;     // run PLL on HSE (crystal oscillator)
  // PLL_VCO = crystal mhz * PLLMUL = 72 MHz
  // board has 8 MHz crystal, so we want PLLMUL = 72/8 = 9
  RCC->CFGR |= RCC_CFGR_PLLMULL9;
  RCC->CR |= RCC_CR_PLLON; // start spinning up the PLL
  while (!(RCC->CR & RCC_CR_PLLRDY)) { } // wait until it's spun up
  RCC->CFGR &= ~((uint32_t)RCC_CFGR_SW); // select internal oscillator
  RCC->CFGR |= RCC_CFGR_SW_PLL; // select PLL as clock source
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { } // wait for it...
#endif
  // hooray we're done! we're now running at 72 MHz.
  main(); // jump to application main()
  while (1) { } // hopefully we never get here...
}

#if 0
void unmapped_vector()
{
  while (1) { } // spin to allow jtag trap
}

#define STACK_SIZE 0x1000
__attribute__((aligned(8),section(".stack"))) uint8_t g_stack[STACK_SIZE];

// declare weak symbols for all interrupt so they can be overridden easily
#define WEAK_VECTOR __attribute__((weak, alias("unmapped_vector")))
void nmi_vector() WEAK_VECTOR;
void hardfault_vector() WEAK_VECTOR;
void memmanage_vector() WEAK_VECTOR;
void busfault_vector() WEAK_VECTOR;
void usagefault_vector() WEAK_VECTOR;
void svcall_vector() WEAK_VECTOR;
void dbgmon_vector() WEAK_VECTOR;
void pendsv_vector() WEAK_VECTOR;
void systick_vector() WEAK_VECTOR;
void wwdg_vector() WEAK_VECTOR;
void pvd_vector() WEAK_VECTOR;
void tamper_vector() WEAK_VECTOR;
void rtc_vector() WEAK_VECTOR;
void flash_vector() WEAK_VECTOR;
void rcc_vector() WEAK_VECTOR;
void exti0_vector() WEAK_VECTOR;
void exti1_vector() WEAK_VECTOR;
void exti2_vector() WEAK_VECTOR;
void exti3_vector() WEAK_VECTOR;
void exti4_vector() WEAK_VECTOR;
void dma1_channel1_vector() WEAK_VECTOR;
void dma1_channel2_vector() WEAK_VECTOR;
void dma1_channel3_vector() WEAK_VECTOR;
void dma1_channel4_vector() WEAK_VECTOR;
void dma1_channel5_vector() WEAK_VECTOR;
void dma1_channel6_vector() WEAK_VECTOR;
void dma1_channel7_vector() WEAK_VECTOR;
void adc_vector() WEAK_VECTOR;
void usb_hp_can1_tx_vector() WEAK_VECTOR;
void usb_lp_can1_rx0_vector() WEAK_VECTOR;
void can1_rx1_vector() WEAK_VECTOR;
void can1_sce_vector() WEAK_VECTOR;
void exti9_5_vector() WEAK_VECTOR;
void tim1brk_vector() WEAK_VECTOR;
void tim1up_vector() WEAK_VECTOR;
void tim1trg_vector() WEAK_VECTOR;
void tim1cc_vector() WEAK_VECTOR;
void tim2_vector() WEAK_VECTOR;
void tim3_vector() WEAK_VECTOR;
void tim4_vector() WEAK_VECTOR;
void i2c1_ev_vector() WEAK_VECTOR;
void i2c1_er_vector() WEAK_VECTOR;
void i2c2_ev_vector() WEAK_VECTOR;
void i2c2_er_vector() WEAK_VECTOR;
void spi1_vector() WEAK_VECTOR;
void spi2_vector() WEAK_VECTOR;
void usart1_vector() WEAK_VECTOR;
void usart2_vector() WEAK_VECTOR;
void usart3_vector() WEAK_VECTOR;
void exti15_10_vector() WEAK_VECTOR;
void rtc_alarm_vector() WEAK_VECTOR;
void otg_fs_wkup_vector() WEAK_VECTOR;
void tim8brk_vector() WEAK_VECTOR;
void tim8up_vector() WEAK_VECTOR;
void tim8trg_vector() WEAK_VECTOR;
void tim8cc_vector() WEAK_VECTOR;
void adc3_vector() WEAK_VECTOR;
void fsmc_vector() WEAK_VECTOR;
void sdio_vector() WEAK_VECTOR;
void tim5_vector() WEAK_VECTOR;
void spi3_vector() WEAK_VECTOR;
void uart4_vector() WEAK_VECTOR;
void uart5_vector() WEAK_VECTOR;
void tim6_vector() WEAK_VECTOR;
void tim7_vector() WEAK_VECTOR;
void dma2_channel1_vector() WEAK_VECTOR;
void dma2_channel2_vector() WEAK_VECTOR;
void dma2_channel3_vector() WEAK_VECTOR;
void dma2_channel4_5_vector() WEAK_VECTOR;

/*
typedef void (*vector_func_t)();
__attribute__((section(".vectors"))) vector_func_t g_vectors[] =
{
  (vector_func_t)(&g_stack[STACK_SIZE-8]), // initial stack pointer
  reset_vector,
  nmi_vector,
  hardfault_vector,
  memmanage_vector,
  busfault_vector,
  usagefault_vector,
  0, 0, 0, 0,  
  svcall_vector,
  dbgmon_vector,
  0,                                      
  pendsv_vector,
  systick_vector,
  wwdg_vector,       // 0
  pvd_vector,        
  tamper_vector,
  rtc_vector,
  flash_vector,
  rcc_vector,
  exti0_vector,
  exti1_vector,
  exti2_vector,
  exti3_vector,
  exti4_vector,      // 10
  dma1_channel1_vector,
  dma1_channel2_vector,
  dma1_channel3_vector,
  dma1_channel4_vector,
  dma1_channel5_vector,
  dma1_channel6_vector,
  dma1_channel7_vector,
  adc_vector,
  usb_hp_can1_tx_vector,
  usb_lp_can1_rx0_vector, // 20
  can1_rx1_vector,
  can1_sce_vector,
  exti9_5_vector, // 23
  tim1brk_vector,
  tim1up_vector,
  tim1trg_vector,
  tim1cc_vector,
  tim2_vector,
  tim3_vector,
  tim4_vector, // 30
  i2c1_ev_vector,
  i2c1_er_vector,
  i2c2_ev_vector,
  i2c2_er_vector,
  spi1_vector,
  spi2_vector,
  usart1_vector,
  usart2_vector,
  usart3_vector,
  exti15_10_vector, // 40
  rtc_alarm_vector,
  otg_fs_wkup_vector,
  tim8brk_vector,
  tim8up_vector,
  tim8trg_vector,
  tim8cc_vector,
  adc3_vector,
  fsmc_vector,
  sdio_vector,
  tim5_vector, // 50
  spi3_vector,
  uart4_vector,
  uart5_vector,
  tim6_vector,
  tim7_vector,
  dma2_channel1_vector,
  dma2_channel2_vector,
  dma2_channel3_vector,
  dma2_channel4_5_vector, // 59
};
*/
#endif
