#include "dmxl.h"
#include <string.h>
#include "systime.h"
#include "pin.h"
#include <stdio.h>
//#include "state.h"
#include "delay.h"

#define DMXL_PORTB_TXE_PIN 5
#define DMXL_PORTB_TX_PIN  6
#define DMXL_PORTB_RX_PIN  7

// remap alternate function so that USART1 uses PB6 and PB7

typedef enum 
{
  DMXL_PORT_STATE_IDLE = 0,
  DMXL_PORT_STATE_TX_REG,
  DMXL_PORT_STATE_POLL_TX_EN,  // assert tx-enable on the rs485 bus
  DMXL_PORT_STATE_POLL_TX,
  DMXL_PORT_STATE_POLL_TX_LAST,
  DMXL_PORT_STATE_POLL_TX_DIS,
  DMXL_PORT_STATE_POLL_RX,
  DMXL_PORT_STATE_POLL_RX_WAIT
} dmxl_port_state_t;

typedef enum
{
  DMXL_PARSER_STATE_PREAMBLE_0 = 0,
  DMXL_PARSER_STATE_PREAMBLE_1,
  DMXL_PARSER_STATE_ID,
  DMXL_PARSER_STATE_LENGTH,
  DMXL_PARSER_STATE_ERROR,
  DMXL_PARSER_STATE_PARAMETER,
  DMXL_PARSER_STATE_CHECKSUM,
  DMXL_PARSER_STATE_DONE
} dmxl_parser_state_t;

#define DMXL_CHAIN_TXBUF_LEN 16
#define DMXL_CHAIN_RXBUF_LEN 32 

typedef struct
{
  uint32_t num_nodes;
  uint32_t id_base;
  dmxl_port_state_t port_state;
  dmxl_parser_state_t parser_state;
  uint32_t t_state;
  uint8_t tx_byte_idx;
  uint16_t polling_id;
  uint8_t txbuf[DMXL_CHAIN_TXBUF_LEN];
  volatile uint8_t rxbuf[DMXL_CHAIN_RXBUF_LEN];
  uint8_t tx_pkt_len;
  volatile uint8_t rx_rpos, rx_wpos;
  uint8_t rx_csum, rx_len, rx_pkt_wpos, rx_status;
  uint8_t rx_pkt[DMXL_CHAIN_RXBUF_LEN];
} dmxl_chain_t;

dmxl_chain_t g_dmxl_chain =
{ 2, 1, 0, 0, 0, 0, 0, {0}, {0}, 0, 0, 0, 0, 0, 0, 0, {0} };
//{ 5, 1, 0, 0, 0, GPIOD, USART2, GPIOD,  2, 0, 0, {0}, {0}, 0, 0, 0, 0, 0, 0, 0, {0} };

uint32_t g_dmxl_last_poll_time = 0;

////////////////////////////////////////////////////////////////////////////

void usart1_vector()
{
  volatile uint8_t __attribute__((unused)) sr = USART1->SR; // clear errors
  //printf("p%d rx 0x%02x\r\n", dmxl_port, byte);
  dmxl_chain_t *c = &g_dmxl_chain;
  c->rxbuf[c->rx_wpos] = USART1->DR;
  if (++c->rx_wpos >= DMXL_CHAIN_RXBUF_LEN)
    c->rx_wpos = 0;
}

void dmxl_init()
{
  printf("dmxl_init()\r\n");
  //serial_init();
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  pin_set_output(GPIOB, DMXL_PORTB_TXE_PIN, 0);
  AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
  pin_set_alternate_function(GPIOB, DMXL_PORTB_TX_PIN, true, PIN_PULL_NONE);
  pin_set_alternate_function(GPIOB, DMXL_PORTB_RX_PIN, false, PIN_PULL_NONE);
  USART1->CR1 &= ~USART_CR1_UE;
  USART1->CR1 |=  USART_CR1_TE | USART_CR1_RE;
  USART1->BRR  = (((uint16_t)4) << 4) | 8; // apb2 is 72 mhz; we want 1M baud
  USART1->CR1 |=  USART_CR1_UE | USART_CR1_RXNEIE;
  NVIC_SetPriority(USART1_IRQn, 4);
  NVIC_EnableIRQ(USART1_IRQn);
  /*
  uint8_t status_return_level = 1;
  for (int i = 0; i < DMXL_NUM_CHAINS; i++)
    for (int j = 0; j < g_dmxl_chains[i].num_nodes; j++)
      dmxl_set_regs(i, j+1, 16, 1, &status_return_level);
      */
}

void dmxl_stuff_poll_pkt(dmxl_chain_t *c)
{
  // read 8 bytes starting at addr 36
  c->txbuf[0] = 0xff;
  c->txbuf[1] = 0xff;
  c->txbuf[2] = c->polling_id;
  c->txbuf[3] = 4; // dmxl pkt length = 4
  c->txbuf[4] = 2; // instruction: "read data"
  c->txbuf[5] = 36; // start address
  c->txbuf[6] = 8; // read 8 bytes
  uint8_t csum = 0;
  for (int i = 2; i < 7; i++)
    csum += c->txbuf[i];
  c->txbuf[7] = ~csum;
  c->tx_pkt_len = 8;
}

static void dmxl_process_byte(dmxl_chain_t *c, const uint8_t b)
{
  //printf("0x%02x\r\n", b);
  switch (c->parser_state)
  {
    case DMXL_PARSER_STATE_PREAMBLE_0:
      if (b == 0xff)
        c->parser_state = DMXL_PARSER_STATE_PREAMBLE_1;
      break;
    case DMXL_PARSER_STATE_PREAMBLE_1:
      if (b == 0xff)
        c->parser_state = DMXL_PARSER_STATE_ID;
      else
        c->parser_state = DMXL_PARSER_STATE_PREAMBLE_0;
      break;
    case DMXL_PARSER_STATE_ID:
      c->rx_csum = b;
      c->parser_state = DMXL_PARSER_STATE_LENGTH;
      break;
    case DMXL_PARSER_STATE_LENGTH:
      c->rx_len = b - 2;
      c->rx_csum += b;
      c->parser_state = DMXL_PARSER_STATE_ERROR;
      break;
    case DMXL_PARSER_STATE_ERROR:
      //g_state.dynamixel_error_status[i] = b; // save for global state
      c->rx_status = b; 
      c->rx_csum += b;
      c->rx_pkt_wpos = 0;
      if (c->rx_len)
        c->parser_state = DMXL_PARSER_STATE_PARAMETER;
      else
        c->parser_state = DMXL_PARSER_STATE_CHECKSUM;
      break;
    case DMXL_PARSER_STATE_PARAMETER:
      c->rx_csum += b;
      c->rx_pkt[c->rx_pkt_wpos] = b;
      if (c->rx_pkt_wpos == c->rx_len - 1)
        c->parser_state = DMXL_PARSER_STATE_CHECKSUM;
      if (c->rx_pkt_wpos < DMXL_CHAIN_RXBUF_LEN - 1)
        c->rx_pkt_wpos++;
      break;
    case DMXL_PARSER_STATE_CHECKSUM:
      if (((uint8_t)(~c->rx_csum)) == b)
      {
        int dmxl_idx = c->polling_id - 1;
        uint16_t angle = (c->rx_pkt[1] << 8) | c->rx_pkt[0];
        //int16_t vel = (c->rx_pkt[3] << 8) | c->rx_pkt[2];
        //int16_t load = (c->rx_pkt[5] << 8) | c->rx_pkt[4];
        //uint8_t voltage = c->rx_pkt[6];
        //uint8_t temp = c->rx_pkt[7];
        /*
        if (vel & 0x400)
        {
          vel -= 0x400;
          vel *= -1;
        }
        if (load & 0x400)
        {
          load -= 0x400;
          load *= -1;
        }
        */
        printf("dmxl %d angle = %05d\r\n", dmxl_idx, (int)angle);

#if 0
        g_state.dmxl_status[dmxl_idx]  = c->rx_status;
        g_state.dmxl_angle[dmxl_idx]   = (float)angle / 2048.0f * 3.14159f - 3.14159f;
        g_state.dmxl_vel[dmxl_idx]     = (float)vel * 0.114f * 60.0f * 3.14159f / 180.0f;
        g_state.dmxl_load[dmxl_idx]    = (float)load * 4.1f / 2047.0f; // amps ?
        g_state.dmxl_voltage[dmxl_idx] = (float)voltage * 0.1f;
        g_state.dmxl_temp[dmxl_idx  ]  = (float)temp;
        c->parser_state = DMXL_PARSER_STATE_DONE;
#endif
        /*
        printf("  angle: %d\r\n", g_state.dmxl_angle[dmxl_idx]);
        printf("    vel: %d\r\n", (int16_t)g_state.dmxl_vel[dmxl_idx]);
        printf("   load: %d\r\n", (int16_t)g_state.dmxl_load[dmxl_idx]);
        printf("  volt: %d\r\n", g_state.dmxl_voltage[dmxl_idx]);
        printf("   temp: %d\r\n", g_state.dmxl_temp[dmxl_idx]);
        */
      }
      else
      {
        //printf("csum failed\r\n");
      }
      break;
    case DMXL_PARSER_STATE_DONE:
      break;
    default:
      break;
  }
}

#define DMXL_POLL_INTERVAL_USEC 1000000
void dmxl_tick()
{
  volatile uint32_t t = systime_usecs();
  bool start_poll = false;
  if (t - g_dmxl_last_poll_time > DMXL_POLL_INTERVAL_USEC)
  {
    start_poll = true;
    //printf("%d dmxl poll\r\n", (int)SYSTIME);
    if (g_dmxl_last_poll_time)
      g_dmxl_last_poll_time += DMXL_POLL_INTERVAL_USEC;
    else
      g_dmxl_last_poll_time = t;
  }

  // process the rx ring
  dmxl_chain_t *c = &g_dmxl_chain;
  while (c->rx_rpos != c->rx_wpos)
  {
    dmxl_process_byte(c, c->rxbuf[c->rx_rpos++]);
    __disable_irq();
    if (c->rx_rpos >= DMXL_CHAIN_RXBUF_LEN)
      c->rx_rpos = 0;
    __enable_irq();
  }

  // run the port state machine
  switch (c->port_state)
  {
    case DMXL_PORT_STATE_IDLE:
      if (start_poll)
      {
        USART1->CR1 &= ~USART_CR1_RE; 
        c->port_state = DMXL_PORT_STATE_POLL_TX_EN;
        c->polling_id = c->id_base; // start with first servo
        c->t_state = t;
        dmxl_stuff_poll_pkt(c);
        pin_set_output_state(GPIOB, DMXL_PORTB_TXE_PIN, 1); // assert TXE
      }
      break;
    case DMXL_PORT_STATE_POLL_TX_EN:
      if (t - c->t_state > 3)
      {
        //printf("starting dmxl poll\r\n");
        c->tx_byte_idx = 0;
        USART1->DR = c->txbuf[0];
        c->port_state = DMXL_PORT_STATE_POLL_TX;
        c->t_state = systime_usecs();
      }
      break;
    case DMXL_PORT_STATE_POLL_TX:
      if ((USART1->SR & USART_SR_TXE) || systime_usecs() - c->t_state > 100)
      {
        if (c->tx_byte_idx >= c->tx_pkt_len - 1)
        {
          c->port_state = DMXL_PORT_STATE_POLL_TX_LAST;
          c->t_state = systime_usecs();
        }
        else
          USART1->DR = c->txbuf[++c->tx_byte_idx];
      }
      break;
    case DMXL_PORT_STATE_POLL_TX_LAST:
      if ((USART1->SR & USART_SR_TC) || systime_usecs() - c->t_state > 100)
      {
        c->port_state = DMXL_PORT_STATE_POLL_TX_DIS;
        c->t_state = systime_usecs();
        c->parser_state = DMXL_PARSER_STATE_PREAMBLE_0;
      }
      break;
    case DMXL_PORT_STATE_POLL_TX_DIS:
      if (systime_usecs() - c->t_state > 2)
      {
        pin_set_output_state(GPIOB, DMXL_PORTB_TXE_PIN, 0); // de-assert TXE
        c->port_state = DMXL_PORT_STATE_POLL_RX;
        c->t_state = systime_usecs();
        USART1->CR1 |= USART_CR1_RE; 
        //printf("%d starting rx\r\n", (int)SYSTIME);
      }
      break;
    case DMXL_PORT_STATE_POLL_RX:
      if ((c->parser_state == DMXL_PARSER_STATE_DONE) ||
          (systime_usecs() - c->t_state > 1000)) 
      {
        // if we haven't heard from it by 1ms, we're toast anyway
        //printf("%d exiting rx\r\n", (int)SYSTIME);
        // we need to either start polling the next servo, or we're done.
        c->polling_id++;
        //printf("%d poll complete\r\n", (int)SYSTIME);
        if (c->polling_id > c->num_nodes)
          c->port_state = DMXL_PORT_STATE_IDLE;
        else
          c->port_state = DMXL_PORT_STATE_POLL_RX_WAIT;

        dmxl_stuff_poll_pkt(c);
        USART1->CR1 &= ~USART_CR1_RE; 
        c->t_state = systime_usecs();
      }
      break;
    case DMXL_PORT_STATE_POLL_RX_WAIT:
      if (systime_usecs() - c->t_state > 100)
      {
        c->port_state = DMXL_PORT_STATE_POLL_TX_EN;
        pin_set_output_state(GPIOB, DMXL_PORTB_TXE_PIN, 1); // assert TXE
        c->t_state = systime_usecs();
      }
      break;
    default:
      g_dmxl_chain.port_state = DMXL_PORT_STATE_IDLE;
  }
}

static void dmxl_tx(const uint8_t *payload, const uint8_t payload_len)
{
  if (g_dmxl_chain.port_state != DMXL_PORT_STATE_IDLE)
    return; // don't clobber the polls. TODO: buffer until it's free
  uint8_t pkt[255];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  memcpy(pkt+2, payload, payload_len);
  uint8_t csum = 0;
  for (uint8_t i = 0; i < payload_len; i++)
    csum += payload[i];
  pkt[payload_len+2] = ~csum;
  USART1->CR1 &= ~USART_CR1_RE; // disable receiver during transmit
  pin_set_output_state(GPIOB, DMXL_PORTB_TXE_PIN, 1); // assert txe
  delay_us(5);
  for (int i = 0; i < payload_len+3; i++)
  {
    while (!(USART1->SR & USART_SR_TXE)) { } // spin for tx buffer to clear
    USART1->DR = pkt[i];
  }
  while (!(USART1->SR & USART_SR_TC)) { } // spin for completion of last byte
  delay_us(5);
  pin_set_output_state(GPIOB, DMXL_PORTB_TXE_PIN, 0); // de-assert txe
  USART1->CR1 |= USART_CR1_RE; // re-enable receiver
}

static void dmxl_write_data(const uint8_t id, const uint8_t data_len,
                            const uint8_t start_addr, const uint8_t *data)
{
  uint8_t pkt[255];
  pkt[0] = id;
  pkt[1] = data_len + 3;
  pkt[2] = 3; // instruction: "write data"
  pkt[3] = start_addr;
  for (int i = 0; i < data_len; i++)
    pkt[4+i] = data[i];
  dmxl_tx(pkt, data_len+4);
}

void dmxl_set_regs(const uint8_t id,  const uint8_t start_addr,
                   const uint8_t len, const uint8_t *regs)
{
  dmxl_write_data(id, len, start_addr, regs);
}

void dmxl_set_goals(const uint16_t *goals)
{
  // TODO
}

void dmxl_set_goal(const uint8_t id, const uint16_t goal)
{
  // TODO
}

bool dmxl_busy()
{
  if (g_dmxl_chain.port_state != DMXL_PORT_STATE_IDLE)
    return true;
  return false;
}
