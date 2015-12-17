#include "usb.h"
#include "stm32f103xb.h"
#include <stdbool.h>
#include <stdio.h>
#include "delay.h"
#include "pin.h"
#include <string.h>

extern void usb_ep1_txf_empty() __attribute__((weak));
extern void usb_ep1_tx_complete() __attribute__((weak));
static void usb_set_rx_status(const uint32_t ep, const uint32_t status);
static void usb_set_tx_status(const uint32_t ep, const uint32_t status);


static bool g_usb_config_complete = false;
#define PORTA_USB_DM_PIN 11
#define PORTA_USB_DP_PIN 12

#define PORTC_USB_DISCONNECT_PIN 13

#define USB_TIMEOUT 200000

typedef struct usb_ep_desc
{
  uint8_t  len;
  uint8_t  desc_type;
  uint8_t  ep_addr;
  uint8_t  attr;
  uint16_t max_pkt_size;
  uint8_t  interval;
} __attribute__((packed)) usb_ep_desc_t;

typedef struct usb_iface_desc
{
  uint8_t len;
  uint8_t desc_type;
  uint8_t iface_num;
  uint8_t alt_setting;
  uint8_t num_ep;
  uint8_t iface_class;
  uint8_t iface_subclass;
  uint8_t iface_proto;
  uint8_t iface_str_idx;
  usb_ep_desc_t eps[2];
} __attribute__((packed)) usb_iface_desc_t;

typedef struct usb_config_desc
{
  uint8_t  len;
  uint8_t  desc_type;
  uint16_t total_len;
  uint8_t  num_ifaces;
  uint8_t  config_val;
  uint8_t  config_str_idx;
  uint8_t  attributes;
  uint8_t  max_power;
  usb_iface_desc_t ifaces[1];
} __attribute__((packed)) usb_config_desc_t;

#define USB_DESC_TYPE_DEVICE   0x1
#define USB_DESC_TYPE_CONFIG   0x2
#define USB_DESC_TYPE_STRING   0x3
#define USB_DESC_TYPE_IFACE    0x4
#define USB_DESC_TYPE_ENDPOINT 0x5

#define USB_EP_TYPE_BULK       0x2

#define USB_PROTO 0x0200
#define USB_DEV_CLASS_CUSTOM    0xff
#define USB_DEV_SUBCLASS_CUSTOM 0xff
#define USB_DEV_PROTO_CUSTOM    0xff
#define USB_MAX_EP0_PKT_SIZE 64
#define USB_VID 0xf055
#define USB_PID 2
#define USB_BCD_DEV 0
#define USB_NO_MFGR_STR 0
#define USB_NO_PROD_STR 0
#define USB_NO_SERIAL_STR 0

typedef struct usb_device_desc
{
  uint8_t  len;
  uint8_t  desc_type;
  uint16_t usb_ver;
  uint8_t  dev_class;
  uint8_t  dev_subclass;
  uint8_t  dev_protocol;
  uint8_t  max_ep0_pkt_size;
  uint16_t  vid;
  uint16_t  pid;
  uint16_t  bcdDevice; // wtf
  uint8_t   man_str_idx;
  uint8_t   prod_str_idx;
  uint8_t   ser_num_str_idx;
  uint8_t   num_config;
} __attribute__((packed)) usb_device_desc_t;

typedef struct usb_lang_list_desc
{
  uint8_t len;
  uint8_t desc_type;
  uint16_t langs[1]; // todo... allow more languages... someday
} __attribute__((packed)) usb_lang_list_desc_t;

#define USB_LANG_ID_ENGLISH_USA 0x0409

typedef struct usb_string_desc
{
  uint8_t  len;
  uint8_t  desc_type;
  char    *str;
} __attribute__((packed)) usb_string_desc_t;

#define USB_VENDOR_STRING  1
#define USB_PRODUCT_STRING 2

static const struct usb_device_desc g_usb_device_desc =
{
  sizeof(struct usb_device_desc),
  USB_DESC_TYPE_DEVICE,
  USB_PROTO,
  USB_DEV_CLASS_CUSTOM,
  USB_DEV_SUBCLASS_CUSTOM,
  USB_DEV_PROTO_CUSTOM,
  USB_MAX_EP0_PKT_SIZE,
  USB_VID,
  USB_PID,
  USB_BCD_DEV,
  USB_VENDOR_STRING,
  USB_PRODUCT_STRING,
  USB_NO_SERIAL_STR,
  1 // one configuration
};

static const struct usb_config_desc g_usb_config_desc = 
{ 
  9, // length of configuration descriptor
  USB_DESC_TYPE_CONFIG, // configuration descriptor type
  9 + 9 + 2 * sizeof(struct usb_ep_desc), // total length 
  1, // number of interfaces
  1, // configuration value
  0, // no string
  0x80, // attributes
  50, // max power
  {
    {
      9, // length of this interface descriptor
      USB_DESC_TYPE_IFACE, // interface descriptor type
      0, // interface number
      0, // alternate setting
      2, // no extra endpoints
      0xff, // custom class code,
      0xff, // custom subclass code,
      0xff, // custom protocol code
      0, // no string 
      { 
        {
          sizeof(struct usb_ep_desc), // length of endpoint descriptor
          USB_DESC_TYPE_ENDPOINT,
          0x81, // EP1 IN
          USB_EP_TYPE_BULK,
          64, // max packet size
          1 // interval. ignored for bulk endpoints anyway.
        },
        {
          sizeof(struct usb_ep_desc), // length of endpoint descriptor
          USB_DESC_TYPE_ENDPOINT,
          0x02, // EP2 OUT
          USB_EP_TYPE_BULK, // bulk endpoint
          64, // max packet size
          1 // interval. ignored for bulk endpoints anyway.
        }
      }
    }
  }
};

const struct usb_lang_list_desc g_usb_lang_list_desc =
{
  sizeof(struct usb_lang_list_desc),
  USB_DESC_TYPE_STRING,
  {
    USB_LANG_ID_ENGLISH_USA
  }
};

static const char *g_usb_vendor_str = "OSRF";
static const char *g_usb_product_str = "roscar";

static uint8_t g_usb_setup_pkt_buf[256];

////////////////////////////////////////////////////////////////

uint8_t usb_stuff_desc_string(const char *str)
{
  const uint8_t len = strlen(str);
  g_usb_setup_pkt_buf[0] = 2 + 2 * len;
  g_usb_setup_pkt_buf[1] = USB_DESC_TYPE_STRING;
  for (int i = 0; i < len; i++)
  {
    g_usb_setup_pkt_buf[2 + i*2] = str[i];
    g_usb_setup_pkt_buf[3 + i*2] = 0;
  }
  return 2 + 2 * len;
}

static uint8_t g_usb_new_addr = 0;

void usb_rx_setup(const uint8_t *buf, const unsigned len)
{
  const uint8_t  req_type  = buf[0];
  const uint8_t  req       = buf[1];
  const uint16_t req_val   = buf[2] | (buf[3] << 8);
  //const uint16_t req_index = buf[4] | (buf[5] << 8);
  const uint16_t req_count = buf[6] | (buf[7] << 8);

  /*
  printf("ep0 setup type %02x req %02x val %04x index %04x count %04x\r\n",
         req_type, req, req_val, req_index, req_count);
  */
  if (req_type == 0x80 && req == 0x06) // get descriptor
  {
    const void *p_desc = NULL;
    uint16_t desc_len = 0;
    if (req_val == 0x0100) // get device descriptor
    {
      p_desc = &g_usb_device_desc;
      desc_len = sizeof(g_usb_device_desc);
    }
    else if (req_val == 0x0200) // get configuration descriptor
    {
      p_desc = &g_usb_config_desc;
      desc_len = sizeof(g_usb_config_desc);
    }
    else if (req_val == 0x0300) // get string language list
    {
      p_desc = &g_usb_lang_list_desc;
      desc_len = sizeof(g_usb_lang_list_desc);
    }
    else if (req_val == 0x0301) // get vendor string
    {
      p_desc = g_usb_setup_pkt_buf;
      desc_len = usb_stuff_desc_string(g_usb_vendor_str);
    }
    else if (req_val == 0x0302) // get product string
    {
      p_desc = g_usb_setup_pkt_buf;
      desc_len = usb_stuff_desc_string(g_usb_product_str);
    }
    ////////////////////
    if (p_desc)
    {
      int tx_len = desc_len < req_count ? desc_len : req_count;
      usb_tx(0, p_desc, tx_len);
    }
    else
    {
      //printf("TRAP!!! unknown descriptor request: 0x%04x\r\n", req_val);
      //while(1) { } // IT'S A TRAP
      //usb_tx_stall(0);
      usb_set_tx_status(0, 0x1);
      usb_set_rx_status(0, 0x3);
      //printf("unknown descriptor 0x%04x requested\r\n", req_val);
    }
  }
  else if (req_type == 0x00 && req == 0x05) // set address
  {
    g_usb_new_addr = (uint8_t)req_val; // we can't set address until txn done
    usb_tx(0, NULL, 0);
  }
  else if (req_type == 0x00 && req == 0x09) // set configuration
  {
    // todo: call into mac-specific function to set up endpoints, etc.
    usb_tx(0, NULL, 0);
  }
  else
  {
    printf("unknown setup rx: req_type = 0x%02x, req = 0x%02x\r\n",
           req_type, req);
    printf("trapping...\r\n");
    while(1) { } // IT'S A TRAP
  }
}

//#define USB_PKTBUF ((uint32_t)0x40006000)
#define USB_TX_ADDR(ep) ((uint32_t *)(USB_PMAADDR + (ep*8  )*2))
#define USB_TX_CNT(ep)  ((uint32_t *)(USB_PMAADDR + (ep*8+2)*2))
#define USB_RX_ADDR(ep) ((uint32_t *)(USB_PMAADDR + (ep*8+4)*2))
#define USB_RX_CNT(ep)  ((uint32_t *)(USB_PMAADDR + (ep*8+6)*2))

// we'll have 4 descriptors: EP0 control TX/RX, EP1 bulk RX, EP2 bulk TX
// so our descriptor table will be 8*4 = 0x20 bytes long
// we'll start putting 64-byte buffers after that
#define USB_EP0_TX_BUF 0x20
#define USB_EP0_RX_BUF 0x60
#define USB_EP1_RX_BUF 0xc0
#define USB_EP2_TX_BUF 0x100

static void usb_reset();

void usb_init()
{
  RCC->APB1RSTR |= RCC_APB1RSTR_USBRST; // reset the USB subsystem
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // power up the AF i/o machinery
  RCC->APB1RSTR &= ~RCC_APB1RSTR_USBRST; // de-assert USB core reset

  // force disconnect from host OS
  pin_set_output(GPIOC, PORTC_USB_DISCONNECT_PIN, 1);
  delay_ms(50);
  // connect (activate pullup on usb line)
  pin_set_output(GPIOC, PORTC_USB_DISCONNECT_PIN, 0);
  pin_set_alternate_function(GPIOA, PORTA_USB_DM_PIN, false, PIN_PULL_UP);
  pin_set_alternate_function(GPIOA, PORTA_USB_DP_PIN, false, PIN_PULL_UP);
  RCC->APB1ENR |= RCC_APB1ENR_USBEN; // turn on USB OTG FS clock gate
  USB->CNTR = USB_CNTR_FRES; // assert USB reset and power-up analog stuff
  delay_us(2); // wait at least 1us, according to datasheet
  USB->CNTR = 0; // clear USB reset
  USB->ISTR = 0; // clear any pending interrupts
  USB->BTABLE = 0;
  usb_reset();
  USB->CNTR |= USB_CNTR_CTRM    |
               USB_CNTR_RESETM  |
               USB_CNTR_PMAOVRM |
               USB_CNTR_WKUPM   ;
               //USB_CNTR_SUSPM   |
               //USB_CNTR_ERRM    ;
  // enable USB interrupts
  NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 1);
  NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1);
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  // wait for reset interrupt
}

static void usb_reset()
{
  printf("URESET\r\n");
  USB->ISTR &= ~USB_ISTR_RESET;
  *(USB_TX_ADDR(0)) = USB_EP0_TX_BUF;
  *(USB_TX_CNT(0)) = 0;
  *(USB_RX_ADDR(0)) = USB_EP0_RX_BUF;
  *(USB_RX_CNT(0))  = 0x8400; // this rx buffer is 64 bytes
  usb_set_rx_status(0, 0x3); // enabled for reception
  usb_set_tx_status(0, 0x2); // respond to TX requests with NAK
  USB->DADDR = USB_DADDR_EF; // enable function
}

static void usb_set_rx_status(const uint32_t ep, const uint32_t status)
{
  volatile uint32_t toggle_mask = USB_EP_CTR_RX | USB_EP_CTR_TX | ep;
  volatile uint16_t *epr = NULL;
  if (ep == 0)
  {
    epr = &USB->EP0R;
    toggle_mask |= USB_EP_CONTROL;
  }
  if (!epr)
  {
    printf("woah! unknown EP: %d\r\n", (int)ep);
    while (1) { } // IT'S A TRAP!!!
  }
  if (((*epr >> 12) & 0x1) != (status & 0x1))
    toggle_mask |= USB_EP0R_STAT_RX_0;
  if (((*epr >> 12) & 0x2) != (status & 0x2))
    toggle_mask |= USB_EP0R_STAT_RX_1;
  *epr = toggle_mask;
}

static void usb_set_tx_status(const uint32_t ep, const uint32_t status)
{
  uint32_t toggle_mask = USB_EP_CTR_RX | USB_EP_CTR_TX | ep;
  volatile uint16_t *epr = NULL;
  if (ep == 0)
  {
    epr = &USB->EP0R;
    toggle_mask |= USB_EP_CONTROL;
  }
  if (!epr)
  {
    printf("woah! unknown EP: %d\r\n", (int)ep);
    while (1) { } // IT'S A TRAP!!!
  }
  if (((*epr >> 4) & 0x1) != (status & 0x1))
    toggle_mask |= USB_EP0R_STAT_TX_0;
  if (((*epr >> 4) & 0x2) != (status & 0x2))
    toggle_mask |= USB_EP0R_STAT_TX_1;
  *epr = toggle_mask;
}

bool usb_tx(const uint8_t ep, const uint8_t *payload, const uint8_t payload_len)
{
  //printf("usb tx %d bytes\r\n", payload_len);
  //volatile uint16_t ep0r = USB->EP0R;
  //printf("  tx start ep0r = 0x%04x\r\n", ep0r);

  /*
  if (payload_len == 0)
  {
    printf("can't tx 0 bytes!\r\n");
    return false;
  }
  */
  if (ep != 0 && !g_usb_config_complete)
  {
    printf("usb can't tx on EP%d before enumeration\r\n", ep);
    return false; // can't transmit yet
  }
  uint32_t *usb_tx_sram = NULL;
  if (ep == 0)
    usb_tx_sram = (uint32_t *)(USB_PMAADDR + 2 * USB_EP0_TX_BUF);
  if (usb_tx_sram == NULL)
  {
    printf("usb can't tx on EP%d\r\n", ep);
    return false;
  }
  for (int i = 0; i < payload_len/2 && i < 32; i++)
  {
    // todo: care about non-even length transactions
    *usb_tx_sram = *((uint16_t *)&payload[i*2]);
    usb_tx_sram++;
  }
  *USB_TX_CNT(ep) = payload_len;
  usb_set_tx_status(ep, 0x3); // set TX state to VALID
  //printf("  ep0r = 0x%04x\r\n", USB->EP0R);
  //volatile uint16_t ep_reg = USB->EP0R;
  //printf("  tx set epr to 0x%04x\r\n", ep_reg);
  return true;
}

bool usb_txf_avail(const uint8_t ep, const uint8_t nbytes)
{
#if 0
  if (ep >= 4)
    return false;
  return USB_INEP(ep)->DTXFSTS >= nbytes / 4;
#endif
  return true;
}

bool usb_tx_stall(const uint8_t ep)
{
  //printf("usb tx stall\r\n");
  usb_set_tx_status(ep, 0x1);
#if 0
  if (ep >= 4)
    return false; // adios amigo
  USB_INEP(ep)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
#endif
  return true;
}

void usb_hp_can1_tx_vector()
{
  printf("usb HP vector\r\n");
}

static void usb_txrx_complete()
{
  // transfer_complete
  //printf("CTR\r\n");
  // check our endpoints to see which one triggered the interrupt
  static volatile uint16_t *eps[1] = { &USB->EP0R };
  for (int ep = 0; ep < 1; ep++)
  {
    uint16_t epr = *eps[ep];
    if (epr & USB_EP_CTR_RX) // rx complete
    {
      uint16_t nrx = (*USB_RX_CNT(ep)) & 0x3f;
      static uint16_t rxbuf[32] = {0};
      uint32_t *usb_rx_sram = NULL;
      if (ep == 0)
        usb_rx_sram = (uint32_t *)(USB_PMAADDR + 2 * USB_EP0_RX_BUF);
      for (int i = 0; i < nrx/2 && i < sizeof(rxbuf)/2; i++)
      {
        // todo: care about non-even numbers of RX bytes
        rxbuf[i] = *usb_rx_sram;
        usb_rx_sram++;
      }
      /*
      volatile uint16_t ep0r = USB->EP0R;
      printf("rx ep0r = 0x%04x\r\n", ep0r);
      *eps[ep] &= ~USB_EP_CTR_RX; // clear the RX-complete flag
      {
        volatile uint16_t ep0r = USB->EP0R;
        printf("  rx after clear ep0r = 0x%04x\r\n", ep0r);
      }
      */
      if (ep == 0)
      {
        if (USB->EP0R & USB_EP_SETUP)
        {
          usb_set_rx_status(ep, 0x2); // hold this in STALL while we handle SETUP
          usb_rx_setup((uint8_t *)rxbuf, nrx);
        }
        else
        {
          usb_set_rx_status(ep, 0x3); // ready to rx again
          //printf("rx %d non-setup bytes on EP0\r\n", nrx);
        }
        USB->EP0R = USB_EP_CTR_TX | // clear the RX flag, leave TX flag intact
                    USB_EP_CONTROL; // remember, this is a control EP
      }
      else
      {
        USB->EP0R = USB_EP_CTR_TX | // clear the RX flag, leave TX flag intact
                    ep; // remember to save the endpoint address
        usb_set_rx_status(ep, 0x3);
        // call user-facing EP RX
        printf("  EP %d rx %d bytes\r\n", ep, nrx);
      }
    }
    if (epr & USB_EP_CTR_TX) // tx complete
    {
      uint32_t next_epr = USB_EP_CTR_RX | ep; // clear TX flag, leave RX intact
      if (ep == 0)
      {
        next_epr |= USB_EP_CONTROL;
        usb_set_rx_status(ep, 0x3);
        if (g_usb_new_addr)
        {
          printf("setting addr %d\r\n", g_usb_new_addr);
          USB->DADDR = USB_DADDR_EF | g_usb_new_addr;
          g_usb_new_addr = 0;
        }
      }
      *eps[ep] = next_epr;
    }
  }
}

void usb_lp_can1_rx0_vector()
{
  //printf("  ep0r = 0x%08x\r\n", USB->EP0R);
  //printf("  istr = %08x\r\n", USB->ISTR);
  if (USB->ISTR & USB_ISTR_CTR)
    usb_txrx_complete();
  else if (USB->ISTR & USB_ISTR_RESET)
    usb_reset();
  else if (USB->ISTR & USB_ISTR_WKUP)
  {
    printf("WKUP\r\n");
    //USB->ISTR &= ~USB_ISTR_WKUP;
  }
  else
  {
    printf("unknown usb reset vector cause! usb istr = 0x%08x\r\n", USB->ISTR);
    while (1) { } // TRAP
  }
}

#if 0
void otg_fs_vector()
{
  //printf("otg vect\r\n");
  //printf("unhandled gintsts = %08x\r\n", (unsigned)USB_OTG_FS->GINTSTS);
  if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST)
  {
    printf("usb reset\r\n");
    g_usbd->DCTL &= ~USB_OTG_DCTL_RWUSIG;
    usb_flush_txfifo(0);
    g_usbd->DCFG |= 0x3 | // peg to full-speed
                    USB_OTG_DCFG_NZLSOHSK; // non-zero length handshake (?)
    for (int i = 0; i < 4; i++)
    {
      USB_OUTEP(i)->DOEPINT = 0xff; // wipe out any pending EP flags
      USB_INEP(i)->DIEPINT = 0xff; // wipe out any pending EP flags
      //USB_OUTEP(i)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
    }
    //g_usbd->DAINT = 0xffffffff; // wipe out any flags (aren't these r/o ?)
    //g_usbd->DAINTMSK = 0x10001; // enable IN0 and OUT0 for control msgs
    g_usbd->DOEPMSK |= USB_OTG_DOEPMSK_STUPM |  // enable setup-done irq
                       USB_OTG_DOEPMSK_EPDM  |  // enable EP-disabled irq
                       USB_OTG_DOEPMSK_XFRCM ;  // enable tx-done irq
    g_usbd->DIEPMSK |= USB_OTG_DIEPMSK_TOM   |  // timeout irq
                       USB_OTG_DIEPMSK_XFRCM |
                       USB_OTG_DIEPMSK_EPDM;
    // set up FIFO sizes, in bytes:
    #define USB_RXFIFO_SIZE     512
    #define USB_TXFIFO_EP0_SIZE 128
    #define USB_TXFIFO_EP1_SIZE 128
     
    USB_OTG_FS->GRXFSIZ = USB_RXFIFO_SIZE / 4; // size is in 32-bit words !
    g_usbd->DCFG &= ~USB_OTG_DCFG_DAD; // zero out the device address fields
    uint32_t usb_ram_addr = USB_RXFIFO_SIZE;
    // the length of this buffer is in 32-bit words, but addr is bytes. argh.
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = ((USB_TXFIFO_EP0_SIZE/4) << 16) |
                                     usb_ram_addr;
    usb_ram_addr += USB_TXFIFO_EP0_SIZE;
    // the length of this buffer is in 32-bit words, but addr is bytes. argh.
    USB_OTG_FS->DIEPTXF[0] = ((USB_TXFIFO_EP1_SIZE/4) << 16) | 
                             usb_ram_addr;

    USB_INEP (0)->DIEPTSIZ = 64;
    USB_OUTEP(0)->DOEPTSIZ = (USB_OTG_DOEPTSIZ_PKTCNT & (1 << 19)) |
                             USB_OTG_DOEPTSIZ_STUPCNT | // allow 3 setup pkt
                             (3 * 8); // transfer size
    USB_INEP (0)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP | // active EP (always 1)
                            USB_OTG_DIEPCTL_SNAK   ; // set NAK bit
    USB_OUTEP(0)->DOEPCTL = USB_OTG_DOEPCTL_EPENA  | // enable endpoint
                            USB_OTG_DOEPCTL_USBAEP | // active EP (always 1)
                            USB_OTG_DOEPCTL_CNAK   ; // clear NAK bit

    USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_USBRST; // clear the flag (rc_w1)
  }
  else if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
  {
    //printf("enum done\r\n");
    USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_ENUMDNE; // clear the flag (rc_w1)
  }
  else if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_SOF)
  {
    USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_SOF; // clear the flag (rc_w1)
  }
  else if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL)
  {
    static uint8_t setup_buf[64] = {0};
    static uint8_t setup_nbytes = 0;
    const volatile uint32_t rx_status = USB_OTG_FS->GRXSTSP;
    const uint8_t epnum = rx_status & USB_OTG_GRXSTSP_EPNUM;
    const uint16_t nbytes = (rx_status & USB_OTG_GRXSTSP_BCNT) >> 4;
    if (nbytes > 64)
    {
      printf("woah there partner. nbytes = %d\r\n", (int)nbytes);
      return;
    }
    if (epnum >= 4)
    {
      printf("woah there partner. epnum = %d\r\n", (int)epnum);
      return;
    }

    //uint8_t dpid = (rx_status & USB_OTG_GRXSTSP_DPID) >> 15;
    const uint8_t pktsts = (rx_status & USB_OTG_GRXSTSP_PKTSTS) >> 17;
    //printf("rx epnum = %d nbytes = %d dpid = %d pktsts = %d\r\n",
    //       epnum, nbytes, dpid, pktsts);
    uint8_t buf[72] = {0}; // bigger than the largest possible usb packet...
    int wpos = 0;
    uint32_t w = 0;
    for (int rword = 0; rword < nbytes/4; rword++)
    {
      w = *USB_FIFO(epnum);
      buf[wpos++] =  w        & 0xff;
      buf[wpos++] = (w >>  8) & 0xff;
      buf[wpos++] = (w >> 16) & 0xff;
      buf[wpos++] = (w >> 24) & 0xff;
    }
    if (nbytes % 4)
    {
      w = *USB_FIFO(epnum);
      for (int partial = 0; partial < nbytes % 4; partial++)
      {
        buf[wpos++] = w & 0xff;
        w >>= 8;
      }
    }

    if (epnum == 0)
    {
      //printf("RXFLVL ep0 pktsts = %d\r\n", pktsts);
      if (pktsts == 6)
      {
        memcpy(setup_buf, buf, nbytes);
        setup_nbytes = nbytes;
      }
      else if (pktsts == 4)
      {
        usb_setup_rx(setup_buf, setup_nbytes);
        USB_OUTEP(0)->DOEPCTL = USB_OTG_DOEPCTL_EPENA  | // enable endpoint
                                USB_OTG_DOEPCTL_USBAEP | // active endpoint (?)
                                USB_OTG_DOEPCTL_CNAK   ; // clear NAK bit
      }
    }
    else 
    {
      //printf("pktsts %d nbytes %d\r\n", pktsts, nbytes);
      if (nbytes) // ignore transfer-complete notifications
        usb_rx_internal(epnum, buf, nbytes);
    }
  }
  else if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IEPINT)
  {
    //printf("iepint vec = 0x%08x\r\n", (unsigned)g_usbd->DAINT);
    //if (usb_ep1_txf_empty)
    //  usb_ep1_txf_empty();

    if (g_usbd->DAINT & 0x2)
    {
      // EP1 is firing an IRQ
      if (USB_INEP(1)->DIEPINT & USB_OTG_DIEPINT_XFRC)
      {
        USB_INEP(1)->DIEPINT = USB_OTG_DIEPINT_XFRC; // clear the flag
        // fire the handler
        if (usb_ep1_tx_complete)
          usb_ep1_tx_complete();
      }
    }
  }
  else
  {
    printf("unhandled gintsts = %08x\r\n", (unsigned)USB_OTG_FS->GINTSTS);
  }
}
#endif
#if 0
static void usb_rx_ep0(const uint8_t *data, const uint8_t len)
{
  /*
  printf("  EP0 rx %d bytes:\r\n", len);
  for (int i = 0; i < len; i++)
    printf("    %d: 0x%02x\r\n", i, data[i]);
  */
  if (USB->EP0R & USB_EP_SETUP)
    usb_rx_setup(data, len);
}
#endif

//static USB_TypeDef * const g_usbd = 
//  (USB_TypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + 
//                            USB_OTG_DEVICE_BASE);
#if 0
#define USB_INEP(i)  ((USB_OTG_INEndpointTypeDef *)(( uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))        
#define USB_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))
#define USB_WAIT_RET_BOOL(cond) \
  do { \
    int count = 0; \
    do { \
      if ( ++count > USB_TIMEOUT ) \
        return false; \
    } while (cond); \
  } while (0)
#define USB_FIFO_BASE ((uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE))
#define USB_FIFO(i) ((uint32_t *)(USB_FIFO_BASE + 1024 * i))
#endif
#if 0
bool usb_flush_txfifo(uint32_t fifo)
{
  USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (fifo << 5);
  USB_WAIT_RET_BOOL((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == 
                    USB_OTG_GRSTCTL_TXFFLSH);
  return true;
}

bool usb_flush_rxfifo()
{
  USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
  USB_WAIT_RET_BOOL((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) ==
                    USB_OTG_GRSTCTL_RXFFLSH);
  return true;
}
#endif

#if 0
bool usb_setup_rx(const uint8_t *buf, const uint32_t len)
{
  const uint8_t  request_type = buf[0];
  const uint8_t  request = buf[1];
  const uint16_t request_value = buf[2] | ((uint16_t)buf[3] << 8);
  const uint16_t request_len   = buf[6] | ((uint16_t)buf[7] << 8);
  printf("setup rx type=0x%02x request=0x%02x request_value=0x%02x len=%d\r\n",
         request_type, request, request_value, request_len);
#if 0
  if (request_type == 0x0 && request == 0x5) // set address
  {
    const uint16_t addr = buf[2];
    g_usbd->DCFG |= (addr << 4);
    usb_tx(0, NULL, 0);
    printf("set addr %d\r\n", addr);
    return true;
    //return usb_tx(0, NULL, 0); // send an empty status packet back
  }
  else if (request_type == 0x80 && request == 0x0) // get status
  {
    uint8_t status[2] = { 0, 0 };
    return usb_tx(0, status, sizeof(status)); // send an empty status packet back
  }
  else if (request_type == 0x80 && request == 0x6) // get descriptor
  {
    uint8_t *pdesc = NULL;
    uint8_t desc_len = 0;
    //printf("desc req val = 0x%04x\r\n", (unsigned)request_value);
    if (request_value == 0x0100) // request device descriptor
    {
      pdesc = g_usb_device_descriptor;
      desc_len = sizeof(g_usb_device_descriptor);
    }
    else if (request_value == 0x0200) // request configuration descriptor
    {
      pdesc = (uint8_t *)&g_usb_config_desc;
      desc_len = g_usb_config_desc.total_length;
    }
    if (pdesc)
    {
      int write_len = request_len < desc_len ? request_len : desc_len;
      // todo: loop until done sending it, if needed
      return usb_tx(0, pdesc, write_len);
    }
    else
      return usb_tx_stall(0); // we don't know how to handle this request
  }
  else if (request_type == 0 && request == 0x9) // set configuration
  {
    if (request_value != 0)
      return usb_tx_stall(0);
    // enable the IN endpoint for configuration #0
    //USB_INEP(1)->DIEPTSIZ = (1 << 19) | payload_len; // set outbound pkt size
    //USB_INEP(1)->DIEPINT  = USB_OTG_DIEPINT_TXFE; // interrupt on TXF empty
    USB_INEP(1)->DIEPINT  = USB_OTG_DIEPINT_XFRC; // interrupt on TXF empty
    USB_INEP(1)->DIEPTSIZ = 64; 
    USB_INEP(1)->DIEPCTL = USB_OTG_DIEPCTL_SNAK    | // set NAK bit
                           (1 << 22)               | // use txfifo #1
                           USB_OTG_DIEPCTL_EPTYP_1 | // bulk transfer
                           USB_OTG_DIEPCTL_USBAEP  | // active EP (always 1)
                           USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                           64; // max packet size

    // enable the OUT endpoint for configuration #0
    USB_OUTEP(2)->DOEPTSIZ = (1 << 19) | 64; // buffer one full-length packet
    USB_OUTEP(2)->DOEPCTL = USB_OTG_DOEPCTL_EPENA  | // enable endpoint
                            USB_OTG_DOEPCTL_USBAEP | // active endpoint (always 1)
                            USB_OTG_DOEPCTL_CNAK   | // clear NAK bit
                            USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
                            USB_OTG_DOEPCTL_EPTYP_1 |
                            64; // max packet size = 64 bytes
    g_usbd->DAINTMSK = 0x40002; // OUT2 and IN1 IRQ . used to be 0x10001
    // done configuring endpoints; now we'll send an empty status packet back
    g_usb_config_complete = true;
    uint8_t bogus[64] = {0};
    usb_tx(1, bogus, sizeof(bogus)); // kick-start the FIFO low interrupt
    return usb_tx(0, NULL, 0); 
  }
  printf("unhandled usb_setup_rx len %u\r\n  ", (unsigned)len);
  for (int i = 0; i < len; i++)
    printf("%02x ", buf[i]);
  printf("\r\n");
#endif
  return false;
}
#endif

