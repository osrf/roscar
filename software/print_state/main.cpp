#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <signal.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>
#include <errno.h>

#define VENDOR_ID  0xf055
#define PRODUCT_ID 0x0002

void perish_if(bool b, const char *msg)
{
  if (b)
  {
    printf("%s\n", msg);
    exit(1);
  }
}

static bool g_done = false;
void signal_handler(int signum)
{
  if (signum == SIGINT)
    g_done = true;
}

int usage()
{
  puts("usage: menc COMMAND [OPTIONS]\n");
  puts("  where COMMAND is one of { stream }\n");
  return 1;
}

void stream(libusb_device_handle *h)
{
  FILE *f = fopen("log.txt", "w");
  while (!g_done)
  {
    uint8_t rx_msg[64] = {0};
    int nrx = 0;
    int rx_rc = libusb_bulk_transfer(h, 0x81, 
                                     rx_msg, sizeof(rx_msg),
                                     &nrx, 10);
    if (rx_rc == LIBUSB_ERROR_TIMEOUT)
    {
      continue;
      //printf("timeout\n");
      //break;
    }
    else if (rx_rc != 0)
    {
      printf("rx err code: %d\n", rx_rc);
      printf("errno: %d = %s\n", errno, strerror(errno));
      break;
    }
    //printf("rx %d\n", nrx);
    uint32_t t = 0;
    static uint32_t t_prev = 0;
    memcpy(&t, &rx_msg[0], sizeof(uint32_t));
    uint32_t dt = t - t_prev;
    printf("t = %8d  dt = %8d\n", t, t - t_prev);
    t_prev = t;
  }
}

void blink(libusb_device_handle *h)
{
  FILE *f = fopen("log.txt", "w");
  uint8_t tx_msg[64] = {0};
  while (!g_done)
  {
    usleep(500000);
    if (!tx_msg[4])
      tx_msg[4] = 0x1;
    else
      tx_msg[4] = 0;

    uint8_t rx_msg[64] = {0};
    int nrx = 0;
    int tx_rc = libusb_bulk_transfer(h, 0x02, tx_msg, 8, &nrx, 10);
    if (tx_rc == LIBUSB_ERROR_TIMEOUT)
    {
      printf("timeout\n");
      continue;
    }
    else if (tx_rc != 0)
    {
      printf("tx err code: %d\n", tx_rc);
      printf("errno: %d = %s\n", errno, strerror(errno));
      break;
    }
  }
}

bool dmxl_set_reg(libusb_device_handle *h, uint8_t id, 
                  uint8_t reg_idx, uint8_t reg_val)
{
  uint8_t tx_msg[64] = {0};
  tx_msg[0] = 2; // set register
  tx_msg[4] = id;
  tx_msg[5] = reg_idx;
  tx_msg[6] = reg_val;
  uint8_t rx_msg[64] = {0};
  int nrx = 0;
  int tx_rc = libusb_bulk_transfer(h, 0x02, tx_msg, 8, &nrx, 10);
  if (tx_rc == LIBUSB_ERROR_TIMEOUT)
  {
    printf("timeout\n");
    return false;
  }
  else if (tx_rc != 0)
  {
    printf("tx err code: %d\n", tx_rc);
    printf("errno: %d = %s\n", errno, strerror(errno));
    return false;
  }
  return true;
}

void spin(libusb_device_handle *h)
{
  FILE *f = fopen("log.txt", "w");
  uint8_t tx_msg[64] = {0};
  tx_msg[0] = 1;
  dmxl_set_reg(h, 1, 0x18, 1); // enable torque
  dmxl_set_reg(h, 2, 0x18, 1); // enable torque
  int loop_count = 0;
  while (!g_done)
  {
    usleep(50000);
    if (++loop_count > 10)
    {
      printf("toggle wheel\n");
      if (tx_msg[4] != 0)
      {
        tx_msg[4] = 0x10;
        tx_msg[6] = 0x00;
        loop_count = 0;
      }
      else
      {
        tx_msg[4] = 0x00;
        tx_msg[6] = 0x10;
        loop_count = 0;
      }
    }

    uint8_t rx_msg[64] = {0};
    int nrx = 0;
    int tx_rc = libusb_bulk_transfer(h, 0x02, tx_msg, 8, &nrx, 10);
    if (tx_rc == LIBUSB_ERROR_TIMEOUT)
    {
      printf("timeout\n");
      continue;
    }
    else if (tx_rc != 0)
    {
      printf("tx err code: %d\n", tx_rc);
      printf("errno: %d = %s\n", errno, strerror(errno));
      break;
    }
  }
  dmxl_set_reg(h, 1, 0x18, 0); // disable torque
  dmxl_set_reg(h, 2, 0x18, 0); // disable torque
}

int main(int argc, char **argv)
{
  if (argc < 2)
    return usage();
  const char *cmd = argv[1];
  perish_if(libusb_init(NULL) < 0, "Couldn't init libusb");
  libusb_device_handle *h = libusb_open_device_with_vid_pid(NULL, 
                                                            VENDOR_ID, 
                                                            PRODUCT_ID);
  perish_if(h == NULL, "couldn't find or open device. check permissions?");
  perish_if(0 != libusb_claim_interface(h, 0), "couldn't claim interface");
  printf("device opened successfully.\n");
  signal(SIGINT, signal_handler);
  if (!strcmp(cmd, "stream"))
    stream(h);
  else if (!strcmp(cmd, "blink"))
    blink(h);
  else if (!strcmp(cmd, "spin"))
    spin(h);
  libusb_exit(NULL);
  return 0;
}

