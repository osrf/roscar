#ifndef USB_H
#define USB_H

#include <stdint.h>
#include <stdbool.h>

void usb_init();
bool usb_tx(const uint8_t ep, const uint8_t *buf, const uint8_t buf_len);
//bool usb_txf_avail(const uint8_t ep, const uint8_t nbytes);
//void usb_rx(const uint8_t ep, const uint8_t *data, const uint8_t len);

#endif
