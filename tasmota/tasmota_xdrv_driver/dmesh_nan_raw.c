/*
 * dmesh_nan_raw.c - pinned ESP8266 NONOS raw NAN transmit/filter adapter.
 *
 * SDK ABI: NONOSDK22x_190703 (the Arduino-ESP8266 framework selected by the
 * repository's Tasmota build).  This does not modify libnet80211.a.  It
 * calls the SDK's linked ieee80211_freedom_output() symbol through a tiny
 * Xtensa shim.  The function is exported by the SDK object even though it is
 * not declared in the public headers.
 */

#ifdef ESP8266

#include "user_interface.h"
#include "dmesh_nan_raw.h"

/* g_ic is the SDK's interface context.  These offsets are from the pinned
 * NONOSDK22x_190703 user_interface.o wrapper. */
extern uint8_t g_ic[];

extern int dmesh_nan_raw_trampoline(const void *freedom_output,
                                    void *conn,
                                    uint8_t *frame,
                                    int len,
                                    int sys_seq);

extern void ieee80211_freedom_output(void);

int ICACHE_FLASH_ATTR dmesh_nan_raw_tx(const uint8_t *frame, uint16_t len, uint8_t sys_seq) {
  if (!frame || len < 24 || len > 0x578) return -3;

  /* Match wifi_send_pkt_freedom()'s interface selection. */
  uint8_t opmode = wifi_get_opmode();
  void *conn = (void *)0;
  if (opmode == 1) {
    conn = *(void **)(g_ic + 16);
  } else if (opmode >= 2 && opmode < 4) {
    conn = *(void **)(g_ic + 20);
  }
  if (!conn) return -10;

  return dmesh_nan_raw_trampoline((const void *)&ieee80211_freedom_output, conn,
                                  (uint8_t *)frame, len, sys_seq);
}

void ICACHE_FLASH_ATTR dmesh_nan_raw_set_mac_filter(const uint8_t mac[6]) {
  if (mac) wifi_promiscuous_set_mac(mac);
}

uint8_t dmesh_nan_raw_is_mac_filter_available(void) {
  return 1;
}

#endif
