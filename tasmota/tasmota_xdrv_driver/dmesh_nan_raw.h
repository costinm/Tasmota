/*
 * Source-level ESP8266 NONOS raw 802.11 adapter.
 *
 * This is intentionally ESP8266/Tasmota-only.  ESP32 uses the Rust NAN
 * implementation and must not include this header.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * NAN action frames are rejected by wifi_send_pkt_freedom() in the
 * NONOSDK22x_190703 objects used by this Tasmota build.  This entry point
 * uses the same SDK allocator and ppTxPkt path, entering after that narrow
 * frame-control check.  The ABI is deliberately pinned; do not use this
 * backend with a different NONOS SDK archive.
 */
int dmesh_nan_raw_tx(const uint8_t *frame, uint16_t len, uint8_t sys_seq);

/* Optional SDK destination-MAC filter for promiscuous reception.  It is a
 * single destination filter, not a NAN-BSSID or attribute filter. */
void dmesh_nan_raw_set_mac_filter(const uint8_t mac[6]);

/* Software filtering remains necessary for NAN BSSID/OUI/action attributes. */
uint8_t dmesh_nan_raw_is_mac_filter_available(void);

#ifdef __cplusplus
}
#endif
