/*
  xdrv_95_dmesh_nan.ino - DMesh raw NAN support for ESP8266

  This is deliberately an ESP8266 raw-frame adapter, not an ESP-NOW wrapper.
  The NAN cluster beacon is the only timing authority.  The ESP8266 first
  associates as a station with the lmesh/Recovery AP on channel 6; the AP's
  clock is never used for NAN scheduling.
*/

#ifdef ESP8266

#include "tasmota_xdrv_driver/dmesh_nan_raw.h"
#include <WiFiUdp.h>

#define XDRV_95 95

/* Wi-Fi Aware/NAN values shared with fw/esp32/rust/src/components/nan.rs. */
static const uint8_t DMESH_NAN_CHANNEL = 6;
static const uint8_t DMESH_NAN_BSSID[6] = { 0x50, 0x6f, 0x9a, 0x01, 0x05, 0x01 };
static const uint8_t DMESH_NAN_DISCOVERY_MAC[6] = { 0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00 };
static const uint8_t DMESH_NAN_SERVICE_ID[6] = { 0x75, 0x94, 0x31, 0x93, 0xea, 0xc9 };
static const uint8_t DMESH_NAN_HEADER[30] = {
  0xd0, 0x00, 0x00, 0x00,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x50, 0x6f, 0x9a, 0x01, 0x05, 0x01,
  0x00, 0x00, 0x04, 0x09, 0x50, 0x6f, 0x9a, 0x13
};

static const uint8_t DMESH_NAN_RX_QUEUE_LEN = 8;
static const uint8_t DMESH_NAN_TX_QUEUE_LEN = 8;
static const uint16_t DMESH_NAN_FRAME_MAX = 512;
static const uint16_t DMESH_NAN_PAYLOAD_MAX = 231;
static const uint32_t DMESH_NAN_CLUSTER_STALE_US = 3UL * 512UL * 1024UL;
static const uint32_t DMESH_NAN_DW_GUARD_US = 64000;
/* Existing DMesh/lmesh multicast transport convention. */
static const IPAddress DMESH_NAN_UDP_GROUP(224, 0, 0, 250);
static const uint16_t DMESH_NAN_UDP_RX_PORT = 15009;
static const uint16_t DMESH_NAN_UDP_TX_PORT = 15010;
static const uint8_t DMESH_NAN_UDP_RX_TAG = 'R';
static const uint8_t DMESH_NAN_UDP_TX_TAG = 'T';
static const uint8_t DMESH_NAN_UDP_VERSION = 1;
static const uint8_t DMESH_NAN_UDP_KIND_BEACON = 1;
static const uint8_t DMESH_NAN_UDP_KIND_ACTION = 2;
static const uint16_t DMESH_NAN_UDP_HEADER_LEN = 12;

struct DmeshNanRxSlot {
  uint16_t len;
  int8_t rssi;
  uint8_t frame[DMESH_NAN_FRAME_MAX];
};

struct DmeshNanTxItem {
  uint8_t kind;                         // 1 = publish, 2 = follow-up
  uint8_t destination[6];
  uint16_t len;
  uint8_t payload[DMESH_NAN_PAYLOAD_MAX];
};

struct DmeshNanState {
  bool enabled;
  bool promisc;
  bool cluster_locked;
  uint8_t cluster_bssid[6];
  uint32_t last_beacon_local_us;
  uint64_t last_beacon_tsf_us;
  uint32_t rx_beacons;
  uint32_t rx_actions;
  uint32_t rx_services;
  uint32_t rx_followups;
  uint32_t rx_drops;
  uint32_t tx_publishes;
  uint32_t tx_followups;
  uint32_t tx_drops;
  uint32_t tx_errors;
  uint8_t last_source[6];
  uint16_t last_payload_len;
  uint32_t rx_frames;
  uint32_t rx_short;
  uint8_t last_frame_type;
  uint16_t last_frame_len;
  int8_t last_tx_result;
  uint32_t udp_rx;
  uint32_t udp_tx;
  uint32_t udp_drops;
};

static DmeshNanRxSlot DmeshNanRx[DMESH_NAN_RX_QUEUE_LEN];
static volatile uint8_t DmeshNanRxHead = 0;
static volatile uint8_t DmeshNanRxTail = 0;
static DmeshNanTxItem DmeshNanTx[DMESH_NAN_TX_QUEUE_LEN];
static uint8_t DmeshNanTxHead = 0;
static uint8_t DmeshNanTxTail = 0;
static volatile bool DmeshNanTxReady = true;
static DmeshNanState DmeshNan = {};
static WiFiUDP DmeshNanUdpRx;
static WiFiUDP DmeshNanUdpTx;
static bool DmeshNanUdpReady = false;
static uint8_t DmeshNanUdpTxFrame[DMESH_NAN_FRAME_MAX];
static uint16_t DmeshNanUdpTxLen = 0;
static uint8_t DmeshNanUdpTxKind = 0;

static uint64_t DmeshNanReadLe64(const uint8_t *p) {
  uint64_t value = 0;
  for (uint8_t i = 0; i < 8; i++) {
    value |= ((uint64_t)p[i]) << (8 * i);
  }
  return value;
}

static bool DmeshNanMacEqual(const uint8_t *a, const uint8_t *b) {
  return 0 == memcmp(a, b, 6);
}

static void DmeshNanMacText(const uint8_t *mac, char *out, size_t size) {
  snprintf_P(out, size, PSTR("%02x:%02x:%02x:%02x:%02x:%02x"),
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void DmeshNanLocalMac(uint8_t *mac) {
  memset(mac, 0, 6);
  String mac_text = (WiFi.getMode() & WIFI_STA) ? WiFi.macAddress() : WiFi.softAPmacAddress();
  sscanf(mac_text.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
}

static bool DmeshNanHasFreshCluster(void) {
  return DmeshNan.cluster_locked &&
         (micros() - DmeshNan.last_beacon_local_us <= DMESH_NAN_CLUSTER_STALE_US);
}

static bool DmeshNanIsNanBeacon(const uint8_t *frame, uint16_t len);
static bool DmeshNanIsNanSdf(const uint8_t *frame, uint16_t len);

/* Non-OS exposes no management/action subtype filter. Keep the callback
   cheap by dropping control/data frames before they enter the queue; NAN
   still needs both beacons and public-action frames. */
static bool DmeshNanIsMgmtCandidate(const uint8_t *frame, uint16_t len) {
  if (!frame || len < 2 || (frame[0] & 0x0c) != 0) return false;
  uint8_t subtype = frame[0] & 0xf0;
  return subtype == 0x80 || subtype == 0xd0;
}

static bool DmeshNanUdpStart(void) {
  if (DmeshNanUdpReady) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  bool rx_ok = DmeshNanUdpRx.beginMulticast(WiFi.localIP(), DMESH_NAN_UDP_GROUP,
                                            DMESH_NAN_UDP_RX_PORT);
  bool tx_ok = rx_ok && DmeshNanUdpTx.begin(DMESH_NAN_UDP_TX_PORT);
  DmeshNanUdpReady = rx_ok && tx_ok;
  if (!DmeshNanUdpReady) {
    DmeshNanUdpRx.stop();
    DmeshNanUdpTx.stop();
  }
  return DmeshNanUdpReady;
}

static void DmeshNanUdpStop(void) {
  if (DmeshNanUdpReady) {
    DmeshNanUdpRx.stop();
    DmeshNanUdpTx.stop();
    DmeshNanUdpReady = false;
  }
}

static void DmeshNanUdpForward(uint8_t kind, const uint8_t *frame, uint16_t len, int8_t rssi) {
  if (!frame || !len || len > DMESH_NAN_FRAME_MAX || !DmeshNanUdpStart()) {
    DmeshNan.udp_drops++;
    return;
  }
  uint8_t origin[6];
  DmeshNanLocalMac(origin);
  uint8_t header[DMESH_NAN_UDP_HEADER_LEN] = {
    DMESH_NAN_UDP_RX_TAG, DMESH_NAN_UDP_VERSION, kind, (uint8_t)rssi,
    origin[0], origin[1], origin[2], origin[3], origin[4], origin[5],
    (uint8_t)(len >> 8), (uint8_t)len
  };
  if (!DmeshNanUdpRx.beginPacket(DMESH_NAN_UDP_GROUP, DMESH_NAN_UDP_RX_PORT) ||
      DmeshNanUdpRx.write(header, sizeof(header)) != sizeof(header) ||
      DmeshNanUdpRx.write(frame, len) != len || !DmeshNanUdpRx.endPacket()) {
    DmeshNan.udp_drops++;
    return;
  }
  DmeshNan.udp_tx++;
}

static void DmeshNanUdpPoll(void) {
  if (!DmeshNanUdpStart()) return;
  uint16_t packet_len = DmeshNanUdpTx.parsePacket();
  if (!packet_len) return;
  uint8_t packet[DMESH_NAN_UDP_HEADER_LEN + DMESH_NAN_FRAME_MAX];
  if (packet_len > sizeof(packet)) {
    while (DmeshNanUdpTx.available()) DmeshNanUdpTx.read();
    DmeshNan.udp_drops++;
    return;
  }
  int received = DmeshNanUdpTx.read(packet, packet_len);
  if (received < DMESH_NAN_UDP_HEADER_LEN ||
      packet[0] != DMESH_NAN_UDP_TX_TAG || packet[1] != DMESH_NAN_UDP_VERSION) {
    DmeshNan.udp_drops++;
    return;
  }
  uint8_t local[6];
  DmeshNanLocalMac(local);
  if (!memcmp(packet + 4, local, 6)) return;
  uint16_t frame_len = ((uint16_t)packet[10] << 8) | packet[11];
  if (frame_len == 0 || frame_len > DMESH_NAN_FRAME_MAX ||
      frame_len + DMESH_NAN_UDP_HEADER_LEN != (uint16_t)received) {
    DmeshNan.udp_drops++;
    return;
  }
  uint8_t kind = packet[6];
  const uint8_t *frame = packet + DMESH_NAN_UDP_HEADER_LEN;
  bool valid = kind == DMESH_NAN_UDP_KIND_BEACON ? DmeshNanIsNanBeacon(frame, frame_len) :
               kind == DMESH_NAN_UDP_KIND_ACTION && DmeshNanIsNanSdf(frame, frame_len);
  if (!valid || DmeshNanUdpTxLen) {
    DmeshNan.udp_drops++;
    return;
  }
  memcpy(DmeshNanUdpTxFrame, frame, frame_len);
  DmeshNanUdpTxLen = frame_len;
  DmeshNanUdpTxKind = kind;
  DmeshNan.udp_rx++;
}

static bool DmeshNanIsNanBeacon(const uint8_t *frame, uint16_t len) {
  // The final two BSSID octets identify the current NAN cluster and are
  // dynamic.  Match the NAN OUI/type prefix, then learn the full BSSID.
  return len >= 36 && frame[0] == 0x80 &&
         frame[16] == 0x50 && frame[17] == 0x6f &&
         frame[18] == 0x9a && frame[19] == 0x01;
}

static bool DmeshNanIsNanSdf(const uint8_t *frame, uint16_t len) {
  return len > 30 && frame[0] == 0xd0 &&
         DmeshNanMacEqual(frame + 16, DmeshNan.cluster_bssid) &&
         frame[24] == 0x04 && frame[25] == 0x09 &&
         frame[26] == 0x50 && frame[27] == 0x6f &&
         frame[28] == 0x9a && frame[29] == 0x13;
}

static bool DmeshNanQueueRx(const uint8_t *frame, uint16_t len, int8_t rssi) {
  uint8_t next = (DmeshNanRxHead + 1) % DMESH_NAN_RX_QUEUE_LEN;
  if (next == DmeshNanRxTail) {
    DmeshNan.rx_drops++;
    return false;
  }
  if (len > DMESH_NAN_FRAME_MAX) {
    len = DMESH_NAN_FRAME_MAX;
  }
  DmeshNanRx[DmeshNanRxHead].len = len;
  DmeshNanRx[DmeshNanRxHead].rssi = rssi;
  memcpy(DmeshNanRx[DmeshNanRxHead].frame, frame, len);
  DmeshNanRxHead = next;
  return true;
}

/* The callback is intentionally copy-only: no parsing, logging, allocation,
   or Tasmota command dispatch is safe in the SDK Wi-Fi callback. */
static void ICACHE_FLASH_ATTR DmeshNanPromiscCallback(uint8_t *buf, uint16_t len) {
  DmeshNan.rx_frames++;
  if (!DmeshNan.enabled || !buf || len <= 12) {
    DmeshNan.rx_short++;
    return;
  }
  struct DmeshNanRxControl {
    int8_t rssi;
    uint8_t reserved[11];
  } __attribute__((packed));
  DmeshNanRxControl *control = reinterpret_cast<DmeshNanRxControl *>(buf);
  const uint8_t *frame = buf + sizeof(DmeshNanRxControl);
  uint16_t frame_len = len - sizeof(DmeshNanRxControl);
  if (!DmeshNanIsMgmtCandidate(frame, frame_len)) {
    return;
  }
  DmeshNan.last_frame_type = frame[0];
  DmeshNan.last_frame_len = frame_len;
  DmeshNanQueueRx(frame, frame_len, control->rssi);
}

static void DmeshNanStopPromisc(void) {
  if (DmeshNan.promisc) {
    wifi_promiscuous_enable(0);
    wifi_set_promiscuous_rx_cb(nullptr);
    DmeshNan.promisc = false;
  }
}

static void ICACHE_FLASH_ATTR DmeshNanFreedomSent(uint8 status) {
  (void)status;
  DmeshNanTxReady = true;
}

static void DmeshNanStartPromisc(void) {
  if (!DmeshNan.enabled || DmeshNan.promisc) {
    return;
  }
  // ESP8266 NONOS sniffer mode is station-only.  This is called only after
  // normal STA association, so switching away from any transient AP mode
  // cannot interfere with the lmesh/Recovery AP join.
  if (WiFi.getMode() != WIFI_STA) {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
  }
  if (wifi_get_channel() != DMESH_NAN_CHANNEL) {
    wifi_set_channel(DMESH_NAN_CHANNEL);
  }
  wifi_register_send_pkt_freedom_cb(DmeshNanFreedomSent);
  wifi_set_promiscuous_rx_cb(DmeshNanPromiscCallback);
  wifi_promiscuous_enable(1);
  DmeshNan.promisc = true;
}

static bool DmeshNanQueueTx(uint8_t kind, const uint8_t *destination, const uint8_t *payload, uint16_t len) {
  if (len > DMESH_NAN_PAYLOAD_MAX) {
    return false;
  }
  uint8_t next = (DmeshNanTxHead + 1) % DMESH_NAN_TX_QUEUE_LEN;
  if (next == DmeshNanTxTail) {
    DmeshNan.tx_drops++;
    return false;
  }
  DmeshNanTx[DmeshNanTxHead].kind = kind;
  memcpy(DmeshNanTx[DmeshNanTxHead].destination, destination, 6);
  DmeshNanTx[DmeshNanTxHead].len = len;
  if (len) {
    memcpy(DmeshNanTx[DmeshNanTxHead].payload, payload, len);
  }
  DmeshNanTxHead = next;
  return true;
}

static bool DmeshNanBuildFollowup(const uint8_t *destination, uint16_t item_len,
                                  const uint8_t *payload, uint8_t *frame, uint16_t *frame_len) {
  if (item_len > 255 || !DmeshNanHasFreshCluster()) {
    return false;
  }
  memcpy(frame, DMESH_NAN_HEADER, sizeof(DMESH_NAN_HEADER));
  memcpy(frame + 4, destination, 6);
  static const uint8_t broadcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
  if (DmeshNanMacEqual(destination, broadcast)) {
    memcpy(frame + 4, DMESH_NAN_DISCOVERY_MAC, 6);
  }
  uint8_t mac[6];
  DmeshNanLocalMac(mac);
  memcpy(frame + 10, mac, 6);
  memcpy(frame + 16, DmeshNan.cluster_bssid, 6);
  uint16_t body_len = item_len + 10;
  uint16_t pos = 30;
  frame[pos++] = 0x03;
  frame[pos++] = body_len & 0xff;
  frame[pos++] = body_len >> 8;
  memcpy(frame + pos, DMESH_NAN_SERVICE_ID, 6); pos += 6;
  frame[pos++] = 1;                 // instance
  frame[pos++] = 0;                 // requestor instance
  frame[pos++] = 0x12;              // follow-up with service info
  frame[pos++] = item_len;
  memcpy(frame + pos, payload, item_len); pos += item_len;
  *frame_len = pos;
  return true;
}

static bool DmeshNanBuildPublish(uint8_t *frame, uint16_t *frame_len) {
  if (!DmeshNanHasFreshCluster()) {
    return false;
  }
  memcpy(frame, DMESH_NAN_HEADER, sizeof(DMESH_NAN_HEADER));
  memcpy(frame + 4, DMESH_NAN_DISCOVERY_MAC, 6);
  uint8_t mac[6];
  DmeshNanLocalMac(mac);
  memcpy(frame + 10, mac, 6);
  memcpy(frame + 16, DmeshNan.cluster_bssid, 6);
  uint16_t pos = 30;
  const uint8_t service_info[21] = {
    'D', 'M', 1, 1, 0, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };
  // Service Descriptor Attribute: service id, instances, publish control,
  // and the shared 21-byte DMesh service-specific information.
  frame[pos++] = 0x03;
  frame[pos++] = 31;
  frame[pos++] = 0;
  memcpy(frame + pos, DMESH_NAN_SERVICE_ID, 6); pos += 6;
  frame[pos++] = 1;
  frame[pos++] = 0;
  frame[pos++] = 0x10;
  frame[pos++] = sizeof(service_info);
  memcpy(frame + pos, service_info, sizeof(service_info)); pos += sizeof(service_info);
  // SDEA service-update indicator, matching the ESP32 publisher.
  frame[pos++] = 0x0e; frame[pos++] = 4; frame[pos++] = 0;
  frame[pos++] = 1; frame[pos++] = 0; frame[pos++] = 2; frame[pos++] = 2;
  // Minimal 2.4-GHz Device Capability and Availability attributes.
  const uint8_t capability[] = { 0x0f, 9, 0, 0, 1, 0, 4, 0, 0x11, 0, 0, 0 };
  memcpy(frame + pos, capability, sizeof(capability)); pos += sizeof(capability);
  const uint8_t availability[] = {
    0x12, 13, 0, 1, 0, 0, 8, 0, 1, 0x11, 0x18, 1, 0x0f, 0x10, 2
  };
  memcpy(frame + pos, availability, sizeof(availability)); pos += sizeof(availability);
  *frame_len = pos;
  return true;
}

static bool DmeshNanTransmit(const uint8_t *frame, uint16_t len) {
  bool beacon = len && frame[0] == 0x80;
  if (len > DMESH_NAN_FRAME_MAX || (!beacon && !DmeshNanHasFreshCluster()) || !DmeshNanTxReady) {
    return false;
  }
  // The private source-level adapter uses the same SDK allocator/transmit
  // path as wifi_send_pkt_freedom(), but bypasses only its category check.
  // Capture is paused for the bounded send because NONOS sniffer mode and TX
  // share the radio scheduler.
  bool was_promisc = DmeshNan.promisc;
  if (was_promisc) {
    wifi_set_promiscuous_rx_cb(nullptr);
    wifi_promiscuous_enable(0);
    delay(1);
    wifi_set_channel(DMESH_NAN_CHANNEL);
  }
  int result = dmesh_nan_raw_tx(frame, len, true);
  DmeshNan.last_tx_result = result;
  if (was_promisc) {
    wifi_set_promiscuous_rx_cb(DmeshNanPromiscCallback);
    wifi_promiscuous_enable(1);
  }
  if (result) {
    DmeshNan.tx_errors++;
    return false;
  }
  DmeshNanTxReady = false;
  return true;
}

static void DmeshNanDrainTx(void) {
  bool udp_pending = DmeshNanUdpTxLen != 0;
  if ((!DmeshNanHasFreshCluster() && !(udp_pending && DmeshNanUdpTxKind == DMESH_NAN_UDP_KIND_BEACON)) ||
      (!udp_pending && DmeshNanTxTail == DmeshNanTxHead)) {
    return;
  }
  // The observed NAN beacon is the phase anchor. Do not use the AP's TSF.
  uint32_t elapsed = micros() - DmeshNan.last_beacon_local_us;
  if (!udp_pending && elapsed > DMESH_NAN_DW_GUARD_US) {
    return;
  }
  uint8_t frame[DMESH_NAN_FRAME_MAX];
  uint16_t len = 0;
  DmeshNanTxItem *item = udp_pending ? nullptr : &DmeshNanTx[DmeshNanTxTail];
  bool built = udp_pending ? (memcpy(frame, DmeshNanUdpTxFrame, DmeshNanUdpTxLen),
                              len = DmeshNanUdpTxLen, true) :
               item->kind == 1 ? DmeshNanBuildPublish(frame, &len) :
               DmeshNanBuildFollowup(item->destination, item->len, item->payload, frame, &len);
  if (!built) {
    return;
  }
  if (DmeshNanTransmit(frame, len)) {
    if (udp_pending) {
      DmeshNanUdpTxLen = 0;
      DmeshNanUdpTxKind = 0;
    } else {
      if (item->kind == 1) DmeshNan.tx_publishes++; else DmeshNan.tx_followups++;
      DmeshNanTxTail = (DmeshNanTxTail + 1) % DMESH_NAN_TX_QUEUE_LEN;
    }
  }
}

static void DmeshNanParseService(const uint8_t *frame, uint16_t len, int8_t rssi) {
  if (!DmeshNanIsNanSdf(frame, len)) {
    return;
  }
  DmeshNan.rx_actions++;
  uint16_t pos = 30;
  while (pos + 3 <= len) {
    uint8_t attr = frame[pos++];
    uint16_t attr_len = frame[pos++] | ((uint16_t)frame[pos++] << 8);
    if (pos + attr_len > len) return;
    if (attr == 0x03 && attr_len >= 10 && !memcmp(frame + pos, DMESH_NAN_SERVICE_ID, 6)) {
      uint8_t control = frame[pos + 8];
      uint8_t info_len = frame[pos + 9];
      if (10U + info_len <= attr_len) {
        memcpy(DmeshNan.last_source, frame + 10, 6);
        DmeshNan.last_payload_len = info_len;
        if (control == 0x12) DmeshNan.rx_followups++; else DmeshNan.rx_services++;
        char source[18];
        DmeshNanMacText(frame + 10, source, sizeof(source));
        AddLog(LOG_LEVEL_DEBUG, PSTR("DMN: RX %s source=%s len=%u rssi=%d"),
               control == 0x12 ? "followup" : "service", source, info_len, rssi);
      }
    }
    pos += attr_len;
  }
}

static void DmeshNanPollRx(void) {
  while (DmeshNanRxTail != DmeshNanRxHead) {
    DmeshNanRxSlot &slot = DmeshNanRx[DmeshNanRxTail];
    if (DmeshNanIsNanBeacon(slot.frame, slot.len)) {
      uint32_t now = micros();
      if (!DmeshNan.cluster_locked || DmeshNanMacEqual(DmeshNan.cluster_bssid, slot.frame + 16) ||
          now - DmeshNan.last_beacon_local_us > DMESH_NAN_CLUSTER_STALE_US) {
        memcpy(DmeshNan.cluster_bssid, slot.frame + 16, 6);
        DmeshNan.cluster_locked = true;
        DmeshNan.last_beacon_local_us = now;
        DmeshNan.last_beacon_tsf_us = DmeshNanReadLe64(slot.frame + 24);
        DmeshNan.rx_beacons++;
      }
      DmeshNanUdpForward(DMESH_NAN_UDP_KIND_BEACON, slot.frame, slot.len, slot.rssi);
    } else {
      DmeshNanParseService(slot.frame, slot.len, slot.rssi);
      if (DmeshNanIsNanSdf(slot.frame, slot.len)) {
        DmeshNanUdpForward(DMESH_NAN_UDP_KIND_ACTION, slot.frame, slot.len, slot.rssi);
      }
    }
    DmeshNanRxTail = (DmeshNanRxTail + 1) % DMESH_NAN_RX_QUEUE_LEN;
  }
}

static int DmeshNanHex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static uint16_t DmeshNanParseHex(const char *text, uint8_t *out, uint16_t max_len) {
  uint16_t len = 0;
  while (*text && len < max_len) {
    while (*text == ' ' || *text == ',' || *text == ':') text++;
    if (!text[0] || !text[1]) break;
    int hi = DmeshNanHex(text[0]);
    int lo = DmeshNanHex(text[1]);
    if (hi < 0 || lo < 0) break;
    out[len++] = (hi << 4) | lo;
    text += 2;
  }
  return len;
}

static void DmeshNanStatus(void) {
  char bssid[18] = "00:00:00:00:00:00";
  if (DmeshNan.cluster_locked) DmeshNanMacText(DmeshNan.cluster_bssid, bssid, sizeof(bssid));
  Response_P(PSTR("{\"DmeshNan\":{\"enabled\":%u,\"promisc\":%u,\"channel\":%u,\"synced\":%u,\"bssid\":\"%s\",\"beacons\":%u,\"services\":%u,\"followups\":%u,\"publish_tx\":%u,\"followup_tx\":%u,\"rx_drops\":%u,\"tx_drops\":%u,\"tx_errors\":%u,\"last_tx_result\":%d,\"rx_frames\":%u,\"rx_short\":%u,\"last_type\":%u,\"last_len\":%u,\"udp_rx\":%u,\"udp_tx\":%u,\"udp_drops\":%u}}"),
             DmeshNan.enabled, DmeshNan.promisc, WiFi.channel(), DmeshNanHasFreshCluster(), bssid,
             DmeshNan.rx_beacons, DmeshNan.rx_services, DmeshNan.rx_followups,
             DmeshNan.tx_publishes, DmeshNan.tx_followups, DmeshNan.rx_drops,
             DmeshNan.tx_drops, DmeshNan.tx_errors, DmeshNan.last_tx_result,
             DmeshNan.rx_frames, DmeshNan.rx_short,
             DmeshNan.last_frame_type, DmeshNan.last_frame_len,
             DmeshNan.udp_rx, DmeshNan.udp_tx, DmeshNan.udp_drops);
}

static void DmeshNanCommandHandler(void) {
  if (XdrvMailbox.command_code == 1) {
    DmeshNanStatus();
    return;
  }
  if (XdrvMailbox.data_len == 0 || !strcasecmp(XdrvMailbox.data, "status")) {
    DmeshNanStatus();
  } else if (!strcasecmp(XdrvMailbox.data, "0")) {
    DmeshNan.enabled = false;
    DmeshNanStopPromisc();
    ResponseCmndNumber(0);
  } else if (!strcasecmp(XdrvMailbox.data, "1")) {
    DmeshNan.enabled = true;
    if (WiFi.status() == WL_CONNECTED) {
      DmeshNanStartPromisc();
    }
    ResponseCmndNumber(1);
  } else if (!strncasecmp(XdrvMailbox.data, "publish", 7)) {
    static const uint8_t broadcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    ResponseCmndNumber(DmeshNanQueueTx(1, broadcast, nullptr, 0));
  } else if (!strncasecmp(XdrvMailbox.data, "send ", 5)) {
    uint8_t payload[DMESH_NAN_PAYLOAD_MAX];
    uint16_t len = DmeshNanParseHex(XdrvMailbox.data + 5, payload, sizeof(payload));
    static const uint8_t broadcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    ResponseCmndNumber(len && DmeshNanQueueTx(2, broadcast, payload, len));
  } else {
    Response_P(PSTR("{\"DmeshNan\":\"usage: DmeshNan 0|1|status|publish|send <hex>\"}"));
  }
}

const char kDmeshNanCommands[] PROGMEM = "DmeshNan|DmeshNanStatus";
void (* const DmeshNanCommand[])(void) PROGMEM = {
  &DmeshNanCommandHandler, &DmeshNanStatus };

bool Xdrv95(uint32_t function) {
  bool result = false;
  switch (function) {
    case FUNC_INIT:
      DmeshNan.enabled = true;
      break;
    case FUNC_NETWORK_UP:
      DmeshNanUdpStart();
      DmeshNanStartPromisc();
      break;
    case FUNC_EVERY_SECOND:
      if (WiFi.status() == WL_CONNECTED) {
        DmeshNanUdpStart();
        DmeshNanStartPromisc();
      } else {
        // Let the normal STA state machine associate first.  Starting the
        // Non-OS sniffer while association is pending prevents the ESP8266
        // from receiving the Recovery/lmesh AP beacon and causes an AP
        // timeout.  `DmeshNan 1` only arms capture; association still gates
        // the actual sniffer start.
        DmeshNanStopPromisc();
      }
      break;
    case FUNC_LOOP:
      DmeshNanUdpPoll();
      DmeshNanPollRx();
      DmeshNanDrainTx();
      break;
    case FUNC_COMMAND:
      if (!strncasecmp(XdrvMailbox.topic, "DMESHNAN", 8)) {
        strlcpy(XdrvMailbox.command, XdrvMailbox.topic, CMDSZ);
        XdrvMailbox.command_code = !strcasecmp(XdrvMailbox.topic, "DMESHNANSTATUS");
        DmeshNanCommandHandler();
        result = true;
      } else {
        result = DecodeCommand(kDmeshNanCommands, DmeshNanCommand);
      }
      break;
    case FUNC_NETWORK_DOWN:
      DmeshNanUdpStop();
      DmeshNanStopPromisc();
      break;
    case FUNC_ACTIVE:
      result = true;
      break;
  }
  return result;
}

#endif  // ESP8266
