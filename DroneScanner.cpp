#include "DroneScanner.h"
#include <helpers/AdvertDataHelpers.h>
#include <helpers/TxtDataHelpers.h>

#define CTL_TYPE_NODE_DISCOVER_REQ  0x80
#define CTL_TYPE_NODE_DISCOVER_RESP 0x90

extern void set_display(const char* l1, const char* l2, const char* l3);

DroneScanner::DroneScanner(mesh::Radio& radio, mesh::RNG& rng,
                           mesh::RTCClock& rtc, mesh::MeshTables& tables)
  : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc,
                 *new StaticPoolPacketManager(48), tables)
{
}

void DroneScanner::begin(FILESYSTEM* fs) {
  IdentityStore store(*fs, "/drone_id");
  if (!store.load("_main", self_id)) {
    self_id = radio_new_identity();
    int count = 0;
    while (count++ < 10 &&
           (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {
      self_id = radio_new_identity();
    }
    store.save("_main", self_id);
  }

  Serial.print("DroneScanner ID: ");
  mesh::Utils::printHex(Serial, self_id.pub_key, PUB_KEY_SIZE);
  Serial.println();

  strncpy(_prefs.node_name, ADVERT_NAME, sizeof(_prefs.node_name) - 1);

  auto* ch = addChannel(SCAN_CHANNEL_NAME, SCAN_CHANNEL_PSK);
  if (ch) {
    _scan_channel = ch->channel;
    _channel_ready = true;
    Serial.println("Channel ready: " SCAN_CHANNEL_NAME);
  } else {
    Serial.println("ERROR: channel setup failed!");
  }

  Mesh::begin();
  _next_scan_at = millis() + 5000UL;
  set_display("DroneScanner", "Ready for", "takeoff!");
  Serial.println("Ready. First scan in 5s.");
}

void DroneScanner::_resetResults() {
  _scan_count = 0;
  memset(_results, 0, sizeof(_results));
}

void DroneScanner::_addResult(const char* name, const uint8_t* pubkey,
                               uint8_t adv_type, float snr, float rssi) {
  for (int i = 0; i < _scan_count; i++) {
    if (memcmp(_results[i].pubkey_prefix, pubkey, 4) == 0) {
      if (snr > _results[i].snr) {
        _results[i].snr  = snr;
        _results[i].rssi = rssi;
      }
      return;
    }
  }
  if (_scan_count >= MAX_SCAN_RESULTS) return;

  ScanResult& r = _results[_scan_count++];
  strncpy(r.name, name, sizeof(r.name) - 1);
  memcpy(r.pubkey_prefix, pubkey, 4);
  r.snr      = snr;
  r.rssi     = rssi;
  r.adv_type = adv_type;

  // Update display live
  char disp2[22], disp3[22];
  snprintf(disp2, sizeof(disp2), "Nodes: %u", _scan_count);
  snprintf(disp3, sizeof(disp3), "%.10s %.0fdB", name, snr);
  set_display("Scanning...", disp2, disp3);
}

void DroneScanner::_sendScanReport() {
  if (!_channel_ready) return;

  uint32_t ts = rtc_clock.getCurrentTime();
  char buf[120];
  char disp2[22], disp3[22];

  if (_scan_count == 0) {
    snprintf(buf, sizeof(buf), "[Drone] Scan #%u: no nodes heard.", _scan_index);
    sendGroupMessage(ts, _scan_channel, _prefs.node_name, buf, strlen(buf));
    Serial.println(buf);
    snprintf(disp2, sizeof(disp2), "Scan #%u done", _scan_index);
    set_display(disp2, "No nodes heard", "Land + check");
    return;
  }

  // Single summary message
  char summary[200] = {0};
  int pos = snprintf(summary, sizeof(summary),
                     "[Drone] Scan #%u: %u node(s) |", _scan_index, _scan_count);
  for (int i = 0; i < _scan_count && pos < (int)sizeof(summary) - 30; i++) {
    pos += snprintf(summary + pos, sizeof(summary) - pos,
                    " %s SNR:%.0f RSSI:%.0f",
                    _results[i].name, _results[i].snr, _results[i].rssi);
    if (i < _scan_count - 1)
      pos += snprintf(summary + pos, sizeof(summary) - pos, " |");
  }
  sendGroupMessage(ts, _scan_channel, _prefs.node_name, summary, strlen(summary));
  Serial.println(summary);

  // Update display
  int best = 0;
  for (int i = 1; i < _scan_count; i++) {
    if (_results[i].snr > _results[best].snr) best = i;
  }
  snprintf(disp2, sizeof(disp2), "#%u: %u node(s)", _scan_index, _scan_count);
  snprintf(disp3, sizeof(disp3), "%.8s %.0fdB", _results[best].name, _results[best].snr);
  set_display("Scan done!", disp2, disp3);
}

void DroneScanner::onControlDataRecv(mesh::Packet* packet) {
  if (!_collecting) return;
  if (packet->payload_len < 6) return;

  uint8_t type = packet->payload[0] & 0xF0;
  if (type != CTL_TYPE_NODE_DISCOVER_RESP) return;

  // Verify tag
  uint32_t resp_tag;
  memcpy(&resp_tag, &packet->payload[2], 4);
  if (_pending_tag != 0 && resp_tag != _pending_tag) return;

  uint8_t adv_type = packet->payload[0] & 0x0F;
  float snr  = packet->getSNR();
  float rssi = radio_driver.getLastRSSI();

  int pubkey_avail = (int)packet->payload_len - 6;
  if (pubkey_avail < 4) return;

  const uint8_t* pubkey = &packet->payload[6];
  char name[32] = {0};
  snprintf(name, sizeof(name), "RPT_%02X%02X%02X%02X",
           pubkey[0], pubkey[1], pubkey[2], pubkey[3]);

  Serial.printf("  RESP: %s SNR:%.1f RSSI:%.0f\n", name, snr, rssi);
  _addResult(name, pubkey, adv_type, snr, rssi);
}

void DroneScanner::onAdvertRecv(mesh::Packet* pkt, const mesh::Identity& id,
                                 uint32_t timestamp,
                                 const uint8_t* app_data, size_t app_data_len) {
  BaseChatMesh::onAdvertRecv(pkt, id, timestamp, app_data, app_data_len);

  if (!_collecting) return;

  AdvertDataParser parser(app_data, app_data_len);
  uint8_t adv_type = parser.getType();

  if (adv_type != ADV_TYPE_REPEATER &&
      adv_type != ADV_TYPE_CHAT &&
      adv_type != ADV_TYPE_ROOM) return;

  char name[32] = {0};
  const char* parsed_name = parser.getName();
  if (parsed_name && parsed_name[0]) {
    strncpy(name, parsed_name, sizeof(name) - 1);
  } else {
    snprintf(name, sizeof(name), "%02X%02X%02X%02X",
             id.pub_key[0], id.pub_key[1], id.pub_key[2], id.pub_key[3]);
  }

  float snr  = pkt->getSNR();
  float rssi = radio_driver.getLastRSSI();

  Serial.printf("  ADVERT: %s SNR:%.1f RSSI:%.0f\n", name, snr, rssi);
  _addResult(name, id.pub_key, adv_type, snr, rssi);
}

void DroneScanner::onDiscoveredContact(ContactInfo& contact, bool is_new,
                                       uint8_t path_len, const uint8_t* path) {}

void DroneScanner::loop() {
  BaseChatMesh::loop();

  unsigned long now = millis();

  if (_collecting && now >= _collect_until) {
    _collecting = false;
    Serial.printf("Scan #%u done: %u node(s).\n", _scan_index, _scan_count);
    _sendScanReport();
    _next_scan_at = now + SCAN_INTERVAL_MS;
  }

  if (!_collecting && _next_scan_at && now >= _next_scan_at) {
    _next_scan_at = 0;
    _scan_index++;
    _resetResults();
    _collecting    = true;
    _collect_until = now + SCAN_COLLECT_MS;

    char disp2[22];
    snprintf(disp2, sizeof(disp2), "Scan #%u...", _scan_index);
    set_display("Scanning!", disp2, "Listening...");

    Serial.printf("\n--- Scan #%u ---\n", _scan_index);

    uint8_t data[10];
    data[0] = CTL_TYPE_NODE_DISCOVER_REQ;
    data[1] = (1 << ADV_TYPE_REPEATER);
    getRNG()->random(&data[2], 4);
    memcpy(&_pending_tag, &data[2], 4);
    uint32_t since = 0;
    memcpy(&data[6], &since, 4);
    auto* pkt = createControlData(data, sizeof(data));
    if (pkt) {
      sendZeroHop(pkt);
      Serial.printf("Discover sent (tag=%08X)\n", _pending_tag);
    }
  }
}
