#pragma once

#include <Arduino.h>
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#include <RTClib.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/IdentityStore.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/BaseChatMesh.h>
#include <helpers/AdvertDataHelpers.h>
#include <helpers/TxtDataHelpers.h>
#include <target.h>

#ifndef FIRMWARE_BUILD_DATE
  #define FIRMWARE_BUILD_DATE  "1 Jul 2026"
#endif
#ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION     "v1.0.0"
#endif
#ifndef ADVERT_NAME
  #define ADVERT_NAME          "DroneScanner"
#endif

#ifndef SCAN_CHANNEL_PSK
  #define SCAN_CHANNEL_PSK     "izOH6cXN6mrJ5e26oRXNcg=="
#endif
#ifndef SCAN_CHANNEL_NAME
  #define SCAN_CHANNEL_NAME    "Public"
#endif

#ifndef SCAN_INTERVAL_MS
  #define SCAN_INTERVAL_MS     60000UL
#endif
#ifndef SCAN_COLLECT_MS
  #define SCAN_COLLECT_MS      10000UL
#endif

#define MAX_SCAN_RESULTS       16

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS         50
#endif
#ifndef MAX_GROUP_CHANNELS
  #define MAX_GROUP_CHANNELS   4
#endif

struct NodePrefs {
  char node_name[32];
};

struct ScanResult {
  char    name[32];
  uint8_t pubkey_prefix[4];
  float   snr;
  float   rssi;
  uint8_t adv_type;
};

class DroneScanner : public BaseChatMesh {
  NodePrefs          _prefs;
  mesh::GroupChannel _scan_channel;
  bool               _channel_ready = false;

  unsigned long      _next_scan_at  = 0;
  unsigned long      _collect_until = 0;
  bool               _collecting    = false;
  uint8_t            _scan_count    = 0;
  uint8_t            _scan_index    = 0;
  uint32_t           _pending_tag   = 0;
  ScanResult         _results[MAX_SCAN_RESULTS];

  void _resetResults();
  void _addResult(const char* name, const uint8_t* pubkey,
                  uint8_t adv_type, float snr, float rssi);
  void _sendScanReport();

protected:
  mesh::DispatcherAction onRecvPacket(mesh::Packet* pkt) override {
    if (pkt->isRouteDirect() && pkt->getPayloadType() == PAYLOAD_TYPE_CONTROL
        && pkt->payload_len > 0 && (pkt->payload[0] & 0x80) != 0
        && pkt->getPathHashCount() == 0) {
      onControlDataRecv(pkt);
      return ACTION_RELEASE;
    }
    return mesh::Mesh::onRecvPacket(pkt);
  }

  void onControlDataRecv(mesh::Packet* packet) override;

  void onAdvertRecv(mesh::Packet* pkt, const mesh::Identity& id,
                    uint32_t timestamp,
                    const uint8_t* app_data, size_t app_data_len) override;

  void onDiscoveredContact(ContactInfo& contact, bool is_new,
                           uint8_t path_len, const uint8_t* path) override;
  ContactInfo* processAck(const uint8_t* data) override { return nullptr; }
  void onContactPathUpdated(const ContactInfo& contact) override {}
  void onMessageRecv(const ContactInfo& contact, mesh::Packet* pkt,
                     uint32_t sender_timestamp, const char* text) override {}
  void onCommandDataRecv(const ContactInfo& contact, mesh::Packet* pkt,
                         uint32_t sender_timestamp, const char* text) override {}
  void onSignedMessageRecv(const ContactInfo& contact, mesh::Packet* pkt,
                           uint32_t sender_timestamp,
                           const uint8_t* sender_prefix,
                           const char* text) override {}
  void onChannelMessageRecv(const mesh::GroupChannel& channel,
                            mesh::Packet* pkt, uint32_t timestamp,
                            const char* text) override {}
  void onChannelDataRecv(const mesh::GroupChannel& channel,
                         mesh::Packet* pkt, uint16_t data_type,
                         const uint8_t* data, size_t len) override {}
  uint8_t onContactRequest(const ContactInfo& contact,
                           uint32_t sender_timestamp,
                           const uint8_t* data, uint8_t len,
                           uint8_t* reply) override { return 0; }
  void onContactResponse(const ContactInfo& contact,
                         const uint8_t* data, uint8_t len) override {}

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return pkt_airtime_millis * 3;
  }
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    return pkt_airtime_millis * (path_len + 1) * 2;
  }
  void onSendTimeout() override {}

  void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) override {
    Serial.printf("  RAW rx: len=%d snr=%.1f rssi=%.0f bytes:", len, snr, rssi);
    for (int i = 0; i < min(len, 8); i++) Serial.printf("%02X ", raw[i]);
    Serial.println();
  }

  bool filterRecvFloodPacket(mesh::Packet* pkt) override { return false; }

public:
  DroneScanner(mesh::Radio& radio, mesh::RNG& rng,
               mesh::RTCClock& rtc, mesh::MeshTables& tables);

  void begin(FILESYSTEM* fs);
  void loop();

  NodePrefs* getNodePrefs() { return &_prefs; }
};
