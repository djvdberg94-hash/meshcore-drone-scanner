# MeshCore Drone Scanner

A drone-mounted MeshCore node that actively scans for repeaters in the area and reports results on the Public channel. Designed for wardriving and RF coverage mapping.

## Supported Hardware

- Heltec Wireless Stick V3 (ESP32-S3 + SX1262) - with OLED display support
- Seeed XIAO nRF52840 + WIO SX1262 Kit

## How it works

Every 60 seconds the node sends a CTL_TYPE_NODE_DISCOVER_REQ control packet.
Nearby repeaters respond immediately with their identity and signal info.
The drone collects responses for 10 seconds, then sends a single summary
message on the Public channel:

[Drone] Scan 1: 2 node(s) | RPT_C854D584 SNR:12 RSSI:-50 | RPT_E46D83D9 SNR:14 RSSI:-49

The OLED display (Heltec only) shows live scan status and results.

## Setup

1. Clone MeshCore from https://github.com/meshcore-dev/MeshCore
2. Copy DroneScanner.h, DroneScanner.cpp, main.cpp into examples/drone_scanner/
3. For Heltec WSL V3: append platformio_env.ini contents to variants/heltec_v3/platformio.ini
4. For XIAO nRF52840: append platformio_xiao_nrf52_env.ini contents to variants/xiao_nrf52/platformio.ini

### Build and Flash - Heltec WSL V3
pio run -e Heltec_WSL3_drone_scanner
pio run -e Heltec_WSL3_drone_scanner -t upload --upload-port /dev/ttyUSB0

### Build and Flash - XIAO nRF52840
pio run -e Xiao_nrf52_drone_scanner
pio run -e Xiao_nrf52_drone_scanner -t upload

## Configuration

Edit DroneScanner.h to change:
- SCAN_INTERVAL_MS: time between scans (default 60s)
- SCAN_COLLECT_MS: collection window per scan (default 10s)
- ADVERT_NAME: node name on the mesh (default DroneScanner)
- SCAN_CHANNEL_PSK: channel key (default MeshCore Public channel)

## Radio settings

Configured for 869.618 MHz, BW 62.5, SF8, CR8 (South African MeshCore community settings).
Adjust LORA_FREQ, LORA_BW, LORA_SF, LORA_CR in the platformio env for your region.

## OLED Display (Heltec only)

Boot:     DroneScanner / Starting...
Ready:    DroneScanner / Ready for / takeoff!
Scanning: Scanning! / Scan #1... / Listening...
Live:     Scanning... / Nodes: 2 / RPT_E46D 14dB
Done:     Scan done! / #1: 2 node(s) / RPT_E46D 14dB

## Author

ZR6DJV - Queenswood, Pretoria, South Africa
Amateur Radio Operator - MeshCore mesh at 869.618 MHz
