# MeshCore Drone Scanner

A drone-mounted MeshCore node that actively scans for repeaters in the area and reports results on the Public channel.

## Hardware
- Heltec Wireless Stick V3 (ESP32-S3 + SX1262)

## How it works
Every 60 seconds the node sends a CTL_TYPE_NODE_DISCOVER_REQ control packet.
Nearby repeaters respond immediately. The drone collects responses for 10 seconds,
then sends a single summary message on the Public channel.

Example output:
[Drone] Scan 1: 2 node(s) | RPT_C854D584 SNR:12 RSSI:-50 | RPT_E46D83D9 SNR:14 RSSI:-49

## Setup
1. Clone MeshCore from https://github.com/meshcore-dev/MeshCore
2. Copy DroneScanner.h, DroneScanner.cpp, main.cpp into examples/drone_scanner/
3. Append the env from platformio_env.ini to variants/heltec_v3/platformio.ini
4. Build: pio run -e Heltec_WSL3_drone_scanner
5. Flash: pio run -e Heltec_WSL3_drone_scanner -t upload --upload-port /dev/ttyUSB0

## Configuration
Edit DroneScanner.h to change:
- SCAN_INTERVAL_MS: time between scans (default 60s)
- SCAN_COLLECT_MS: collection window per scan (default 10s)
- ADVERT_NAME: node name on the mesh
- SCAN_CHANNEL_PSK: channel key (default MeshCore Public channel)

## Radio settings
Configured for 869.618 MHz, BW 62.5, SF8, CR8 - adjust in platformio.ini for your region.

## Author
ZR6DJV - Queenswood, Pretoria, South Africa
