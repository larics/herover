// herover_esp32 configuration. ROBOT_ID (and the static IP, if used) is the
// only thing that differs between robots.
#pragma once

#include <Arduino.h>

// Unique per robot; must match `id` for this robot in config/robots.yaml.
// Datagrams carrying another id are dropped (catches swapped IPs).
#define ROBOT_ID 1

#define UDP_PORT 5005  // must match udp_port in config/robots.yaml

// ---------------- UART to the OpenRB-150 ----------------
// Only ESP32 TX -> OpenRB RX (plus GND) is wired; the robot never replies.
// Both pins are always set explicitly (on core 2.x, -1 falls back to UART1's
// default pins, which are the flash pins on a classic ESP32).
#define OPENRB_BAUD 115200

// Uncomment to override the per-chip defaults below.
// #define OPENRB_RX_PIN 16
// #define OPENRB_TX_PIN 17

#ifndef OPENRB_TX_PIN
  #if defined(CONFIG_IDF_TARGET_ESP32)        // NodeMCU-32S, DevKitC (WROOM)
    #define OPENRB_RX_PIN 16
    #define OPENRB_TX_PIN 17
  #elif defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
    #define OPENRB_RX_PIN 18
    #define OPENRB_TX_PIN 17
  #elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
    #define OPENRB_RX_PIN 4
    #define OPENRB_TX_PIN 5
  #else
    #error "Unknown ESP32 variant: set OPENRB_RX_PIN / OPENRB_TX_PIN in config.h"
  #endif
#endif

// ---------------- network ----------------
// 0 = DHCP (reserve the address on the router so robots.yaml stays valid).
// 1 = fixed address below. Either way the IP is printed on USB at connect.
#define USE_STATIC_IP 0
#define STATIC_IP      192, 168, 1, 101
#define STATIC_GATEWAY 192, 168, 1, 1
#define STATIC_SUBNET  255, 255, 255, 0
