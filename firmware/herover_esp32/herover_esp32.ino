// herover_esp32 — WiFi command link for the herover.
//
// Receives one UDP datagram per velocity command from the ROS 2
// cmd_vel_bridge and forwards it to the OpenRB-150 over UART:
//
//   UDP  in : CMD <id> <vx> <vy> <wz>\n   (m/s, m/s, rad/s)
//   UART out: CMD <vx> <vy> <wz>\n
//
// Every valid datagram addressed to ROBOT_ID is forwarded, including one
// identical to the previous. Nothing is ever transmitted back over WiFi;
// diagnostics go to USB Serial only. The onboard LED flashes for each
// forwarded command. Works on any ESP32 variant — only the UART pins differ
// (see config.h).

#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #error "Copy secrets.example.h to secrets.h and fill in your WiFi credentials"
#endif

HardwareSerial OpenRB(1);
WiFiUDP udp;

char hostname[24];
bool netStarted = false;   // UDP socket + mDNS, set up on first connect
bool wasConnected = false;
uint32_t lastReconnectMs = 0;

// Diagnostics (USB only).
uint32_t nRx = 0, nFwd = 0, nBad = 0, nWrongId = 0;
uint32_t lastStatsMs = 0, lastStatsRx = 0;

bool ledLit = false;
uint32_t ledOnMs = 0;

// ---------------- status LED ----------------

void flashLed() {
#if USE_STATUS_LED
  digitalWrite(STATUS_LED_PIN, HIGH);
  ledLit = true;
  ledOnMs = millis();
#endif
}

void serviceLed() {
#if USE_STATUS_LED
  if (ledLit && millis() - ledOnMs >= LED_PULSE_MS) {
    digitalWrite(STATUS_LED_PIN, LOW);
    ledLit = false;
  }
#endif
}

// ---------------- parsing ----------------

// Whole token is a finite number (same acceptance as the OpenRB parser).
bool isNumberToken(const char* tok) {
  if (!tok) return false;
  char* end;
  const float v = strtof(tok, &end);
  if (end == tok || *end != '\0') return false;
  return fabsf(v) < 1.0e3f;  // false for NaN too
}

bool parseId(const char* tok, long& id) {
  if (!tok) return false;
  char* end;
  id = strtol(tok, &end, 10);
  return end != tok && *end == '\0';
}

// Validate "CMD <id> <vx> <vy> <wz>" and forward it without the id. The
// number tokens are passed through verbatim, so no precision is lost.
void handleDatagram(char* msg) {
  const char* SEP = " \t\r\n";
  char* cmd = strtok(msg, SEP);
  char* idTok = strtok(NULL, SEP);
  char* num[3];
  for (int i = 0; i < 3; i++) num[i] = strtok(NULL, SEP);

  long id;
  if (!cmd || strcmp(cmd, "CMD") != 0 || !parseId(idTok, id) ||
      !isNumberToken(num[0]) || !isNumberToken(num[1]) ||
      !isNumberToken(num[2]) || strtok(NULL, SEP) != NULL) {
    nBad++;
    return;
  }
  if (id != ROBOT_ID) {
    nWrongId++;
    return;
  }

  char line[96];
  const int len = snprintf(line, sizeof(line), "CMD %s %s %s\n",
                           num[0], num[1], num[2]);
  if (len <= 0 || len >= (int)sizeof(line)) {
    nBad++;
    return;
  }
  OpenRB.write((const uint8_t*)line, len);
  nFwd++;
  flashLed();
}

void pollUdp() {
  int size;
  while ((size = udp.parsePacket()) > 0) {
    nRx++;
    char buf[96];
    if (size >= (int)sizeof(buf)) {  // oversize: drop (the rest is discarded by the next parsePacket)
      nBad++;
      continue;
    }
    const int len = udp.read(buf, sizeof(buf) - 1);
    if (len <= 0) {
      nBad++;
      continue;
    }
    buf[len] = '\0';
    handleDatagram(buf);
  }
}

// ---------------- WiFi ----------------

void setupWiFi() {
  snprintf(hostname, sizeof(hostname), "herover-%d", ROBOT_ID);
  WiFi.setHostname(hostname);  // before mode()/begin() so DHCP announces it
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);        // modem sleep delays inbound packets by ~100 ms+
#if USE_STATIC_IP
  WiFi.config(IPAddress(STATIC_IP), IPAddress(STATIC_GATEWAY),
              IPAddress(STATIC_SUBNET));
#endif
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("herover_esp32 id %d: connecting to '%s'\n", ROBOT_ID, WIFI_SSID);
}

void serviceWiFi() {
  const bool connected = WiFi.status() == WL_CONNECTED;
  const uint32_t now = millis();

  if (connected && !wasConnected) {
    Serial.printf("WiFi up: %s  ip %s  udp port %d\n", hostname,
                  WiFi.localIP().toString().c_str(), UDP_PORT);
    if (!netStarted) {
      // Bound to INADDR_ANY, so the socket survives reconnects.
      udp.begin(UDP_PORT);
      if (!MDNS.begin(hostname)) Serial.println("mDNS start failed");
      netStarted = true;
    }
  } else if (!connected && wasConnected) {
    Serial.println("WiFi lost, reconnecting");
    lastReconnectMs = now;
  }
  // Belt and braces: auto-reconnect occasionally gives up (e.g. AP reboot).
  if (!connected && now - lastReconnectMs > 10000) {
    WiFi.reconnect();
    lastReconnectMs = now;
  }
  wasConnected = connected;
}

void printStats() {
  const uint32_t now = millis();
  if (now - lastStatsMs < 5000) return;
  lastStatsMs = now;
  if (nRx == lastStatsRx) return;  // quiet while idle
  lastStatsRx = nRx;
  Serial.printf("rx %lu  fwd %lu  bad %lu  wrong_id %lu\n",
                (unsigned long)nRx, (unsigned long)nFwd,
                (unsigned long)nBad, (unsigned long)nWrongId);
}

// ---------------- setup / loop ----------------

void setup() {
  Serial.begin(115200);  // USB diagnostics only
#if USE_STATUS_LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
#endif
  OpenRB.begin(OPENRB_BAUD, SERIAL_8N1, OPENRB_RX_PIN, OPENRB_TX_PIN);
  setupWiFi();
}

void loop() {
  serviceWiFi();
  if (netStarted) pollUdp();
  serviceLed();
  printStats();
  delay(1);  // let the idle task run (single-core C3/C6); lwIP queues datagrams meanwhile
}
