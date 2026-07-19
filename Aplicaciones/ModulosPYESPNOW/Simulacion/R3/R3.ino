#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "AX12A.h"

// ======== Configuración del nodo ========
#define NODE_ID 3

// ADC
#define IN_1 1
#define IN_2 2
#define IN_4 4

// AX-12A / UART
#define AX_ID          (1u)
#define AX_BAUD        (115200ul)
#define AX_DIR_PIN     (10u)

static int axSpeed = 250;
static const int START_POS = 512;

// Canal Wi-Fi fijo
static const uint8_t WIFI_CH = 1;

// ======== Protocolo ========
enum MsgType : uint8_t {
  MSG_HELLO = 1,
  MSG_ACK   = 2,
  MSG_ADC   = 3,
  MSG_CMD   = 4
};

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t node_id;
} hello_msg_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t node_id;
} ack_msg_t;

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  node_id;
  uint16_t in1;
  uint16_t in2;
  uint16_t in4;
} adc_msg_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t node_id;
  char    cmd[32];
} cmd_msg_t;

// ======== Estado ========
static bool haveHub = false;
static uint8_t hubMac[6] = {0};
static uint32_t lastHelloMs = 0;

static const uint8_t BCAST[6] = {0x14,0x2B,0x2F,0xC8,0xE8,0x14};

// ======== Motor ========
static int angleToPos(float deg) {
  if (deg > 150.0f) deg = 150.0f;
  if (deg < -150.0f) deg = -150.0f;
  float pos = START_POS + (deg * (512.0f / 150.0f));
  int p = (int)lroundf(pos);
  if (p < 0) p = 0;
  if (p > 1023) p = 1023;
  return p;
}

static void axMovePos(int pos) {
  if (pos < 0) pos = 0;
  if (pos > 1023) pos = 1023;
  ax12a.moveSpeed(AX_ID, pos, axSpeed);
}

static void handleCmd(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String up = cmd;
  up.toUpperCase();

  if (up == "HOME") {
    axMovePos(START_POS);
    return;
  }
  if (up.startsWith("SPD")) {
    int s = (int)cmd.substring(3).toFloat();
    if (s < 0) s = 0;
    if (s > 1023) s = 1023;
    axSpeed = s;
    return;
  }
  if (up.startsWith("POS")) {
    int p = (int)cmd.substring(3).toFloat();
    axMovePos(p);
    return;
  }
  if (up.startsWith("ANG")) {
    float deg = cmd.substring(3).toFloat();
    axMovePos(angleToPos(deg));
    return;
  }
}

// ======== ESP-NOW helpers ========
static bool addPeerIfNeeded(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = WIFI_CH;
  p.encrypt = false;
  return (esp_now_add_peer(&p) == ESP_OK);
}

// ======== RX callback (IDF5) ========
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (!info || !incomingData || len < 2) return;
  const uint8_t *src = info->src_addr;

  uint8_t type = incomingData[0];

  if (type == MSG_ACK) {
    if (len != (int)sizeof(ack_msg_t)) return;
    ack_msg_t a;
    memcpy(&a, incomingData, sizeof(a));
    if (a.node_id != NODE_ID) return;

    memcpy(hubMac, src, 6);
    haveHub = true;
    addPeerIfNeeded(hubMac);

    Serial.print("# HUB_ACK ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                  src[0],src[1],src[2],src[3],src[4],src[5]);
    return;
  }

  if (type == MSG_CMD) {
    if (len != (int)sizeof(cmd_msg_t)) return;
    cmd_msg_t m;
    memcpy(&m, incomingData, sizeof(m));
    if (m.node_id != NODE_ID) return;

    handleCmd(String(m.cmd));
    Serial.print("# CMD ");
    Serial.println(String(m.cmd));
    return;
  }
}

void setup() {
  Serial.begin(115200);

  // ADC
  analogReadResolution(8);
  analogSetAttenuation(ADC_11db);

  // AX-12A
  Serial1.begin(AX_BAUD, SERIAL_8N1, 21, 20);
  ax12a.begin(AX_BAUD, AX_DIR_PIN, &Serial1);
  axMovePos(START_POS);

  // WiFi + canal fijo
  WiFi.mode(WIFI_STA);
  Serial.print("# NODE STA MAC ");
  Serial.println(WiFi.macAddress());

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("# ERR initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Permitir broadcast
  addPeerIfNeeded(BCAST);
}

void loop() {
  uint32_t now = millis();

  // HELLO por broadcast hasta tener HUB
  if (!haveHub && (now - lastHelloMs > 1000)) {
    lastHelloMs = now;
    hello_msg_t h{MSG_HELLO, (uint8_t)NODE_ID};
    esp_now_send(BCAST, (uint8_t*)&h, sizeof(h));
    Serial.println("# HELLO");
  }

  // ADC cada 250 ms
  static uint32_t lastAdcMs = 0;
  if (now - lastAdcMs > 250) {
    lastAdcMs = now;

    adc_msg_t p;
    p.type = MSG_ADC;
    p.node_id = (uint8_t)NODE_ID;
    p.in1 = (uint16_t)analogRead(IN_1);
    p.in2 = (uint16_t)analogRead(IN_2);
    p.in4 = (uint16_t)analogRead(IN_4);

    if (haveHub) {
      addPeerIfNeeded(hubMac);
      esp_now_send(hubMac, (uint8_t*)&p, sizeof(p));
    } else {
      esp_now_send(BCAST, (uint8_t*)&p, sizeof(p));
    }
  }

  delay(5);
}
