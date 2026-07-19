#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>

#include "AX12A.h"

// ======== Configuración del nodo ========
#define NODE_ID 2

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
  MSG_CMD   = 4,
  MSG_TEXT  = 5,
  MSG_CPG   = 6,
  MSG_OBS   = 7
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

// ===== CPG =====
typedef struct __attribute__((packed)) {
  uint8_t  type;         // MSG_CPG
  uint8_t  target_id;    // 0=all o 1..6
  uint16_t seq;
  uint32_t t_ms;
  uint16_t phi_u16;
  uint16_t f_milliHz;
  uint16_t amp_permille; // 0..1000
  uint16_t dphi_u16;
  uint8_t  mode;         // 0 normal, 1 avoid, 2 stop
  int8_t   turn_sign;    // -1 izq +1 der
  uint16_t avoid_gain;
} cpg_msg_t;

typedef struct __attribute__((packed)) {
  uint8_t  type;       // MSG_OBS
  uint8_t  node_id;
  uint16_t seq_last;
  uint16_t s_u16;
  int8_t   side;
  uint16_t adc_raw;
  uint32_t t_ms;
} obs_msg_t;

// ======== Estado ========
static bool haveHub = false;
static uint8_t hubMac[6] = {0};
static uint32_t lastHelloMs = 0;

// Broadcast real
static const uint8_t BCAST_FF[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

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

// ======== CPG estado local ========
static volatile bool haveCpg = false;
static cpg_msg_t lastCpg{};
static uint16_t lastCpgSeq = 0;

// Parámetros locales AX
static const int   AX_AMP_MAX_TICKS = 200;      // amplitud máxima en ticks
static const float kA = 0.60f;                  // reducción amplitud por obstáculo
static const float kDelta = (float)M_PI/6.0f;   // hasta 30°
static const float tauA = 0.20f;                // s
static const float tauD = 0.15f;                // s
static float Aeff = 0.0f;
static float delta = 0.0f;

// Umbrales obstáculo (ADC 8 bits: 0..255) para IN_2
static const uint8_t OBS_TH  = 140;
static const uint8_t OBS_SAT = 220;

static float clamp01(float x){ return (x<0)?0:((x>1)?1:x); }

// Si el sensor aumenta al acercarse, esta función es directa.
// Si el sensor disminuye al acercarse, invertir: adc8 = 255 - adc8;
static float obstacleStrength8(uint8_t adc8) {
  if (adc8 <= OBS_TH) return 0.0f;
  if (adc8 >= OBS_SAT) return 1.0f;
  return (float)(adc8 - OBS_TH) / (float)(OBS_SAT - OBS_TH);
}

static void sendObs(float s, uint16_t adc_raw, int8_t side) {
  obs_msg_t o{};
  o.type = MSG_OBS;
  o.node_id = (uint8_t)NODE_ID;
  o.seq_last = lastCpgSeq;
  o.s_u16 = (uint16_t)lroundf(s * 65535.0f);
  o.side = side;
  o.adc_raw = adc_raw;
  o.t_ms = millis();

  if (haveHub) {
    addPeerIfNeeded(hubMac);
    esp_now_send(hubMac, (uint8_t*)&o, sizeof(o));
  } else {
    addPeerIfNeeded(BCAST_FF);
    esp_now_send(BCAST_FF, (uint8_t*)&o, sizeof(o));
  }
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

  if (type == MSG_CPG) {
    if (len != (int)sizeof(cpg_msg_t)) return;
    cpg_msg_t c;
    memcpy(&c, incomingData, sizeof(c));
    if (c.target_id != 0 && c.target_id != NODE_ID) return;

    lastCpg = c;
    lastCpgSeq = c.seq;
    haveCpg = true;
    return;
  }
}

static void setFixedChannel() {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

// ======== CPG ejecución ========
static void runCpgAX() {
  if (!haveCpg) return;

  // STOP
  if (lastCpg.mode == 2) {
    axMovePos(START_POS);
    return;
  }

  static uint32_t lastMs = 0;
  uint32_t now = millis();
  float dt = (lastMs == 0) ? 0.01f : (now - lastMs) / 1000.0f;
  lastMs = now;

  // fase global y desfase por módulo
  float phi  = (lastCpg.phi_u16 / 65536.0f) * 2.0f * (float)M_PI;
  float dphi = (lastCpg.dphi_u16 / 65536.0f) * 2.0f * (float)M_PI;
  float Phi_i = phi + (NODE_ID - 1) * dphi;

  // amplitud base normalizada
  float A_base = (lastCpg.amp_permille / 1000.0f) * (float)AX_AMP_MAX_TICKS;

  // Por defecto no hay obstáculo local, excepto NODE_ID==1 (sensor IN_2)
  float s = 0.0f;
  uint16_t adcRaw = 0;
  if (NODE_ID == 1) {
    uint8_t adc8 = (uint8_t)analogRead(IN_2);
    adcRaw = adc8;
    s = obstacleStrength8(adc8);
  }

  // Objetivos (reflejo local solo en nodo 1)
  float A_tgt = A_base;
  float delta_tgt = 0.0f;

  if (NODE_ID == 1) {
    A_tgt = A_base * (1.0f - kA * s);
    // si hay obstáculo local, el signo puede venir del modo global (turn_sign)
    int8_t sign = (lastCpg.mode == 1) ? lastCpg.turn_sign : 0;
    delta_tgt = (float)sign * (kDelta * s);
  } else {
    // nodos 2-3: sin reflejo local, pero en modo avoid pueden adoptar una pequeña fase global
    if (lastCpg.mode == 1) {
      delta_tgt = (float)lastCpg.turn_sign * (kDelta * 0.15f);
    }
  }

  // filtrado
  float aA = dt / (tauA + dt);
  float aD = dt / (tauD + dt);
  Aeff  = Aeff  + aA * (A_tgt - Aeff);
  delta = delta + aD * (delta_tgt - delta);

  float y = (float)START_POS + Aeff * sinf(Phi_i + delta);
  axMovePos((int)lroundf(y));

  // reporte obstáculo (solo nodo 1)
  if (NODE_ID == 1) {
    static uint32_t lastObsSend = 0;
    if (s > 0.10f && (now - lastObsSend > 150)) {
      lastObsSend = now;
      int8_t side = (lastCpg.mode == 1) ? lastCpg.turn_sign : 0;
      sendObs(s, adcRaw, side);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // ADC
  analogReadResolution(8);
  analogSetAttenuation(ADC_11db);

  // AX-12A
  Serial1.begin(AX_BAUD, SERIAL_8N1, 21, 20);
  ax12a.begin(AX_BAUD, AX_DIR_PIN, &Serial1);
  axMovePos(START_POS);

  // WiFi + canal fijo
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  Serial.print("# NODE STA MAC ");
  Serial.println(WiFi.macAddress());

  setFixedChannel();

  if (esp_now_init() != ESP_OK) {
    Serial.println("# ERR initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Permitir broadcast FF
  addPeerIfNeeded(BCAST_FF);
}

void loop() {
  uint32_t now = millis();

  // HELLO por broadcast hasta tener HUB
  if (!haveHub && (now - lastHelloMs > 1000)) {
    lastHelloMs = now;
    hello_msg_t h{MSG_HELLO, (uint8_t)NODE_ID};
    esp_now_send(BCAST_FF, (uint8_t*)&h, sizeof(h));
    Serial.println("# HELLO");
  }

  // ADC cada 250 ms (mantiene su esquema)
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
      esp_now_send(BCAST_FF, (uint8_t*)&p, sizeof(p));
    }
  }

  // Ejecutar CPG (si hay)
  runCpgAX();

  delay(5);
}
