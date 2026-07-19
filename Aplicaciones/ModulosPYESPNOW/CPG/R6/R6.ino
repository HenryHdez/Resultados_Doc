#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>

// ===== LX16A =====
#include "LX16A-bus.h"

// ====== Configuración del nodo ======
#define NODE_ID 6

// ====== ADC ======
#define IN_1 1
#define IN_2 2
#define IN_3 4

// ====== LX16A / UART1 ======
HardwareSerial SerialServo(1);
const uint32_t BAUD_SERVO = 115200;
const int RX_SERVO_PIN = 21;
const int TX_SERVO_PIN = 20;

// Servo ID en el bus
static int lxId = 1;
LX16A motor(lxId, SerialServo);

// ====== WiFi/ESP-NOW ======
static const uint8_t WIFI_CH = 1;
static const uint8_t BCAST_FF[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static bool haveHub = false;
static uint8_t hubMac[6] = {0};
static uint32_t lastHelloMs = 0;

// ====== Protocolo ======
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
  uint16_t in3;
} adc_msg_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t node_id;
  char    cmd[32];
} cmd_msg_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t node_id;
  char    text[32];
} text_msg_t;

// ===== CPG / OBS =====
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
  int8_t   turn_sign;
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

// ====== Utilidades ======
static bool addPeerIfNeeded(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = WIFI_CH;
  p.encrypt = false;
  return (esp_now_add_peer(&p) == ESP_OK);
}

static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void sendTextToHub(const String &s) {
  text_msg_t m{};
  m.type = MSG_TEXT;
  m.node_id = (uint8_t)NODE_ID;
  memset(m.text, 0, sizeof(m.text));
  s.substring(0, 31).toCharArray(m.text, sizeof(m.text));

  if (haveHub) {
    addPeerIfNeeded(hubMac);
    esp_now_send(hubMac, (uint8_t*)&m, sizeof(m));
  } else {
    addPeerIfNeeded(BCAST_FF);
    esp_now_send(BCAST_FF, (uint8_t*)&m, sizeof(m));
  }
}

static void sendObs(float s, uint16_t adc_raw, int8_t side, uint16_t seq_last) {
  obs_msg_t o{};
  o.type = MSG_OBS;
  o.node_id = (uint8_t)NODE_ID;
  o.seq_last = seq_last;
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

// ====== Parseo comandos (desde HUB) ======
static void handleCmd(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String up = cmd;
  up.toUpperCase();

  if (up == "PING") {
    sendTextToHub("PONG");
    return;
  }

  if (up.startsWith("TORQ")) {
    int v = cmd.substring(4).toInt();
    if (v == 0) { motor.disableTorque(); sendTextToHub("TORQ_OK 0"); }
    else        { motor.enableTorque();  sendTextToHub("TORQ_OK 1"); }
    return;
  }

  if (up.startsWith("LX")) {
    int ang = cmd.substring(2).toInt();
    ang = clampInt(ang, 0, 240);
    motor.move(ang);
    sendTextToHub(String("LX_OK ") + String(ang));
    return;
  }

  sendTextToHub("ERR_CMD");
}

// ====== RX callback (IDF5) ======
static volatile bool haveCpg = false;
static cpg_msg_t lastCpg{};
static uint16_t lastCpgSeq = 0;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (!info || !incomingData || len < 2) return;

  const uint8_t *src = info->src_addr;
  uint8_t type = incomingData[0];

  if (type == MSG_ACK) {
    if (len != (int)sizeof(ack_msg_t)) return;
    ack_msg_t a; memcpy(&a, incomingData, sizeof(a));
    if (a.node_id != NODE_ID) return;

    memcpy(hubMac, src, 6);
    haveHub = true;
    addPeerIfNeeded(hubMac);
    return;
  }

  if (type == MSG_CMD) {
    if (len != (int)sizeof(cmd_msg_t)) return;
    cmd_msg_t m; memcpy(&m, incomingData, sizeof(m));
    if (m.node_id != NODE_ID) return;
    handleCmd(String(m.cmd));
    return;
  }

  if (type == MSG_CPG) {
    if (len != (int)sizeof(cpg_msg_t)) return;
    cpg_msg_t c; memcpy(&c, incomingData, sizeof(c));
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

// ====== CPG ejecución LX ======
static const int LX_CENTER_DEG = 95;
static const int LX_AMP_MAX_DEG = 30;

static const float kA = 0.60f;
static const float kDelta = (float)M_PI/6.0f;
static const float tauA = 0.20f;
static const float tauD = 0.15f;

static float Aeff = 0.0f;
static float delta = 0.0f;

static const uint8_t OBS_TH  = 140;
static const uint8_t OBS_SAT = 220;

static float obstacleStrength8(uint8_t adc8) {
  if (adc8 <= OBS_TH) return 0.0f;
  if (adc8 >= OBS_SAT) return 1.0f;
  return (float)(adc8 - OBS_TH) / (float)(OBS_SAT - OBS_TH);
}

static void runCpgLX() {
  if (!haveCpg) return;

  if (lastCpg.mode == 2) {
    motor.move(LX_CENTER_DEG);
    return;
  }

  static uint32_t lastMs = 0;
  uint32_t now = millis();
  float dt = (lastMs == 0) ? 0.01f : (now - lastMs) / 1000.0f;
  lastMs = now;

  float phi  = (lastCpg.phi_u16 / 65536.0f) * 2.0f * (float)M_PI;
  float dphi = (lastCpg.dphi_u16 / 65536.0f) * 2.0f * (float)M_PI;
  float Phi_i = phi + (NODE_ID - 1) * dphi;

  float A_base = (lastCpg.amp_permille / 1000.0f) * (float)LX_AMP_MAX_DEG;

  float s = 0.0f;
  uint16_t adcRaw = 0;
  if (NODE_ID == 4) {
    uint8_t adc8 = (uint8_t)analogRead(IN_2);
    adcRaw = adc8;
    s = obstacleStrength8(adc8);
  }

  float A_tgt = A_base;
  float delta_tgt = 0.0f;

  if (NODE_ID == 4) {
    A_tgt = A_base * (1.0f - kA * s);
    int8_t sign = (lastCpg.mode == 1) ? lastCpg.turn_sign : 0;
    delta_tgt = (float)sign * (kDelta * s);
  } else {
    if (lastCpg.mode == 1) {
      delta_tgt = (float)lastCpg.turn_sign * (kDelta * 0.15f);
    }
  }

  float aA = dt / (tauA + dt);
  float aD = dt / (tauD + dt);
  Aeff  = Aeff  + aA * (A_tgt - Aeff);
  delta = delta + aD * (delta_tgt - delta);

  float y = (float)LX_CENTER_DEG + Aeff * sinf(Phi_i + delta);
  int ang = (int)lroundf(y);
  ang = clampInt(ang, 0, 240);
  motor.move(ang);

  if (NODE_ID == 4) {
    static uint32_t lastObsSend = 0;
    if (s > 0.10f && (now - lastObsSend > 150)) {
      lastObsSend = now;
      int8_t side = (lastCpg.mode == 1) ? lastCpg.turn_sign : 0;
      sendObs(s, adcRaw, side, lastCpgSeq);
    }
  }
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  delay(200);

  // ADC
  analogReadResolution(8);
  analogSetAttenuation(ADC_11db);

  // UART Servo
  SerialServo.begin(BAUD_SERVO, SERIAL_8N1, RX_SERVO_PIN, TX_SERVO_PIN);
  delay(50);

  motor.initialize();
  motor.enableTorque();
  motor.setServoMode();
  motor.move(LX_CENTER_DEG);

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  setFixedChannel();

  if (esp_now_init() != ESP_OK) {
    Serial.println("# ERR esp_now_init");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Permitir broadcast
  addPeerIfNeeded(BCAST_FF);
}

// ====== Loop ======
void loop() {
  uint32_t now = millis();

  // Handshake: HELLO hasta tener HUB
  if (!haveHub && (now - lastHelloMs > 1000)) {
    lastHelloMs = now;
    hello_msg_t h{MSG_HELLO, (uint8_t)NODE_ID};
    esp_now_send(BCAST_FF, (uint8_t*)&h, sizeof(h));
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
    p.in3 = (uint16_t)analogRead(IN_3);

    if (haveHub) {
      addPeerIfNeeded(hubMac);
      esp_now_send(hubMac, (uint8_t*)&p, sizeof(p));
    } else {
      esp_now_send(BCAST_FF, (uint8_t*)&p, sizeof(p));
    }
  }

  // Ejecutar CPG
  runCpgLX();

  delay(5);
}

