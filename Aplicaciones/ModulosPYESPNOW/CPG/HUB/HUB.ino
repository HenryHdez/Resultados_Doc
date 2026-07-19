#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>

static const uint8_t WIFI_CH = 1;

// Broadcast real
static const uint8_t BCAST_FF[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ===== Protocolo base + CPG =====
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

// Unificación ADC (3 canales)
typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  node_id;
  uint16_t in1;
  uint16_t in2;
  uint16_t in3; // en nodos 1-3 corresponde a IN_4, en 4-6 corresponde a IN_3
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

// ===== Mensajes CPG / Obstáculo =====
typedef struct __attribute__((packed)) {
  uint8_t  type;         // MSG_CPG
  uint8_t  target_id;    // 0 = todos, o 1..6
  uint16_t seq;
  uint32_t t_ms;
  uint16_t phi_u16;      // 0..65535 -> 0..2π
  uint16_t f_milliHz;    // Hz*1000
  uint16_t amp_permille; // 0..1000
  uint16_t dphi_u16;     // desfase por módulo (u16)
  uint8_t  mode;         // 0 normal, 1 avoid, 2 stop
  int8_t   turn_sign;    // -1 izq, +1 der, 0
  uint16_t avoid_gain;   // 0..65535 (informativo)
} cpg_msg_t;

typedef struct __attribute__((packed)) {
  uint8_t  type;       // MSG_OBS
  uint8_t  node_id;    // 1..6
  uint16_t seq_last;
  uint16_t s_u16;      // 0..65535
  int8_t   side;       // -1 izq, +1 der, 0
  uint16_t adc_raw;    // lectura cruda
  uint32_t t_ms;
} obs_msg_t;

// ===== Estado peers =====
static bool     peerKnown[7] = {false};
static uint8_t  peerMac[7][6] = {{0}};
static uint32_t lastSeen[7] = {0};

// ===== Estado CPG =====
static uint16_t g_seq = 0;
static uint16_t g_phi = 0;
static uint32_t lastTickMs = 0;
static uint32_t lastSendMs = 0;

// Parámetros (puede ajustar por Serial)
static uint16_t F_MILLIHZ      = 500;   // 0.5 Hz
static uint16_t AMP_PERMILLE   = 600;   // 60%
static uint16_t DPHI_U16       = (uint16_t)(65536UL / 6); // 2π/6
static uint16_t CPG_PERIOD_MS  = 20;    // 50 Hz

// Evasión global por eventos
static volatile uint32_t g_lastObsMs = 0;
static volatile int8_t   g_lastSide  = 0;
static volatile uint16_t g_lastS     = 0;
static const uint32_t    AVOID_HOLD_MS = 900;

// ===== Helpers =====
static void printMac(const uint8_t *m) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X", m[0],m[1],m[2],m[3],m[4],m[5]);
}

static bool addPeerIfNeeded(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = WIFI_CH;
  p.encrypt = false;
  return (esp_now_add_peer(&p) == ESP_OK);
}

// ===== RX callback (IDF5) =====
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !data || len < 2) return;
  const uint8_t *src = info->src_addr;
  uint8_t type = data[0];

  if (type == MSG_HELLO && len == (int)sizeof(hello_msg_t)) {
    hello_msg_t h; memcpy(&h, data, sizeof(h));
    if (h.node_id >= 1 && h.node_id <= 6) {
      memcpy(peerMac[h.node_id], src, 6);
      peerKnown[h.node_id] = true;
      lastSeen[h.node_id] = millis();

      addPeerIfNeeded(src);

      ack_msg_t a{MSG_ACK, h.node_id};
      esp_now_send(src, (uint8_t*)&a, sizeof(a));

      Serial.print("# HELLO from N"); Serial.print(h.node_id);
      Serial.print(" MAC "); printMac(src); Serial.println();
    }
    return;
  }

  if (type == MSG_ADC && len == (int)sizeof(adc_msg_t)) {
    adc_msg_t m; memcpy(&m, data, sizeof(m));
    if (m.node_id >= 1 && m.node_id <= 6) {
      lastSeen[m.node_id] = millis();
      // salida por Serial (opcional)
      Serial.printf("# ADC N%u %u,%u,%u\n", m.node_id, m.in1, m.in2, m.in3);
    }
    return;
  }

  if (type == MSG_TEXT && len == (int)sizeof(text_msg_t)) {
    text_msg_t t; memcpy(&t, data, sizeof(t));
    Serial.printf("# TEXT N%u: %s\n", t.node_id, t.text);
    return;
  }

  if (type == MSG_OBS && len == (int)sizeof(obs_msg_t)) {
    obs_msg_t o; memcpy(&o, data, sizeof(o));
    g_lastObsMs = millis();
    g_lastS = o.s_u16;
    if (o.side != 0) g_lastSide = o.side;

    Serial.printf("# OBS N%u s=%u side=%d adc=%u\n",
                  o.node_id, o.s_u16, (int)o.side, o.adc_raw);
    return;
  }
}

// ===== modo global =====
static uint8_t computeMode(int8_t &turnSign, uint16_t &avoidGain) {
  uint32_t now = millis();
  if ((now - g_lastObsMs) <= AVOID_HOLD_MS && g_lastS > 0) {
    turnSign = (g_lastSide == 0) ? +1 : g_lastSide;
    avoidGain = g_lastS;
    return 1; // avoid
  }
  turnSign = 0;
  avoidGain = 0;
  return 0; // normal
}

static void setFixedChannel() {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  Serial.print("# HUB MAC ");
  Serial.println(WiFi.macAddress());

  setFixedChannel();

  if (esp_now_init() != ESP_OK) {
    Serial.println("# ERR esp_now_init");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // necesario para enviar broadcast 
  addPeerIfNeeded(BCAST_FF);

  lastTickMs = millis();
  lastSendMs = millis();
}

void loop() {
  uint32_t now = millis();

  // Integrar fase cuantizada (sin float)
  uint32_t dtMs = now - lastTickMs;
  lastTickMs = now;
  uint32_t inc = (uint32_t)F_MILLIHZ * dtMs * 65536UL / 1000000UL;
  g_phi = (uint16_t)(g_phi + (uint16_t)inc);

  // Enviar CPG por broadcast cada CPG_PERIOD_MS
  if (now - lastSendMs >= CPG_PERIOD_MS) {
    lastSendMs = now;

    int8_t turnSign = 0;
    uint16_t avoidGain = 0;
    uint8_t mode = computeMode(turnSign, avoidGain);

    cpg_msg_t c{};
    c.type = MSG_CPG;
    c.target_id = 0; // todos
    c.seq = ++g_seq;
    c.t_ms = now;
    c.phi_u16 = g_phi;
    c.f_milliHz = F_MILLIHZ;
    c.amp_permille = AMP_PERMILLE;
    c.dphi_u16 = DPHI_U16;
    c.mode = mode;
    c.turn_sign = turnSign;
    c.avoid_gain = avoidGain;

    esp_now_send(BCAST_FF, (uint8_t*)&c, sizeof(c));
  }

  // Comandos por Serial (opcional):
  //   F 500
  //   AMP 600
  //   DPHI 10922   (ej 65536/6)
  //   STOP  (modo stop en envío)
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim(); s.toUpperCase();
    if (s.startsWith("F "))    F_MILLIHZ = (uint16_t)s.substring(2).toInt();
    if (s.startsWith("AMP "))  AMP_PERMILLE = (uint16_t)s.substring(4).toInt();
    if (s.startsWith("DPHI ")) DPHI_U16 = (uint16_t)s.substring(5).toInt();
  }

  delay(2);
}

