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
  MSG_OBS   = 7,
  MSG_ADC_METRIC = 8,
  MSG_NET_PING   = 9,
  MSG_NET_PONG   = 10
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

// Versión ampliada. El contador se genera en el nodo emisor; por ello permite
// detectar pérdidas reales entre el nodo y el HUB.
typedef struct __attribute__((packed)) {
  uint8_t  type;       // MSG_ADC_METRIC
  uint8_t  node_id;
  uint32_t seq;
  uint32_t tx_us;
  uint16_t in1;
  uint16_t in2;
  uint16_t in3;
} adc_metric_msg_t;

// Medición de tiempo de ida y vuelta. Requiere que el nodo responda el PING
// copiando run_id, seq y pc_tx_ns en un MSG_NET_PONG.
typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  node_id;
  uint32_t seq;
  uint64_t pc_tx_ns;
  uint32_t hub_tx_us;
  char     run_id[24];
} net_ping_msg_t;

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  node_id;
  uint32_t seq;
  uint64_t pc_tx_ns;
  uint32_t node_rx_us;
  char     run_id[24];
} net_pong_msg_t;

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

// Estadísticas observadas en el HUB. rxCountLegacy no sustituye una secuencia
// del emisor y, por tanto, no se usa para calcular pérdida de paquetes.
static uint32_t rxCountLegacy[7] = {0};
static uint32_t rxCountMetric[7] = {0};
static uint32_t lastMetricSeq[7] = {0};
static uint32_t metricGaps[7] = {0};

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

static int getRssi(const esp_now_recv_info_t *info) {
  if (!info || !info->rx_ctrl) return 0;
  return (int)info->rx_ctrl->rssi;
}

// DATA,run,node,seq,tx_us,hub_rx_us,type,v1,v2,v3,bytes,rssi,status
static void printDataRow(const char *runId, uint8_t nodeId,
                         bool hasSeq, uint32_t seq,
                         bool hasTx, uint32_t txUs,
                         uint32_t hubRxUs, const char *payloadType,
                         uint16_t v1, uint16_t v2, uint16_t v3,
                         int payloadBytes, int rssi, const char *status) {
  Serial.print("DATA,");
  Serial.print(runId ? runId : ""); Serial.print(',');
  Serial.print(nodeId); Serial.print(',');
  if (hasSeq) Serial.print(seq);
  Serial.print(',');
  if (hasTx) Serial.print(txUs);
  Serial.print(','); Serial.print(hubRxUs); Serial.print(',');
  Serial.print(payloadType); Serial.print(',');
  Serial.print(v1); Serial.print(',');
  Serial.print(v2); Serial.print(',');
  Serial.print(v3); Serial.print(',');
  Serial.print(payloadBytes); Serial.print(',');
  Serial.print(rssi); Serial.print(',');
  Serial.println(status);
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
      rxCountLegacy[m.node_id]++;
      // En el formato heredado la secuencia queda vacía: inventarla en el HUB
      // produciría un PDR artificial de 100 %.
      printDataRow("", m.node_id, false, 0, false, 0, micros(), "PROX",
                   m.in1, m.in2, m.in3, len, getRssi(info), "LEGACY");
    }
    return;
  }

  if (type == MSG_ADC_METRIC && len == (int)sizeof(adc_metric_msg_t)) {
    adc_metric_msg_t m; memcpy(&m, data, sizeof(m));
    if (m.node_id >= 1 && m.node_id <= 6) {
      lastSeen[m.node_id] = millis();
      rxCountMetric[m.node_id]++;
      if (lastMetricSeq[m.node_id] != 0 && m.seq > lastMetricSeq[m.node_id] + 1) {
        metricGaps[m.node_id] += m.seq - lastMetricSeq[m.node_id] - 1;
      }
      lastMetricSeq[m.node_id] = m.seq;
      printDataRow("", m.node_id, true, m.seq, true, m.tx_us, micros(), "PROX",
                   m.in1, m.in2, m.in3, len, getRssi(info), "METRIC");
    }
    return;
  }

  if (type == MSG_NET_PONG && len == (int)sizeof(net_pong_msg_t)) {
    net_pong_msg_t p; memcpy(&p, data, sizeof(p));
    p.run_id[sizeof(p.run_id) - 1] = '\0';
    Serial.printf("ACK,%s,%u,%lu,%llu,%lu\n",
                  p.run_id, p.node_id, (unsigned long)p.seq,
                  (unsigned long long)p.pc_tx_ns,
                  (unsigned long)p.node_rx_us);
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

  Serial.println("# FORMAT DATA,run,node,seq,tx_us,hub_rx_us,type,v1,v2,v3,bytes,rssi,status");
  Serial.println("# HUB measurement firmware ready");
}

static void handleSerialCommand(String s) {
  s.trim();
  if (s.length() == 0) return;

  // PING,run_id,node_id,seq,pc_tx_ns
  if (s.startsWith("PING,")) {
    char buffer[128];
    s.toCharArray(buffer, sizeof(buffer));
    char *save = nullptr;
    strtok_r(buffer, ",", &save); // PING
    char *run = strtok_r(nullptr, ",", &save);
    char *nodeText = strtok_r(nullptr, ",", &save);
    char *seqText = strtok_r(nullptr, ",", &save);
    char *timeText = strtok_r(nullptr, ",", &save);

    if (!run || !nodeText || !seqText || !timeText) {
      Serial.println("# ERR malformed PING");
      return;
    }

    uint8_t nodeId = (uint8_t)atoi(nodeText);
    if (nodeId < 1 || nodeId > 6 || !peerKnown[nodeId]) {
      Serial.printf("# ERR PING unknown node %u\n", nodeId);
      return;
    }

    net_ping_msg_t p{};
    p.type = MSG_NET_PING;
    p.node_id = nodeId;
    p.seq = (uint32_t)strtoul(seqText, nullptr, 10);
    p.pc_tx_ns = (uint64_t)strtoull(timeText, nullptr, 10);
    p.hub_tx_us = micros();
    strncpy(p.run_id, run, sizeof(p.run_id) - 1);

    esp_err_t result = esp_now_send(peerMac[nodeId], (uint8_t*)&p, sizeof(p));
    if (result != ESP_OK) {
      Serial.printf("# ERR PING send node=%u code=%d\n", nodeId, (int)result);
    }
    return;
  }

  String upper = s;
  upper.toUpperCase();
  if (upper.startsWith("F ")) {
    F_MILLIHZ = (uint16_t)upper.substring(2).toInt();
  } else if (upper.startsWith("AMP ")) {
    AMP_PERMILLE = (uint16_t)upper.substring(4).toInt();
  } else if (upper.startsWith("DPHI ")) {
    DPHI_U16 = (uint16_t)upper.substring(5).toInt();
  } else if (upper == "STATS") {
    for (uint8_t id = 1; id <= 6; id++) {
      Serial.printf("# STATS N%u known=%u age_ms=%lu legacy=%lu metric=%lu gaps=%lu last_seq=%lu\n",
                    id, peerKnown[id] ? 1 : 0,
                    peerKnown[id] ? (unsigned long)(millis() - lastSeen[id]) : 0UL,
                    (unsigned long)rxCountLegacy[id],
                    (unsigned long)rxCountMetric[id],
                    (unsigned long)metricGaps[id],
                    (unsigned long)lastMetricSeq[id]);
    }
  } else {
    Serial.println("# ERR unknown serial command");
  }
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
    handleSerialCommand(s);
  }

  delay(2);
}
