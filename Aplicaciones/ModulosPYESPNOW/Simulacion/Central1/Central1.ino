/*  HUB ESP-NOW (IDF5 / Arduino-ESP32 v3.x) - VERSION EXTENDIDA (1..9)
    - Robots (sin cambios): IDs 1..6 siguen enviando MSG_ADC y el HUB imprime:
        node_id,in1,in2,in4
    - Nuevos nodos humedad (AS7262): IDs 7..9 envían:
        * MSG_SPEC (6 canales)  -> HUB imprime: SPEC,node_id,ch0,ch1,ch2,ch3,ch4,ch5
        * MSG_HUM  (humedad)    -> HUB imprime: HUM,node_id,hum_permille
          (hum_permille: 0..1000 equivale a 0.0%..100.0%)

    - Comandos desde Python por Serial (USB) se mantienen para IDs 1..6 (o para cualquier ID conocido):
        N <id> HOME
        N <id> SPD <0..1023>
        N <id> POS <0..1023>
        N <id> ANG <deg>

    Nota: AS7262 es espectral (no mide humedad directa). "HUM" se asume calculada en el nodo
    (proxy/calibración). Alternativamente puede ignorar HUM y usar SPEC para modelar en Python.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ===================== Configuración =====================
static const int MAX_NODES = 9;              // <<< EXTENDIDO: 1..9 (robots 1..6 + sensores 7..9)
static const uint8_t WIFI_CH = 1;            // canal fijo (igual en todos)
static const uint32_t NODE_TTL_MS = 20000;   // offline si no llega nada en 20 s

// Contexto enviado por Python al iniciar cada método. No modifica el control;
// únicamente relaciona las mediciones con la corrida experimental.
static char CURRENT_RUN_ID[24] = "SIN_RUN";
static char CURRENT_METHOD[32] = "SIN_METODO";

// ===================== Protocolo =========================
enum MsgType : uint8_t {
  MSG_HELLO = 1,
  MSG_ACK   = 2,
  MSG_ADC   = 3,
  MSG_CMD   = 4,
  MSG_SPEC  = 5,   // <<< nuevo (AS7262 espectro)
  MSG_HUM   = 6    // <<< nuevo (humedad/proxy ya calculada en nodo)
};

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t node_id;   // 1..9
} hello_msg_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t node_id;   // 1..9
} ack_msg_t;

// Robots existentes: ADC (3 canales)
typedef struct __attribute__((packed)) {
  uint8_t  type;     // MSG_ADC
  uint8_t  node_id;  // 1..9
  uint16_t in1;
  uint16_t in2;
  uint16_t in4;
} adc_msg_t;

// Comandos (texto)
typedef struct __attribute__((packed)) {
  uint8_t type;      // MSG_CMD
  uint8_t node_id;   // 1..9
  char    cmd[32];   // "HOME" / "SPD 120" / "POS 700" / "ANG 45"
} cmd_msg_t;

// AS7262: 6 canales (escalados a uint16)
typedef struct __attribute__((packed)) {
  uint8_t  type;     // MSG_SPEC
  uint8_t  node_id;  // 1..9
  uint16_t ch[6];    // 6 canales
} spec_msg_t;

// Humedad/proxy: 0..1000 (permille)
typedef struct __attribute__((packed)) {
  uint8_t  type;      // MSG_HUM
  uint8_t  node_id;   // 1..9
  uint16_t hum_permille; // 0..1000
} hum_msg_t;

// ===================== Estado de nodos ===================
struct NodeState {
  bool     known = false;     // MAC conocida
  bool     online = false;    // visto recientemente
  uint8_t  mac[6] = {0};
  uint32_t lastSeenMs = 0;
};

NodeState nodes[MAX_NODES];

// ===================== Utilidades =========================
static void printMac(const uint8_t *mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}

static bool addPeerIfNeeded(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;

  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = WIFI_CH;
  p.encrypt = false;

  return (esp_now_add_peer(&p) == ESP_OK);
}

static void readMacAddress() {
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

static int getRssi(const esp_now_recv_info_t *info) {
  if (!info || !info->rx_ctrl) return 0;
  return (int)info->rx_ctrl->rssi;
}

// Fila adicional para medición. La salida funcional original se conserva.
// DATA,run,node,seq,tx_us,hub_rx_us,type,v1,v2,v3,bytes,rssi,status
static void printMeasurementRow(uint8_t nodeId, const char *payloadType,
                                uint16_t v1, uint16_t v2, uint16_t v3,
                                int payloadBytes, int rssi) {
  Serial.print("DATA,");
  Serial.print(CURRENT_RUN_ID); Serial.print(',');
  Serial.print(nodeId); Serial.print(',');
  Serial.print(',');                 // seq: ausente en nodos heredados
  Serial.print(',');                 // tx_us: ausente en nodos heredados
  Serial.print(micros()); Serial.print(',');
  Serial.print(payloadType); Serial.print(',');
  Serial.print(v1); Serial.print(',');
  Serial.print(v2); Serial.print(',');
  Serial.print(v3); Serial.print(',');
  Serial.print(payloadBytes); Serial.print(',');
  Serial.print(rssi); Serial.print(',');
  Serial.println("LEGACY");
}

// Registrar nodo (actualiza MAC, marca online)
static void registerNode(uint8_t node_id, const uint8_t *srcMac, const char *reasonTag) {
  if (node_id < 1 || node_id > MAX_NODES) return;
  int idx = node_id - 1;

  bool changed = (!nodes[idx].known) || (memcmp(nodes[idx].mac, srcMac, 6) != 0);
  if (changed) {
    memcpy(nodes[idx].mac, srcMac, 6);
    nodes[idx].known = true;

    Serial.print(reasonTag);
    Serial.print(node_id);
    Serial.print(" @ ");
    printMac(srcMac);
    Serial.println();
  }

  nodes[idx].online = true;
  nodes[idx].lastSeenMs = millis();

  // asegurar peer para envíos unicast
  addPeerIfNeeded(srcMac);
}

// ===================== RX callback (IDF5) =================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (!info || !incomingData || len < 2) return;

  const uint8_t *src = info->src_addr;
  uint8_t type = incomingData[0];

  if (type == MSG_HELLO) {
    if (len != (int)sizeof(hello_msg_t)) return;
    hello_msg_t h;
    memcpy(&h, incomingData, sizeof(h));
    if (h.node_id < 1 || h.node_id > MAX_NODES) return;

    registerNode(h.node_id, src, "# FOUND NODE ");

    ack_msg_t ack{MSG_ACK, h.node_id};
    esp_now_send(src, (uint8_t*)&ack, sizeof(ack));
    return;
  }

  if (type == MSG_ADC) {
    if (len != (int)sizeof(adc_msg_t)) return;
    adc_msg_t p;
    memcpy(&p, incomingData, sizeof(p));
    if (p.node_id < 1 || p.node_id > MAX_NODES) return;

    registerNode(p.node_id, src, "# IMPLICIT REGISTER NODE ");

    // Mantener EXACTO el CSV original para robots (compatibilidad)
    Serial.print(p.node_id);
    Serial.print(",");
    Serial.print(p.in1);
    Serial.print(",");
    Serial.print(p.in2);
    Serial.print(",");
    Serial.println(p.in4);
    printMeasurementRow(p.node_id, "PROX", p.in1, p.in2, p.in4,
                        len, getRssi(info));
    return;
  }

  if (type == MSG_SPEC) {
    if (len != (int)sizeof(spec_msg_t)) return;
    spec_msg_t s;
    memcpy(&s, incomingData, sizeof(s));
    if (s.node_id < 1 || s.node_id > MAX_NODES) return;

    registerNode(s.node_id, src, "# IMPLICIT REGISTER NODE ");

    // CSV distinguible para espectro
    Serial.print("SPEC,");
    Serial.print(s.node_id);
    for (int i = 0; i < 6; i++) {
      Serial.print(",");
      Serial.print(s.ch[i]);
    }
    Serial.println();
    printMeasurementRow(s.node_id, "SPEC", s.ch[0], s.ch[1], s.ch[2],
                        len, getRssi(info));
    return;
  }

  if (type == MSG_HUM) {
    if (len != (int)sizeof(hum_msg_t)) return;
    hum_msg_t h;
    memcpy(&h, incomingData, sizeof(h));
    if (h.node_id < 1 || h.node_id > MAX_NODES) return;

    registerNode(h.node_id, src, "# IMPLICIT REGISTER NODE ");

    // CSV distinguible para humedad (0..1000 -> 0.0%..100.0%)
    Serial.print(h.node_id);
    Serial.print(",");
    Serial.println(h.hum_permille);
    printMeasurementRow(h.node_id, "HUM", h.hum_permille, 0, 0,
                        len, getRssi(info));
    return;
  }
}

// ===================== TX callback (IDF5) =================
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("# TX ");
  if (info && info->des_addr) {
    printMac(info->des_addr);
  } else {
    Serial.print("??:??:??:??:??:??");
  }
  Serial.print(" => ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ===================== Serial -> CMD ======================
static void sendCmdToNode(uint8_t node_id, const String &cmdStr) {
  if (node_id < 1 || node_id > MAX_NODES) {
    Serial.println("# ERR bad node id");
    return;
  }
  int idx = node_id - 1;

  if (!nodes[idx].known) {
    Serial.println("# ERR node unknown (wait for ADC/HELLO/SPEC/HUM)");
    return;
  }

  addPeerIfNeeded(nodes[idx].mac);

  cmd_msg_t m{};
  m.type = MSG_CMD;
  m.node_id = node_id;
  memset(m.cmd, 0, sizeof(m.cmd));
  cmdStr.substring(0, 31).toCharArray(m.cmd, sizeof(m.cmd));

  esp_err_t r = esp_now_send(nodes[idx].mac, (uint8_t*)&m, sizeof(m));
  if (r != ESP_OK) {
    Serial.print("# ERR esp_now_send=");
    Serial.println((int)r);
  } else {
    Serial.print("# SENT N");
    Serial.print(node_id);
    Serial.print(" ");
    Serial.println(cmdStr);
  }
}

static void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line.startsWith("#")) return;

  // RUN,<run_id>,<metodo>. Solo fija contexto de medición; no cambia motores.
  if (line.startsWith("RUN,")) {
    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);
    if (p2 < 0) {
      Serial.println("# ERR malformed RUN");
      return;
    }
    String runId = line.substring(p1 + 1, p2);
    String method = line.substring(p2 + 1);
    runId.trim();
    method.trim();
    memset(CURRENT_RUN_ID, 0, sizeof(CURRENT_RUN_ID));
    memset(CURRENT_METHOD, 0, sizeof(CURRENT_METHOD));
    runId.substring(0, sizeof(CURRENT_RUN_ID) - 1).toCharArray(
      CURRENT_RUN_ID, sizeof(CURRENT_RUN_ID));
    method.substring(0, sizeof(CURRENT_METHOD) - 1).toCharArray(
      CURRENT_METHOD, sizeof(CURRENT_METHOD));
    Serial.print("# RUN SET ");
    Serial.print(CURRENT_RUN_ID);
    Serial.print(" METHOD ");
    Serial.println(CURRENT_METHOD);
    return;
  }

  int p1 = line.indexOf(' ');
  if (p1 < 0) return;

  String t0 = line.substring(0, p1);
  t0.toUpperCase();
  if (t0 != "N") return;

  int p2 = line.indexOf(' ', p1 + 1);
  if (p2 < 0) return;

  int node_id = line.substring(p1 + 1, p2).toInt();
  if (node_id < 1 || node_id > MAX_NODES) {
    Serial.println("# ERR bad node id");
    return;
  }

  String cmd = line.substring(p2 + 1);
  cmd.trim();
  if (cmd.length() == 0) return;

  sendCmdToNode((uint8_t)node_id, cmd);
}

// ===================== Setup/Loop =========================
void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("# HUB_ESPNOW_START");

  WiFi.mode(WIFI_STA);
  Serial.print("# HUB STA MAC ");
  readMacAddress();

  // Canal fijo
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("# ERR initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  Serial.println("# HUB_READY");
  Serial.println("# FORMAT legacy + DATA measurement rows");
}

void loop() {
  // Marcar offline por TTL
  uint32_t now = millis();
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].online && (now - nodes[i].lastSeenMs > NODE_TTL_MS)) {
      nodes[i].online = false;
      Serial.print("# OFFLINE N");
      Serial.println(i + 1);
    }
  }

  // Leer líneas desde Python (USB Serial)
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line.trim();
      if (line.length() > 0) handleSerialLine(line);
      line = "";
    } else {
      line += c;
      if (line.length() > 120) line = "";
    }
  }

  delay(10);
}
