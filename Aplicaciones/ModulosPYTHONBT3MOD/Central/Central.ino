#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>

// ======== UUIDs ========
static BLEUUID SVC_UUID("12345678-1234-1234-1234-1234567890ab");
static BLEUUID CHR_NOTIFY_UUID("12345678-1234-1234-1234-1234567890ac");
static BLEUUID CHR_WRITE_UUID ("12345678-1234-1234-1234-1234567890ad");

// ======== Configuración ========
static const int MAX_NODES = 6;
static const uint32_t CONNECT_RETRY_MS = 2000;

// Nombres de los nodos
static const char* NODE_NAMES[MAX_NODES] = {
  "ROBOT_1", "ROBOT_2", "ROBOT_3",
  "ROBOT_4", "ROBOT_5", "ROBOT_6"
};

// Payload binario recibido 
typedef struct __attribute__((packed)) {
  uint8_t  node_id;
  uint16_t in1;
  uint16_t in2;
  uint16_t in4;
} adc_payload_t;

struct NodeConn {
  const char* name;

  BLEAddress addr = BLEAddress("00:00:00:00:00:00");
  bool haveAddr = false;

  BLEClient* client = nullptr;
  BLERemoteCharacteristic* chrNotify = nullptr;
  BLERemoteCharacteristic* chrWrite  = nullptr;

  bool connected = false;
  uint32_t lastConnAttemptMs = 0;
};

NodeConn nodes[MAX_NODES] = {
  { NODE_NAMES[0] }, { NODE_NAMES[1] }, { NODE_NAMES[2] },
  { NODE_NAMES[3] }, { NODE_NAMES[4] }, { NODE_NAMES[5] }
};

// ======== Imprime CSV ========
static void notifyCB(BLERemoteCharacteristic* c, uint8_t* data, size_t len, bool isNotify) {
  (void)c; (void)isNotify;
  if (len != sizeof(adc_payload_t)) return;

  adc_payload_t p;
  memcpy(&p, data, sizeof(p));

  // CSV: node_id,in1,in2,in4
  Serial.print(p.node_id);
  Serial.print(",");
  Serial.print(p.in1);
  Serial.print(",");
  Serial.print(p.in2);
  Serial.print(",");
  Serial.println(p.in4);
}

// ======== Escaneo por nombre ========
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice adv) override {
    if (!adv.haveName()) return;

    String n = adv.getName();
    for (int i = 0; i < MAX_NODES; i++) {
      if (!nodes[i].haveAddr && n == nodes[i].name) {
        nodes[i].addr = adv.getAddress();
        nodes[i].haveAddr = true;

        // Log de control (prefijo # para que Python lo ignore)
        Serial.print("# FOUND ");
        Serial.print(nodes[i].name);
        Serial.print(" @ ");
        Serial.println(nodes[i].addr.toString().c_str());
        break;
      }
    }
  }
};

static bool connectNode(int i) {
  NodeConn &n = nodes[i];
  if (!n.haveAddr) return false;

  uint32_t now = millis();
  if (now - n.lastConnAttemptMs < CONNECT_RETRY_MS) return false;
  n.lastConnAttemptMs = now;

  if (n.client == nullptr) {
    n.client = BLEDevice::createClient();
  }

  if (n.client->isConnected()) {
    n.connected = true;
    return true;
  }

  Serial.print("# CONNECT ");
  Serial.println(n.name);

  if (!n.client->connect(n.addr)) {
    Serial.print("# CONNECT_FAIL ");
    Serial.println(n.name);
    n.connected = false;
    return false;
  }

  BLERemoteService* svc = n.client->getService(SVC_UUID);
  if (!svc) {
    Serial.print("# NO_SERVICE ");
    Serial.println(n.name);
    n.client->disconnect();
    n.connected = false;
    return false;
  }

  n.chrNotify = svc->getCharacteristic(CHR_NOTIFY_UUID);
  n.chrWrite  = svc->getCharacteristic(CHR_WRITE_UUID);
  if (!n.chrNotify || !n.chrWrite) {
    Serial.print("# NO_CHARS ");
    Serial.println(n.name);
    n.client->disconnect();
    n.connected = false;
    return false;
  }

  if (n.chrNotify->canNotify()) {
    n.chrNotify->registerForNotify(notifyCB);
  }

  n.connected = true;
  Serial.print("# CONNECT_OK ");
  Serial.println(n.name);
  return true;
}

static void scanShort1s() {
  static bool init = false;
  static BLEScan* scan = nullptr;

  if (!init) {
    scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(60);
    init = true;
  }
  scan->start(1, false);
  scan->clearResults();
}

// ======== Bridge Serial -> BLE Write ========
// Comandos por Serial:
//   N <id> ANG <deg>
//   N <id> POS <0..1023>
//   N <id> HOME
//   N <id> SPD <0..1023>
static void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  // Ignorar comentarios
  if (line.startsWith("#")) return;
  // Ej: "N 3 ANG 45"
  // Partir por espacios
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

  int idx = node_id - 1;
  if (!nodes[idx].connected || !nodes[idx].chrWrite) {
    Serial.println("# ERR node not connected");
    return;
  }
  nodes[idx].chrWrite->writeValue(cmd.c_str());
  Serial.print("# SENT N");
  Serial.print(node_id);
  Serial.print(" ");
  Serial.println(cmd);
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("# CENTRAL_START");
  BLEDevice::init("CENTRAL_ESP32");
}

void loop() {
  // 1) Escanear si faltan direcciones o hay nodos desconectados
  bool needScan = false;
  for (int i = 0; i < MAX_NODES; i++) {
    if (!nodes[i].haveAddr) { needScan = true; break; }
    if (nodes[i].haveAddr && !nodes[i].connected) needScan = true;
  }
  if (needScan) scanShort1s();

  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].connected && nodes[i].client && !nodes[i].client->isConnected()) {
      nodes[i].connected = false;
      nodes[i].chrNotify = nullptr;
      nodes[i].chrWrite  = nullptr;
      Serial.print("# DROPPED ");
      Serial.println(nodes[i].name);
    }
    if (nodes[i].haveAddr && !nodes[i].connected) {
      connectNode(i);
    }
  }

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
