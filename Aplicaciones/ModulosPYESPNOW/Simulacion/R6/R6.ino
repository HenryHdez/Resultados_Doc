#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ===== LX16A =====
#include "LX16A-bus.h"

// ====== Configuración del nodo ======
#define NODE_ID 6

// ====== ADC ======
#define IN_1 1       // GPIO1  (ADC)  (verificar en tu placa)
#define IN_2 2       // GPIO2  (ADC)
#define IN_3 4       // GPIO4  (ADC)

// ====== LX16A / UART1 ======
HardwareSerial SerialServo(1);
const uint32_t BAUD_SERVO = 115200;
const int RX_SERVO_PIN = 21;
const int TX_SERVO_PIN = 20;

// Servo ID en el bus
static int lxId = 1;
LX16A motor(lxId, SerialServo);

// ====== WiFi/ESP-NOW ======
static const uint8_t WIFI_CH = 1;            // <<< IGUAL que el HUB
static const uint8_t BCAST[6] = {0x14,0x2B,0x2F,0xC8,0xE8,0x14};

static bool haveHub = false;
static uint8_t hubMac[6] = {0};
static uint32_t lastHelloMs = 0;

// ====== Protocolo (igual al HUB) ======
enum MsgType : uint8_t {
  MSG_HELLO = 1,
  MSG_ACK   = 2,
  MSG_ADC   = 3,
  MSG_CMD   = 4,
  MSG_TEXT  = 5   
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
  uint8_t type;      // MSG_TEXT
  uint8_t node_id;
  char    text[32];  // "PONG", "LX_OK 120", etc.
} text_msg_t;

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
    // Si aún no hay HUB, por debug podemos broadcast (opcional)
    addPeerIfNeeded(BCAST);
    esp_now_send(BCAST, (uint8_t*)&m, sizeof(m));
  }
}

// ====== Parseo comandos (desde HUB) ======
// Comandos esperados:
//   LX <0..240>     ej: "LX 120"
//   TORQ <0|1>      ej: "TORQ 1"
//   PING            ej: "PING"
static void handleCmd(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String up = cmd;
  up.toUpperCase();

  // PING -> PONG
  if (up == "PING") {
    sendTextToHub("PONG");
    return;
  }

  // TORQ 0/1
  if (up.startsWith("TORQ")) {
    int v = cmd.substring(4).toInt();
    if (v == 0) {
      motor.disableTorque();
      sendTextToHub("TORQ_OK 0");
    } else {
      motor.enableTorque();
      sendTextToHub("TORQ_OK 1");
    }
    return;
  }

  // LX <angulo>
  if (up.startsWith("LX")) {
    int ang = cmd.substring(2).toInt();
    ang = clampInt(ang, 0, 240);
    motor.move(ang);
    sendTextToHub(String("LX_OK ") + String(ang));
    return;
  }

  // comando desconocido
  sendTextToHub("ERR_CMD");
}

// ====== RX callback (IDF5) ======
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
    return;
  }

  if (type == MSG_CMD) {
    if (len != (int)sizeof(cmd_msg_t)) return;
    cmd_msg_t m;
    memcpy(&m, incomingData, sizeof(m));
    if (m.node_id != NODE_ID) return;

    handleCmd(String(m.cmd));
    return;
  }
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);

  // ===== ADC recomendado =====
  analogReadResolution(8);
  analogSetAttenuation(ADC_11db);

  // ===== UART Servo =====
  SerialServo.begin(BAUD_SERVO, SERIAL_8N1, RX_SERVO_PIN, TX_SERVO_PIN);
  delay(50);

  motor.initialize();
  motor.enableTorque();
  motor.setServoMode();
  motor.move(95);

  // ===== WiFi + ESP-NOW =====
  WiFi.mode(WIFI_STA);

  // Canal fijo (evita grupos 1-3 vs 4-6)
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Permitir broadcast y luego HUB unicast
  addPeerIfNeeded(BCAST);
}

// ====== Loop ======
void loop() {
  uint32_t now = millis();

  // 1) Handshake: HELLO hasta tener HUB
  if (!haveHub && (now - lastHelloMs > 1000)) {
    lastHelloMs = now;
    hello_msg_t h{MSG_HELLO, (uint8_t)NODE_ID};
    addPeerIfNeeded(BCAST);
    esp_now_send(BCAST, (uint8_t*)&h, sizeof(h));
  }

  // 2) Envío ADC cada 250 ms
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
      // opcional: mientras no haya HUB, broadcast (ayuda a debug/registro implícito)
      addPeerIfNeeded(BCAST);
      esp_now_send(BCAST, (uint8_t*)&p, sizeof(p));
    }
  }

  delay(5);
}
