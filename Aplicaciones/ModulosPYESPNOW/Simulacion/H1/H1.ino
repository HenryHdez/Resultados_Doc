/*
  NODO AS7341 + LED -> ESP-NOW
  Plataforma : ESP32-C3 Super Mini
  Sensor     : AS7341 (Adafruit)
  I2C        : SDA = GPIO6, SCL = GPIO7

  Función:
  - Enciende LED del AS7341
  - Espera estabilización
  - Lee espectro completo (F1..F8 + CLEAR + NIR)
  - Apaga LED
  - Envía datos por ESP-NOW al HUB
*/

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_AS7341.h>

// ================= CONFIGURACIÓN =================
#define NODE_ID 7                // <<< 7 / 8 / 9
static const uint8_t WIFI_CH = 1;
static const uint32_t SEND_MS = 1200;

// I2C – ESP32-C3 Super Mini
static const int I2C_SDA = 8;
static const int I2C_SCL = 9;

// LED AS7341
static const uint16_t LED_CURRENT_MA = 10; // 4–100 mA
static const uint16_t LED_ON_DELAY_MS = 15; // estabilización óptica

// ================= PROTOCOLO =====================
enum MsgType : uint8_t {
  MSG_HELLO = 1,
  MSG_ACK   = 2,
  MSG_SPEC  = 5,
  MSG_HUM   = 6
};

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t node_id;
} hello_msg_t;

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  node_id;
  uint16_t ch[10];   // F1..F8 + CLEAR + NIR
} spec_msg_t;

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  node_id;
  uint16_t hum_permille;
} hum_msg_t;

// ================= ESTADO ========================
static bool hubKnown = false;
static uint8_t hubMac[6];

Adafruit_AS7341 as7341;

// ================= UTILIDADES ====================
static void setFixedWiFiChannel(uint8_t ch) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

static bool addPeerIfNeeded(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = WIFI_CH;
  p.encrypt = false;
  return (esp_now_add_peer(&p) == ESP_OK);
}

static uint16_t clampU16(int v) {
  if (v < 0) return 0;
  if (v > 65535) return 65535;
  return (uint16_t)v;
}

// Proxy espectral (ejemplo)
static uint16_t computeHumidityPermille(uint16_t *c) {
  float red   = (float)c[6]; // 630 nm
  float green = (float)c[3]; // 515 nm
  float ratio = red / (green + 1.0f);

  float x = (ratio - 0.5f) / (2.0f - 0.5f);
  int perm = (int)(x * 1000.0f);
  if (perm < 0) perm = 0;
  if (perm > 1000) perm = 1000;
  return (uint16_t)perm;
}

// ================= RX =============================
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data, int len) {
  if (!info || !data || len < 2) return;
  if (data[0] == MSG_ACK && data[1] == NODE_ID) {
    memcpy(hubMac, info->src_addr, 6);
    hubKnown = true;
    addPeerIfNeeded(hubMac);
  }
}

// ================= SETUP =========================
void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  setFixedWiFiChannel(WIFI_CH);

  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  const uint8_t bcast[6] = {255,255,255,255,255,255};
  addPeerIfNeeded(bcast);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!as7341.begin()) {
    Serial.println("AS7341 NOT FOUND");
    while (1) delay(10);
  }

  // Configuración espectral (como ejemplo Adafruit)
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);

  // Configuración LED
  as7341.setLEDCurrent(LED_CURRENT_MA);
  as7341.enableLED(false);

  // HELLO
  hello_msg_t h{MSG_HELLO, NODE_ID};
  esp_now_send(bcast, (uint8_t*)&h, sizeof(h));

  Serial.print("AS7341 NODE READY ID=");
  Serial.println(NODE_ID);
}

// ================= LOOP ==========================
void loop() {
  static uint32_t lastSend = 0;
  uint32_t now = millis();

  if (now - lastSend >= SEND_MS) {
    lastSend = now;

    // === LED ON ===
    as7341.enableLED(true);
    delay(LED_ON_DELAY_MS);

    // === MEDICIÓN ===
    if (!as7341.readAllChannels()) {
      as7341.enableLED(false);
      return;
    }

    uint16_t ch[10];
    ch[0] = as7341.getChannel(AS7341_CHANNEL_415nm_F1);
    ch[1] = as7341.getChannel(AS7341_CHANNEL_445nm_F2);
    ch[2] = as7341.getChannel(AS7341_CHANNEL_480nm_F3);
    ch[3] = as7341.getChannel(AS7341_CHANNEL_515nm_F4);
    ch[4] = as7341.getChannel(AS7341_CHANNEL_555nm_F5);
    ch[5] = as7341.getChannel(AS7341_CHANNEL_590nm_F6);
    ch[6] = as7341.getChannel(AS7341_CHANNEL_630nm_F7);
    ch[7] = as7341.getChannel(AS7341_CHANNEL_680nm_F8);
    ch[8] = as7341.getChannel(AS7341_CHANNEL_CLEAR);
    ch[9] = as7341.getChannel(AS7341_CHANNEL_NIR);

    // === LED OFF ===
    as7341.enableLED(false);

    // === EMPAQUETAR ===
    spec_msg_t s{};
    s.type = MSG_SPEC;
    s.node_id = NODE_ID;
    for (int i = 0; i < 10; i++) s.ch[i] = clampU16(ch[i]);

    hum_msg_t h{};
    h.type = MSG_HUM;
    h.node_id = NODE_ID;
    h.hum_permille = computeHumidityPermille(ch);

    const uint8_t *dst = hubKnown ? hubMac :
      (const uint8_t[6]){255,255,255,255,255,255};

    addPeerIfNeeded(dst);
    esp_now_send(dst, (uint8_t*)&s, sizeof(s));
    esp_now_send(dst, (uint8_t*)&h, sizeof(h));

    Serial.print("N"); Serial.print(NODE_ID);
    Serial.print(" HUM="); Serial.println(h.hum_permille);
  }

  delay(10);
}
