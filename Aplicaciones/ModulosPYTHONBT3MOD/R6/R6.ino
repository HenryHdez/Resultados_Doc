#include <Arduino.h>

// ===== BLE =====
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ===== LX16A =====
#include "LX16A-bus.h"

// ====== Configuración del nodo ======
#define NODE_ID 6

// ====== ADC ======
#define IN_1 1       // GPIO1  (ADC)
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

// ====== BLE UUIDs ======
static BLEUUID SVC_UUID("12345678-1234-1234-1234-1234567890ab");
static BLEUUID CHR_NOTIFY_UUID("12345678-1234-1234-1234-1234567890ac");
static BLEUUID CHR_WRITE_UUID ("12345678-1234-1234-1234-1234567890ad");

// ====== Payload binario ADC ======
typedef struct __attribute__((packed)) {
  uint8_t  node_id;
  uint16_t in1;
  uint16_t in2;
  uint16_t in3;
} adc_payload_t;

BLECharacteristic* chrNotify = nullptr;

// ====== Estado ======
static bool bleConnected = false;

static String devName() {
  return String("ROBOT_") + String(NODE_ID);
}

static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ====== Callbacks conexión BLE ======
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    (void)pServer;
    bleConnected = true;
  }
  void onDisconnect(BLEServer* pServer) override {
    (void)pServer;
    bleConnected = false;
    // Reanuncia para reconexión
    BLEDevice::startAdvertising();
  }
};

// ====== Parseo comandos BLE Write ======
// Comandos esperados:
//   LX <0..240>     ej: "LX 120"
//   TORQ <0|1>      ej: "TORQ 1"
//   PING            ej: "PING"
static void handleCmd(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String up = cmd;
  up.toUpperCase();

  // PING
  if (up == "PING") {
    if (chrNotify) {
      const char* resp = "PONG";
      chrNotify->setValue((uint8_t*)resp, strlen(resp));
      chrNotify->notify();
    }
    return;
  }

  // TORQ 0/1
  if (up.startsWith("TORQ")) {
    int v = cmd.substring(4).toInt();
    if (v == 0) motor.disableTorque();
    else motor.enableTorque();
    return;
  }

  // LX <angulo>
  if (up.startsWith("LX")) {
    int ang = cmd.substring(2).toInt();
    ang = clampInt(ang, 0, 240);
    motor.move(ang);

    if (chrNotify) {
      char buf[24];
      snprintf(buf, sizeof(buf), "LX_OK %d", ang);
      chrNotify->setValue((uint8_t*)buf, strlen(buf));
      chrNotify->notify();
    }
    return;
  }

}

class WriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    // BLE clásico Arduino: getValue() retorna String
    String v = pCharacteristic->getValue();
    handleCmd(v);
  }
};

void setup() {
  // ===== ADC recomendado =====
  analogReadResolution(8);          
  analogSetAttenuation(ADC_11db);

  // ===== UART Servo =====
  SerialServo.begin(BAUD_SERVO, SERIAL_8N1, RX_SERVO_PIN, TX_SERVO_PIN);
  delay(50);

  motor.initialize();
  motor.enableTorque();
  motor.setServoMode();
  motor.move(10);

  // ===== BLE init =====
  BLEDevice::init(devName());
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService* svc = server->createService(SVC_UUID);
  chrNotify = svc->createCharacteristic(
    CHR_NOTIFY_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  chrNotify->addDescriptor(new BLE2902());
  BLECharacteristic* chrWrite = svc->createCharacteristic(
    CHR_WRITE_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  chrWrite->setCallbacks(new WriteCallbacks());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  adv->start();
}

void loop() {
  // ===== Lectura ADC =====
  adc_payload_t p;
  p.node_id = (uint8_t)NODE_ID;
  p.in1 = (uint16_t)analogRead(IN_1);
  p.in2 = (uint16_t)analogRead(IN_2);
  p.in3 = (uint16_t)analogRead(IN_3);

  // ===== Envío por Notify =====
  if (bleConnected && chrNotify) {
    chrNotify->setValue((uint8_t*)&p, sizeof(p));
    chrNotify->notify();
  }

  delay(250); // 10 Hz 
}
