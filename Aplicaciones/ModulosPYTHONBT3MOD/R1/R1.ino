#include <Arduino.h>

// ===== BLE =====
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ===== AX-12A =====
#include "AX12A.h"

// ======== Configuración del nodo ========
#define NODE_ID 1
// ======== PINES ================
// ADC
#define IN_1 1
#define IN_2 2
#define IN_4 4
// ======== AX-12A / UART ========
#define AX_ID          (1u)
#define AX_BAUD        (115200ul)
#define AX_DIR_PIN     (10u)   

//Valores iniciales del motor
static int axSpeed = 25;      
static const int START_POS = 512;

// ======== UUIDs BLE (deben coincidir con el maestro) ========
static BLEUUID SVC_UUID("12345678-1234-1234-1234-1234567890ab");
static BLEUUID CHR_NOTIFY_UUID("12345678-1234-1234-1234-1234567890ac");
static BLEUUID CHR_WRITE_UUID ("12345678-1234-1234-1234-1234567890ad");

// ===== Payload ADC (binario) =====
typedef struct __attribute__((packed)) {
  uint8_t  node_id;
  uint16_t in1;
  uint16_t in2;
  uint16_t in4;
} adc_payload_t;

BLECharacteristic* chrNotify = nullptr;

// ======== Funciones de conversión a angulo ========
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

static String devName() {
  return String("ROBOT_") + String(NODE_ID);
}

// ======== Parseo de comandos BLE Write ========
// Comandos:
//  HOME
//  ANG <deg>     ej: "ANG 45"
//  POS <0..1023> ej: "POS 700"
//  SPD <0..1023> ej: "SPD 120"
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
    float v = cmd.substring(3).toFloat();
    int s = (int)v;
    if (s < 0) s = 0;
    if (s > 1023) s = 1023;
    axSpeed = s;
    return;
  }

  if (up.startsWith("POS")) {
    float v = cmd.substring(3).toFloat();
    int p = (int)v;
    axMovePos(p);
    return;
  }
}

class WriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    // En BLE clásico, getValue() retorna String
    String v = pCharacteristic->getValue();
    handleCmd(v);
  }
};

void setup() {
  // ===== ADC =====
  analogReadResolution(8);        
  analogSetAttenuation(ADC_11db);  
  // ===== AX-12A =====
  Serial1.begin(AX_BAUD, SERIAL_8N1, 21, 20);
  ax12a.begin(AX_BAUD, AX_DIR_PIN, &Serial1);
  // Posición inicial
  axMovePos(START_POS);
  BLEDevice::init(devName());
  BLEServer* server = BLEDevice::createServer();
  BLEService* svc = server->createService(SVC_UUID);
  // Notify (ADC -> maestro)
  chrNotify = svc->createCharacteristic(
    CHR_NOTIFY_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  chrNotify->addDescriptor(new BLE2902());

  // Write (maestro -> nodo) para comandos de motor
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
  // ===== Crear paquete de envio =====
  adc_payload_t p;
  p.node_id = (uint8_t)NODE_ID;
  p.in1 = (uint16_t)analogRead(IN_1);
  p.in2 = (uint16_t)analogRead(IN_2);
  p.in4 = (uint16_t)analogRead(IN_4);

  chrNotify->setValue((uint8_t*)&p, sizeof(p));
  chrNotify->notify();

  delay(250); 
}
