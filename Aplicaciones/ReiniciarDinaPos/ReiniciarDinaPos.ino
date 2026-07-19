#include "Arduino.h"
#include "AX12A.h"

#define DirectionPin   (10u)
#define BaudRate       (115200ul)
#define ID             (1u)

HardwareSerial SerialAX(0);  // como su ejemplo que funciona

// --- Utilidades Dynamixel Protocol 1.0 (AX-12A) ---
// Instrucciones
const uint8_t INST_WRITE = 0x03;

// Registros
const uint8_t CW_ANGLE_LIMIT_L  = 6;
const uint8_t CCW_ANGLE_LIMIT_L = 8;

// Control half-duplex: 1 = TX, 0 = RX (ajuste si su driver invierte)
void dxlSetTx(bool tx)
{
  digitalWrite(DirectionPin, tx ? HIGH : LOW);
  delayMicroseconds(10);
}

// Enviar paquete Dynamixel (sin leer respuesta)
void dxlWriteBytes(uint8_t id, uint8_t address, const uint8_t *data, uint8_t dataLen)
{
  // Longitud = parámetros (address + dataLen) + 2 (INST + CHKSUM)
  uint8_t length = (uint8_t)(dataLen + 3);

  uint8_t checksum = 0;
  checksum += id;
  checksum += length;
  checksum += INST_WRITE;
  checksum += address;
  for (uint8_t i = 0; i < dataLen; i++) checksum += data[i];
  checksum = ~checksum;

  dxlSetTx(true);

  SerialAX.write(0xFF);
  SerialAX.write(0xFF);
  SerialAX.write(id);
  SerialAX.write(length);
  SerialAX.write(INST_WRITE);
  SerialAX.write(address);
  for (uint8_t i = 0; i < dataLen; i++) SerialAX.write(data[i]);
  SerialAX.write(checksum);
  SerialAX.flush();

  dxlSetTx(false);
  delay(10);
}

// Escribir palabra (2 bytes, little-endian) en registro
void dxlWriteWord(uint8_t id, uint8_t address, uint16_t value)
{
  uint8_t data[2];
  data[0] = (uint8_t)(value & 0xFF);       // L
  data[1] = (uint8_t)((value >> 8) & 0xFF);// H
  dxlWriteBytes(id, address, data, 2);
}

// Forzar modo articulación (posición): CW=0, CCW=1023
void forceJointMode(uint8_t id)
{
  dxlWriteWord(id, CW_ANGLE_LIMIT_L, 0);       // CW = 0
  dxlWriteWord(id, CCW_ANGLE_LIMIT_L, 1023);   // CCW = 1023
}

// (Opcional) Forzar modo rueda: CW=0, CCW=0
void forceWheelMode(uint8_t id)
{
  dxlWriteWord(id, CW_ANGLE_LIMIT_L, 0);
  dxlWriteWord(id, CCW_ANGLE_LIMIT_L, 0);
}

void setup()
{
  pinMode(DirectionPin, OUTPUT);
  dxlSetTx(false);

  // UART y librería (para usar move() después)
  SerialAX.begin(BaudRate, SERIAL_8N1, 21, 20);
  ax12a.begin(BaudRate, DirectionPin, &SerialAX);

  // Confirmación de enlace: LED interno
  ax12a.ledStatus(ID, ON);  delay(200);
  ax12a.ledStatus(ID, OFF); delay(200);

  // 1) Forzar modo posición escribiendo registros (sin depender de writeData)
  forceJointMode(ID);
  delay(200);

  // 2) Test de movimiento grande (debe notarse)
  ax12a.move(ID, 512); delay(1200);
  ax12a.move(ID, 712); delay(1200);
  ax12a.move(ID, 312); delay(1200);
  ax12a.move(ID, 512); delay(1200);
}

void loop()
{
  // Repetir patrón
  ax12a.move(ID, 712); delay(1200);
  ax12a.move(ID, 312); delay(1200);
  ax12a.move(ID, 512); delay(1200);
}
