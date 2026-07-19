/*
 * Example of a simple Blink using the AX-12A built-in LED
 * Adaptado para ESP32 usando HardwareSerial con pines configurables
 */

#include "Arduino.h"
#include "AX12A.h"

#define DirectionPin   (10u)        // Pin de control half-duplex
#define BaudRate       (115200ul)
#define ID             (1u)

// Crear puerto serial hardware
HardwareSerial SerialAX(0);          // UART1 del ESP32

void setup()
{
  // Configurar pines del ESP32 para el bus serial del AX-12A
  SerialAX.begin(BaudRate, SERIAL_8N1, 21, 20);  
  // RX = GPIO16, TX = GPIO17  (ajusta si necesitas otros pines)

  ax12a.begin(BaudRate, DirectionPin, &SerialAX);
}

void loop()
{
  ax12a.ledStatus(ID, ON);
  delay(1000);
  ax12a.ledStatus(ID, OFF);
  delay(1000);
}
