#include <Arduino.h>
#include "LX16A-bus.h"

// UART para el bus del servo (UART1 en el ESP32-C3)
HardwareSerial SerialAX(1);

// Servo con ID 1, usando SerialAX (no Serial0)
LX16A motor(1, SerialAX);

const uint32_t BaudRate = 115200;

// Pines del ESP32-C3 usados para el bus del servo
// OJO: orden en begin = (baud, config, RX, TX)
const int RX_SERVO_PIN = 21;  // ajusta según tu cableado
const int TX_SERVO_PIN = 20;  // ajusta según tu cableado

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Test LX16A-bus: solo TX, movimiento simple (ESP32-C3)");

  // Inicializar UART para el bus LX-16A
  SerialAX.begin(BaudRate, SERIAL_8N1, RX_SERVO_PIN, TX_SERVO_PIN);

  // Solo enviamos comandos, sin esperar respuesta
  motor.initialize();
  motor.enableTorque();
  motor.setServoMode();  // modo servo (posición)
  Serial.println("Comandos de inicialización enviados");
}

void loop() {
  // Mover a 0°
  Serial.println("Mover a 0°");
  motor.move(0);
   delay(3000);
  // Mover a 90°

}
