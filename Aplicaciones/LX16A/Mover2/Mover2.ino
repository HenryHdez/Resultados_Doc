#include <Arduino.h>
#include "LX16A-bus.h"

// UART1 en ESP32 (ESP32-C3 soporta reasignación de pines según placa)
HardwareSerial SerialAX(1);

// Servo ID 1 por el bus
LX16A motor(1, SerialAX);

const uint32_t BAUD_SERVO = 115200;

// AJUSTE ESTOS GPIO A LOS QUE REALMENTE EXISTAN EN SU PLACA ESP32-C3
// begin(baud, config, RX, TX)
const int RX_SERVO_PIN = 4;   // ejemplo
const int TX_SERVO_PIN = 5;   // ejemplo

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("ESP32-C3 + LX16A (solo TX) - prueba de posiciones");

  SerialAX.begin(BAUD_SERVO, SERIAL_8N1, RX_SERVO_PIN, TX_SERVO_PIN);
  delay(50);

  motor.initialize();
  motor.enableTorque();
  motor.setServoMode();

  Serial.println("Inicialización enviada. Iniciando movimientos...");
}

void loop() {
  const int poses[] = {0, 60, 120, 180, 240};
  for (int i = 0; i < (int)(sizeof(poses)/sizeof(poses[0])); i++) {
    Serial.print("Mover a ");
    Serial.print(poses[i]);
    Serial.println("°");
    motor.move(poses[i]);
    delay(3000);
  }
}
