#include "Arduino.h"
#include "AX12A.h"

#define DirectionPin (10u)
#define BaudRate     (115200ul)
#define ID           (1u)

// Entradas NORMALES (HIGH al pulsar)
#define IN_1 1
#define IN_2 2
#define IN_4 4

// Salida
#define LED_PIN 8

HardwareSerial SerialAX(0);

// Parámetros de movimiento
const int CENTER = 512;     // Origen
const int LIMITE = 200;     
const int SPEED  = 25;      // Lento
const unsigned long STEP_MS = 300;

unsigned long lastStep = 0;
int offset = 0;
int dir = +50;
bool regresando = false;

void setup()
{
  // Entradas normales (usar pull-down externo)
  pinMode(IN_1, INPUT);
  pinMode(IN_2, INPUT);
  pinMode(IN_4, INPUT);

  // Salida
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Dynamixel (configuración validada)
  SerialAX.begin(BaudRate, SERIAL_8N1, 21, 20);
  ax12a.begin(BaudRate, DirectionPin, &SerialAX);

  ax12a.moveSpeed(ID, CENTER, SPEED);
  ax12a.ledStatus(ID, OFF);
}

void loop()
{
  /* -------------------------------------------------
     1) LECTURA DE ENTRADAS → GPIO 8
     ------------------------------------------------- */
  bool in1 = digitalRead(IN_1);
  bool in2 = digitalRead(IN_2);
  bool in4 = digitalRead(IN_4);

  if (in1 || in2 || in4)
    digitalWrite(LED_PIN, HIGH);
  else
    digitalWrite(LED_PIN, LOW);

  /* -------------------------------------------------
     2) MOVIMIENTO LENTO AX-12
     ------------------------------------------------- */
  unsigned long now = millis();
  if (now - lastStep >= STEP_MS)
  {
    lastStep = now;

    // 0 → +LIMITE → 0 → −LIMITE → 0
    if (!regresando)
    {
      offset += dir;
      if (offset >= LIMITE || offset <= -LIMITE)
        regresando = true;
    }
    else
    {
      offset -= dir;
      if (offset == 0)
      {
        regresando = false;
        dir = -dir;
      }
    }

    int goal = CENTER + offset;
    goal = constrain(goal, 0, 1023);

    /* -------------------------------------------------
       3) LED DEL AX-12 POR SENTIDO
       ------------------------------------------------- */
    if (dir > 0)
      ax12a.ledStatus(ID, ON);   // sentido positivo
    else
      ax12a.ledStatus(ID, OFF);  // sentido negativo

    ax12a.moveSpeed(ID, goal, SPEED);
  }
}
