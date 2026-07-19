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
	SerialAX.begin(BaudRate, SERIAL_8N1, 21, 20); 
	ax12a.begin(BaudRate, DirectionPin, &SerialAX);
	ax12a.setEndless(ID, ON);
	ax12a.turn(ID, LEFT, 100);
}

void loop()
{

}
