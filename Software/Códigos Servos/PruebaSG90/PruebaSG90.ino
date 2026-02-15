//Instalar librería ESP32Servo by Kevin Harringtong 
#include <ESP32Servo.h>

Servo servo;

void setup() {
  servo.attach(21);   //
}

void loop() {
  servo.write(0);     // Ir a 0 grados
  delay(2000);

  servo.write(180);   // Ir a 180 grados
  delay(2000);
}
