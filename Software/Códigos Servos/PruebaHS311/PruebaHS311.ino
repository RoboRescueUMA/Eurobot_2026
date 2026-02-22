#include <ESP32Servo.h>
Servo servo;
void setup() {
  servo.attach(21);
}

void loop() {
  servo.write(0);     // gira hacia un lado
  delay(2000);

  servo.write(94);    // detener
  delay(2000);

  servo.write(180);   // gira hacia el otro lado
  delay(2000);

  servo.write(94);    // detener
  delay(2000);
}
