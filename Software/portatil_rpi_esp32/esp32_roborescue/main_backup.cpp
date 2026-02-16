
#include <Arduino.h>

// ================================================================
//  CONFIGURACIÓN DE PINES
// ================================================================

// DRIVER 1 (TRASERO)
#define RL_PWM 25   // Motor Trasero Izquierda
#define RL_DIR 26  

#define RR_PWM 27   // Motor Trasero Derecha
#define RR_DIR 14   

// DRIVER 2 (DELANTERO)
#define FL_PWM 18   // Motor Frontal Izquierda
#define FL_DIR 19  

#define FR_PWM 32   // Motor Frontal Derecha
#define FR_DIR 33

// Configuración PWM
const int freq = 1000;
const int resolution = 8;
const int ch_RL = 0; 
const int ch_RR = 1;
const int ch_FL = 2;
const int ch_FR = 3;

// ================================================================
//  CONTROLADOR MOTOR
// ================================================================
void set_motor(int channel, int dir_pin, float speed, const char* name) {
  // Limitar
  if (speed > 1.0) speed = 1.0;
  if (speed < -1.0) speed = -1.0;

  int pwm = abs(speed) * 255;
  
  // ZONA MUERTA
  if (pwm < 25) pwm = 0;

  // DIRECCIÓN
  if (speed > 0) {
    digitalWrite(dir_pin, HIGH);
  } else {
    digitalWrite(dir_pin, LOW);
  }
  
  ledcWrite(channel, pwm);
  
  // Debug
  Serial.print(name);
  Serial.print(": speed=");
  Serial.print(speed, 2);
  Serial.print(" pwm=");
  Serial.print(pwm);
  Serial.print(" dir=");
  Serial.println(speed > 0 ? "HIGH" : "LOW");
}

// ================================================================
//  CINEMÁTICA MECANUM
// ================================================================
void test_movement(float x, float y, float z, const char* description) {
  Serial.println("\n========================================");
  Serial.print("TEST: ");
  Serial.println(description);
  Serial.print("Comando: x=");
  Serial.print(x, 2);
  Serial.print(" y=");
  Serial.print(y, 2);
  Serial.print(" z=");
  Serial.println(z, 2);
  Serial.println("----------------------------------------");

  // Cinemática X-Drive
  float fl = x + y + z;
  float fr = x - y - z;
  float rl = x - y + z;
  float rr = x + y - z;

  Serial.println("Velocidades calculadas (antes normalizar):");
  Serial.print("  FL="); Serial.print(fl, 2);
  Serial.print(" FR="); Serial.print(fr, 2);
  Serial.print(" RL="); Serial.print(rl, 2);
  Serial.print(" RR="); Serial.println(rr, 2);

  // Normalizar
  float max_val = max(abs(fl), max(abs(fr), max(abs(rl), abs(rr))));
  if (max_val > 1.0) {
    fl /= max_val; 
    fr /= max_val; 
    rl /= max_val; 
    rr /= max_val;
    Serial.print("Normalizando por: ");
    Serial.println(max_val, 2);
  }

  Serial.println("\nVelocidades finales (después normalizar):");
  Serial.print("  FL="); Serial.print(fl, 2);
  Serial.print(" FR="); Serial.print(fr, 2);
  Serial.print(" RL="); Serial.print(rl, 2);
  Serial.print(" RR="); Serial.println(rr, 2);

  Serial.println("\nAplicando a motores:");
  set_motor(ch_FL, FL_DIR, fl, "FL");
  set_motor(ch_FR, FR_DIR, fr, "FR");
  set_motor(ch_RL, RL_DIR, rl, "RL");
  set_motor(ch_RR, RR_DIR, rr, "RR");
  
  Serial.println("========================================");
  
  // Mantener movimiento 3 segundos
  delay(3000);
  
  // PARAR
  Serial.println("\n⏹️  PARANDO MOTORES\n");
  set_motor(ch_FL, FL_DIR, 0, "FL");
  set_motor(ch_FR, FR_DIR, 0, "FR");
  set_motor(ch_RL, RL_DIR, 0, "RL");
  set_motor(ch_RR, RR_DIR, 0, "RR");
  
  delay(2000); // Pausa entre pruebas
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n========================================");
  Serial.println("   TEST CINEMÁTICA MECANUM");
  Serial.println("========================================");
  Serial.println("Configuración de motores:");
  Serial.println("  FL (Front Left)  = Canal 2, PWM=18, DIR=19");
  Serial.println("  FR (Front Right) = Canal 3, PWM=32, DIR=33");
  Serial.println("  RL (Rear Left)   = Canal 0, PWM=25, DIR=26");
  Serial.println("  RR (Rear Right)  = Canal 1, PWM=27, DIR=14");
  Serial.println("========================================\n");

  // Configurar pines
  pinMode(RL_DIR, OUTPUT); digitalWrite(RL_DIR, LOW);
  pinMode(RR_DIR, OUTPUT); digitalWrite(RR_DIR, LOW);
  pinMode(FL_DIR, OUTPUT); digitalWrite(FL_DIR, LOW);
  pinMode(FR_DIR, OUTPUT); digitalWrite(FR_DIR, LOW);
  
  // Configurar PWM
  ledcSetup(ch_RL, freq, resolution); ledcAttachPin(RL_PWM, ch_RL);
  ledcSetup(ch_RR, freq, resolution); ledcAttachPin(RR_PWM, ch_RR);
  ledcSetup(ch_FL, freq, resolution); ledcAttachPin(FL_PWM, ch_FL);
  ledcSetup(ch_FR, freq, resolution); ledcAttachPin(FR_PWM, ch_FR);

  // Asegurar PWM en 0
  ledcWrite(ch_RL, 0);
  ledcWrite(ch_RR, 0);
  ledcWrite(ch_FL, 0);
  ledcWrite(ch_FR, 0);
  
  Serial.println("✅ Hardware configurado");
  Serial.println("\nIniciando secuencia de pruebas en 3 segundos...");
  delay(3000);
}

void loop() {
  Serial.println("\n\n🔄 INICIANDO CICLO DE PRUEBAS\n");
  
  // PRUEBA 1: Avanzar adelante (solo X positivo)
  test_movement(1.0, 0.0, 0.0, "AVANZAR ADELANTE");
  
  // PRUEBA 2: Retroceder (solo X negativo)
  test_movement(-1.0, 0.0, 0.0, "RETROCEDER");
  
  // PRUEBA 3: Desplazamiento lateral DERECHA (solo Y positivo)
  test_movement(0.0, 1.0, 0.0, "LATERAL DERECHA");
  
  // PRUEBA 4: Desplazamiento lateral IZQUIERDA (solo Y negativo)
  test_movement(0.0, -1.0, 0.0, "LATERAL IZQUIERDA");
  
  //PRUEBA 5: Giro HORARIO (solo Z positivo)
  test_movement(0.0, 0.0, 1.0, "GIRO HORARIO");
  
  // PRUEBA 6: Giro ANTIHORARIO (solo Z negativo)
  test_movement(0.0, 0.0, -1.0, "GIRO ANTIHORARIO");

  Serial.println("\n\n✅ CICLO COMPLETO. Esperando 10 segundos antes de repetir...\n");
  delay(10000);
}
