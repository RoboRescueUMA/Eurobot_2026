#include <Arduino.h>

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ================================================================
//  CONFIGURACIÓN DE PINES 
// ================================================================
#define FL_PWM 13    
#define FL_DIR 25  
#define FL_ENC_A 16
#define FL_ENC_B 17

#define FR_PWM 33   
#define FR_DIR 32  
#define FR_ENC_A 26
#define FR_ENC_B 27

#define RL_PWM 18
#define RL_DIR 4
#define RL_ENC_A 22
#define RL_ENC_B 23

#define RR_PWM 21
#define RR_DIR 19
#define RR_ENC_A 34
#define RR_ENC_B 35

// Configuración PWM
const int freq = 1000;
const int resolution = 8;

// ================================================================
//  CONSTANTES DEL ROBOT Y MOTORES
// ================================================================
const float RADIO_RUEDA = 0.0325; 
const int PPR_MOTOR = 11;          
const int GEAR_RATIO = 34;         

// Resolución 4X confirmada
const int COUNTS_PER_REV = (PPR_MOTOR * GEAR_RATIO) * 4; 
const float rads_por_cuenta = (2.0f * PI) / (float)COUNTS_PER_REV;

// ================================================================
//  CONFIGURACIÓN DEL ENSAYO
// ================================================================
enum Motores { M_FL, M_FR, M_RL, M_RR };

// ---> CAMBIA ESTA VARIABLE PARA ELEGIR QUÉ MOTOR ENSAYAR <---
const Motores MOTOR_A_PROBAR = M_RR; 

const int PWM_ESCALON = 150;     
const unsigned long ventana_us = 40000;  //Tiempo de muestreo

// ================================================================
//  VARIABLES DE ESTADO Y TIEMPO
// ================================================================
volatile long cont_FL = 0, cont_FR = 0, cont_RL = 0, cont_RR = 0;
long prev_FL = 0, prev_FR = 0, prev_RL = 0, prev_RR = 0;

unsigned long lastWindowUs = 0;
unsigned long tiempoInicioEnsayo = 0;

int pwm_actual = 0;
int pwm_base = 0;
bool ensayo_terminado = false;

// ================================================================
//  INTERRUPCIONES (Resolución 4X Real Corregida)
// ================================================================
// ================================================================
//  INTERRUPCIONES (Resolución 4X Real Corregida e Invertida)
// ================================================================
// Canal A: ahora resta cuando son diferentes y suma cuando son iguales
void IRAM_ATTR isr_FL_A() { (digitalRead(FL_ENC_A) != digitalRead(FL_ENC_B)) ? cont_FL-- : cont_FL++; }
void IRAM_ATTR isr_FR_A() { (digitalRead(FR_ENC_A) != digitalRead(FR_ENC_B)) ? cont_FR-- : cont_FR++; }
void IRAM_ATTR isr_RL_A() { (digitalRead(RL_ENC_A) != digitalRead(RL_ENC_B)) ? cont_RL-- : cont_RL++; }
void IRAM_ATTR isr_RR_A() { (digitalRead(RR_ENC_A) != digitalRead(RR_ENC_B)) ? cont_RR-- : cont_RR++; }

// Canal B: ahora resta cuando son iguales y suma cuando son diferentes
void IRAM_ATTR isr_FL_B() { (digitalRead(FL_ENC_A) == digitalRead(FL_ENC_B)) ? cont_FL-- : cont_FL++; }
void IRAM_ATTR isr_FR_B() { (digitalRead(FR_ENC_A) == digitalRead(FR_ENC_B)) ? cont_FR-- : cont_FR++; }
void IRAM_ATTR isr_RL_B() { (digitalRead(RL_ENC_A) == digitalRead(RL_ENC_B)) ? cont_RL-- : cont_RL++; }
void IRAM_ATTR isr_RR_B() { (digitalRead(RR_ENC_A) == digitalRead(RR_ENC_B)) ? cont_RR-- : cont_RR++; }

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(2000); 

  switch (MOTOR_A_PROBAR) {
    case M_FL: pwm_base = 89; break;
    case M_FR: pwm_base = 47; break;
    case M_RL: pwm_base = 84; break;
    case M_RR: pwm_base = 78; break;
  }
  
  Serial.println("Tiempo(s),PWM,rad/s,m/s");

  pinMode(FL_DIR, OUTPUT); digitalWrite(FL_DIR, LOW);  
  pinMode(RL_DIR, OUTPUT); digitalWrite(RL_DIR, LOW);  
  pinMode(FR_DIR, OUTPUT); digitalWrite(FR_DIR, HIGH); 
  pinMode(RR_DIR, OUTPUT); digitalWrite(RR_DIR, HIGH); 

  ledcAttach(FL_PWM, freq, resolution); ledcWrite(FL_PWM, 0);
  ledcAttach(FR_PWM, freq, resolution); ledcWrite(FR_PWM, 0);
  ledcAttach(RL_PWM, freq, resolution); ledcWrite(RL_PWM, 0);
  ledcAttach(RR_PWM, freq, resolution); ledcWrite(RR_PWM, 0);

  pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(RL_ENC_A, INPUT_PULLUP); pinMode(RL_ENC_B, INPUT_PULLUP);
  pinMode(RR_ENC_A, INPUT_PULLUP); pinMode(RR_ENC_B, INPUT_PULLUP);

  // Asignamos las interrupciones separadas
  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isr_FL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_ENC_B), isr_FL_B, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isr_FR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_B), isr_FR_B, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isr_RL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_B), isr_RL_B, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isr_RR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_B), isr_RR_B, CHANGE);

  delay(1000);
  tiempoInicioEnsayo = millis();
  lastWindowUs = micros();
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  if (ensayo_terminado) return;

  unsigned long ahora_us = micros();
  unsigned long ahora_ms = millis();
  unsigned long tiempo_transcurrido = ahora_ms - tiempoInicioEnsayo;

  if ((ahora_us - lastWindowUs) >= ventana_us) {
    
    if (tiempo_transcurrido < 3000) {
      pwm_actual = pwm_base;
    } else if (tiempo_transcurrido < 5000) {
      pwm_actual = PWM_ESCALON;
    } else {
      pwm_actual = 0;
      ensayo_terminado = true;
    }

    ledcWrite(FL_PWM, (MOTOR_A_PROBAR == M_FL) ? pwm_actual : 0);
    ledcWrite(FR_PWM, (MOTOR_A_PROBAR == M_FR) ? pwm_actual : 0);
    ledcWrite(RL_PWM, (MOTOR_A_PROBAR == M_RL) ? pwm_actual : 0);
    ledcWrite(RR_PWM, (MOTOR_A_PROBAR == M_RR) ? pwm_actual : 0);

    portENTER_CRITICAL(&mux);
    long current_FL = cont_FL;
    long current_FR = cont_FR;
    long current_RL = cont_RL;
    long current_RR = cont_RR;
    portEXIT_CRITICAL(&mux);

    unsigned long ventana_real_us = ahora_us - lastWindowUs;
    if (ventana_real_us == 0) ventana_real_us = 1;

    long delta_FL = current_FL - prev_FL;
    long delta_FR = current_FR - prev_FR;
    long delta_RL = current_RL - prev_RL;
    long delta_RR = current_RR - prev_RR;

    prev_FL = current_FL;
    prev_FR = current_FR;
    prev_RL = current_RL;
    prev_RR = current_RR;

    float cps_FL = ((float)delta_FL * 1000000.0f) / (float)ventana_real_us;
    float cps_FR = ((float)delta_FR * 1000000.0f) / (float)ventana_real_us;
    float cps_RL = ((float)delta_RL * 1000000.0f) / (float)ventana_real_us;
    float cps_RR = ((float)delta_RR * 1000000.0f) / (float)ventana_real_us;

    float rads_FL = -(cps_FL * rads_por_cuenta);
    float rads_FR = cps_FR * rads_por_cuenta;
    float rads_RL = -(cps_RL * rads_por_cuenta);
    float rads_RR = cps_RR * rads_por_cuenta;

    float rads_actual = 0.0;
    switch (MOTOR_A_PROBAR) {
      case M_FL: rads_actual = rads_FL; break;
      case M_FR: rads_actual = rads_FR; break;
      case M_RL: rads_actual = rads_RL; break;
      case M_RR: rads_actual = rads_RR; break;
    }

    float m_s_actual = rads_actual * RADIO_RUEDA;

    float tiempo_s = tiempo_transcurrido / 1000.0;
    
    Serial.print(tiempo_s, 3); Serial.print(" ");
    Serial.print(pwm_actual);  Serial.print(" ");
    Serial.print(rads_actual, 3); Serial.print(" ");
    Serial.println(m_s_actual, 3);

    lastWindowUs = ahora_us;
  }
}