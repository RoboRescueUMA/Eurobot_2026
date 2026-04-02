#include <Arduino.h>

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ================================================================
//  ZONA DE CONFIGURACIÓN DEL CHASIS Y CINEMÁTICA
// ================================================================

// Parámetros PID individuales por motor (Sintonizados para overshoot=1.5%, ts=0.52s)
const float Kp_FL = 218.9; const float Ki_FL = 1488.0;
const float Kp_FR = 129.4; const float Ki_FR = 1130.0;
const float Kp_RL = 200.8; const float Ki_RL = 1435.0;
const float Kp_RR = 159.7; const float Ki_RR = 1078.0;

// Compensación de fricción
const int base_FL = 89; const int base_FR = 47;
const int base_RL = 84; const int base_RR = 78;

// Geometría del robot (en metros)
const float LX = 0.130;  // Mitad del ancho (26cm / 2)
const float LY = 0.075;  // Mitad del largo (15cm / 2)
const float K_GEOMETRIA = LX + LY; // 0.205

// Variables globales para las velocidades objetivo calculadas
float ref_FL = 0.0, ref_FR = 0.0, ref_RL = 0.0, ref_RR = 0.0;

// ================================================================
//  PINES Y CONFIGURACIÓN PWM
// ================================================================
#define FL_PWM 13    
#define FL_DIR 25  
#define FR_PWM 33   
#define FR_DIR 32  
#define RL_PWM 18
#define RL_DIR 4
#define RR_PWM 21
#define RR_DIR 19

#define FL_ENC_A 16
#define FL_ENC_B 17
#define FR_ENC_A 26
#define FR_ENC_B 27
#define RL_ENC_A 22
#define RL_ENC_B 23
#define RR_ENC_A 34
#define RR_ENC_B 35

const int freq = 1000;
const int resolution = 8;
const float RADIO_RUEDA = 0.0325; 
const int COUNTS_PER_REV = (11 * 34) * 4; 
const float rads_por_cuenta = (2.0f * PI) / (float)COUNTS_PER_REV;
const unsigned long ventana_us = 40000; // 40ms

// ================================================================
//  VARIABLES DE ESTADO
// ================================================================
volatile long cont_FL = 0, cont_FR = 0, cont_RL = 0, cont_RR = 0;
long prev_FL = 0, prev_FR = 0, prev_RL = 0, prev_RR = 0;
unsigned long lastWindowUs = 0;
unsigned long t_inicio_prueba = 0; 
double errSum_FL = 0, errSum_FR = 0, errSum_RL = 0, errSum_RR = 0;
bool ensayo_finalizado = false;

// ================================================================
//  INTERRUPCIONES
// ================================================================
void IRAM_ATTR isr_FL_A() { (digitalRead(FL_ENC_A) != digitalRead(FL_ENC_B)) ? cont_FL-- : cont_FL++; }
void IRAM_ATTR isr_FR_A() { (digitalRead(FR_ENC_A) != digitalRead(FR_ENC_B)) ? cont_FR-- : cont_FR++; }
void IRAM_ATTR isr_RL_A() { (digitalRead(RL_ENC_A) != digitalRead(RL_ENC_B)) ? cont_RL-- : cont_RL++; }
void IRAM_ATTR isr_RR_A() { (digitalRead(RR_ENC_A) != digitalRead(RR_ENC_B)) ? cont_RR-- : cont_RR++; }
void IRAM_ATTR isr_FL_B() { (digitalRead(FL_ENC_A) == digitalRead(FL_ENC_B)) ? cont_FL-- : cont_FL++; }
void IRAM_ATTR isr_FR_B() { (digitalRead(FR_ENC_A) == digitalRead(FR_ENC_B)) ? cont_FR-- : cont_FR++; }
void IRAM_ATTR isr_RL_B() { (digitalRead(RL_ENC_A) == digitalRead(RL_ENC_B)) ? cont_RL-- : cont_RL++; }
void IRAM_ATTR isr_RR_B() { (digitalRead(RR_ENC_A) == digitalRead(RR_ENC_B)) ? cont_RR-- : cont_RR++; }

// ================================================================
//  FUNCIÓN PID BIDIRECCIONAL (Ahora recibe Kp y Ki individuales)
// ================================================================
int calcular_pwm_rueda(float v_deseada, float v_actual, double &memoria_integral, int pwm_base_motor, float kp, float ki) {
  if (v_deseada == 0.0) { memoria_integral = 0; return 0; }
  double dt = (double)ventana_us / 1000000.0;
  double error = v_deseada - v_actual;
  memoria_integral += (error * dt);

  double limite_integral = (255.0 - pwm_base_motor) / ki; 
  if (memoria_integral > limite_integral) memoria_integral = limite_integral;
  else if (memoria_integral < -limite_integral) memoria_integral = -limite_integral;

  double Output = (kp * error) + (ki * memoria_integral);
  int pwm_final = 0;
  
  if (v_deseada > 0.0) {
    if (Output > 0) pwm_final = pwm_base_motor + (int)Output;
    else pwm_final = 0; 
  } else if (v_deseada < 0.0) {
    if (Output < 0) pwm_final = -(pwm_base_motor + (int)abs(Output));
    else pwm_final = 0; 
  }

  if (pwm_final > 255) pwm_final = 255;
  if (pwm_final < -255) pwm_final = -255;
  return pwm_final;
}

// ================================================================
//  FUNCIÓN DE CINEMÁTICA MECANUM
// ================================================================
void calcular_cinematica(float Vx, float Vy, float W) {
  ref_FL = Vx + Vy - (W * K_GEOMETRIA);
  ref_FR = Vx - Vy + (W * K_GEOMETRIA);
  ref_RL = Vx - Vy - (W * K_GEOMETRIA);
  ref_RR = Vx + Vy + (W * K_GEOMETRIA);
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(3000); 

  pinMode(FL_DIR, OUTPUT); pinMode(RL_DIR, OUTPUT);
  pinMode(FR_DIR, OUTPUT); pinMode(RR_DIR, OUTPUT);
  pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(RL_ENC_A, INPUT_PULLUP); pinMode(RL_ENC_B, INPUT_PULLUP);
  pinMode(RR_ENC_A, INPUT_PULLUP); pinMode(RR_ENC_B, INPUT_PULLUP);

  ledcAttach(FL_PWM, freq, resolution); ledcAttach(FR_PWM, freq, resolution);
  ledcAttach(RL_PWM, freq, resolution); ledcAttach(RR_PWM, freq, resolution);

  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isr_FL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_ENC_B), isr_FL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isr_FR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_B), isr_FR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isr_RL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_B), isr_RL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isr_RR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_B), isr_RR_B, CHANGE);

  lastWindowUs = micros();
  t_inicio_prueba = millis(); 
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  if (ensayo_finalizado) {
    ledcWrite(FL_PWM, 0); ledcWrite(FR_PWM, 0); ledcWrite(RL_PWM, 0); ledcWrite(RR_PWM, 0);
    return; 
  }

  unsigned long ahora_us = micros();

  if ((ahora_us - lastWindowUs) >= ventana_us) {
    unsigned long ventana_real_us = ahora_us - lastWindowUs;
    lastWindowUs = ahora_us;

    unsigned long tiempo = millis() - t_inicio_prueba;
    
    float comando_Vx = 0.0;
    float comando_Vy = 0.0;
    float comando_W  = 0.0;

    if (tiempo < 3000) {
      comando_Vx = 0.1; comando_Vy = 0.0; comando_W = 0.0;
    } 
    else if (tiempo >= 4000 && tiempo < 5570) {
      comando_Vx = 0.0; comando_Vy = 0.0; comando_W = 1.0;
    }
    else if (tiempo >= 6570 && tiempo < 8570) {
      comando_Vx = 0.1; comando_Vy = 0.0; comando_W = 0.0;
    }
    else if (tiempo >= 9570 && tiempo < 11140) {
      comando_Vx = 0.0; comando_Vy = 0.0; comando_W = 1.0;
    }
    else if (tiempo >= 12140 && tiempo < 12640) {
      comando_Vx = 0.0; comando_Vy = 0.25; comando_W = 0.0;
    }
    else if (tiempo > 13500) {
      ensayo_finalizado = true;
    }

    calcular_cinematica(comando_Vx, comando_Vy, comando_W);

    portENTER_CRITICAL(&mux);
    long current_FL = cont_FL; long current_FR = cont_FR;
    long current_RL = cont_RL; long current_RR = cont_RR;
    portEXIT_CRITICAL(&mux);

    long delta_FL = current_FL - prev_FL; long delta_FR = current_FR - prev_FR;
    long delta_RL = current_RL - prev_RL; long delta_RR = current_RR - prev_RR;

    prev_FL = current_FL; prev_FR = current_FR;
    prev_RL = current_RL; prev_RR = current_RR;

    float rads_FL = -(((float)delta_FL * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;
    float rads_FR = (((float)delta_FR * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;
    float rads_RL = -(((float)delta_RL * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;
    float rads_RR = (((float)delta_RR * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;

    float v_FL = rads_FL * RADIO_RUEDA; float v_FR = rads_FR * RADIO_RUEDA;
    float v_RL = rads_RL * RADIO_RUEDA; float v_RR = rads_RR * RADIO_RUEDA;

    // APLICAR PID CON PARÁMETROS INDIVIDUALES
    int pwm_FL = calcular_pwm_rueda(ref_FL, v_FL, errSum_FL, base_FL, Kp_FL, Ki_FL);
    int pwm_FR = calcular_pwm_rueda(ref_FR, v_FR, errSum_FR, base_FR, Kp_FR, Ki_FR);
    int pwm_RL = calcular_pwm_rueda(ref_RL, v_RL, errSum_RL, base_RL, Kp_RL, Ki_RL);
    int pwm_RR = calcular_pwm_rueda(ref_RR, v_RR, errSum_RR, base_RR, Kp_RR, Ki_RR);

    digitalWrite(FL_DIR, (pwm_FL >= 0) ? LOW : HIGH); ledcWrite(FL_PWM, abs(pwm_FL));
    digitalWrite(FR_DIR, (pwm_FR >= 0) ? HIGH : LOW); ledcWrite(FR_PWM, abs(pwm_FR));
    digitalWrite(RL_DIR, (pwm_RL >= 0) ? LOW : HIGH); ledcWrite(RL_PWM, abs(pwm_RL));
    digitalWrite(RR_DIR, (pwm_RR >= 0) ? HIGH : LOW); ledcWrite(RR_PWM, abs(pwm_RR));
  }
}