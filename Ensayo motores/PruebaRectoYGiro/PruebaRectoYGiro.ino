#include <Arduino.h>

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ================================================================
//  ZONA DE CONFIGURACIÓN DEL CHASIS Y CINEMÁTICA
// ================================================================
float Kp = 159.7; 
float Ki = 1078;

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

#define FL_ENC_A 16; #define FL_ENC_B 17
#define FR_ENC_A 26; #define FR_ENC_B 27
#define RL_ENC_A 22; #define RL_ENC_B 23
#define RR_ENC_A 34; #define RR_ENC_B 35

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
//  INTERRUPCIONES (Igual que siempre)
// ================================================================
void IRAM_ATTR isr_FL_A() { (digitalRead(16) != digitalRead(17)) ? cont_FL-- : cont_FL++; }
void IRAM_ATTR isr_FR_A() { (digitalRead(26) != digitalRead(27)) ? cont_FR-- : cont_FR++; }
void IRAM_ATTR isr_RL_A() { (digitalRead(22) != digitalRead(23)) ? cont_RL-- : cont_RL++; }
void IRAM_ATTR isr_RR_A() { (digitalRead(34) != digitalRead(35)) ? cont_RR-- : cont_RR++; }
void IRAM_ATTR isr_FL_B() { (digitalRead(16) == digitalRead(17)) ? cont_FL-- : cont_FL++; }
void IRAM_ATTR isr_FR_B() { (digitalRead(26) == digitalRead(27)) ? cont_FR-- : cont_FR++; }
void IRAM_ATTR isr_RL_B() { (digitalRead(22) == digitalRead(23)) ? cont_RL-- : cont_RL++; }
void IRAM_ATTR isr_RR_B() { (digitalRead(34) == digitalRead(35)) ? cont_RR-- : cont_RR++; }

// ================================================================
//  FUNCIÓN PID BIDIRECCIONAL 
// ================================================================
int calcular_pwm_rueda(float v_deseada, float v_actual, double &memoria_integral, int pwm_base_motor) {
  if (v_deseada == 0.0) { memoria_integral = 0; return 0; }
  double dt = (double)ventana_us / 1000000.0;
  double error = v_deseada - v_actual;
  memoria_integral += (error * dt);

  double limite_integral = (255.0 - pwm_base_motor) / Ki; 
  if (memoria_integral > limite_integral) memoria_integral = limite_integral;
  else if (memoria_integral < -limite_integral) memoria_integral = -limite_integral;

  double Output = (Kp * error) + (Ki * memoria_integral);
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
//  NUEVO: FUNCIÓN DE CINEMÁTICA MECANUM
// ================================================================
void calcular_cinematica(float Vx, float Vy, float W) {
  // Ecuaciones universales de la rueda Mecanum
  // Vx = Adelante(+)/Atras(-) | Vy = Derecha(+)/Izquierda(-) | W = Giro Antihorario(+)/Horario(-)
  
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
  pinMode(16, INPUT_PULLUP); pinMode(17, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP); pinMode(27, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP); pinMode(23, INPUT_PULLUP);
  pinMode(34, INPUT_PULLUP); pinMode(35, INPUT_PULLUP);

  ledcAttach(FL_PWM, freq, resolution); ledcAttach(FR_PWM, freq, resolution);
  ledcAttach(RL_PWM, freq, resolution); ledcAttach(RR_PWM, freq, resolution);

  attachInterrupt(digitalPinToInterrupt(16), isr_FL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(17), isr_FL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(26), isr_FR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(27), isr_FR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(22), isr_RL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(23), isr_RL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(34), isr_RR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(35), isr_RR_B, CHANGE);

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

    // 1. EL CEREBRO DE NAVEGACIÓN (Secuencia: Avance 0.1 -> Giro 90 -> Avance -> Giro 90 -> Lateral)
    unsigned long tiempo = millis() - t_inicio_prueba;
    
    float comando_Vx = 0.0;
    float comando_Vy = 0.0;
    float comando_W  = 0.0;

    // --- SECUENCIA DE TIEMPOS (Ajustada con pausas de 1s para estabilidad) ---
    
    if (tiempo < 3000) {
      // 1. Avance lento (3 seg) a 0.1 m/s
      comando_Vx = 0.1; comando_Vy = 0.0; comando_W = 0.0;
    } 
    else if (tiempo >= 4000 && tiempo < 5570) {
      // 2. Primer Giro 90º (1.57 seg) a 1.0 rad/s
      comando_Vx = 0.0; comando_Vy = 0.0; comando_W = 1.0;
    }
    else if (tiempo >= 6570 && tiempo < 8570) {
      // 3. Avance corto (2 seg) a 0.1 m/s
      comando_Vx = 0.1; comando_Vy = 0.0; comando_W = 0.0;
    }
    else if (tiempo >= 9570 && tiempo < 11140) {
      // 4. Segundo Giro 90º (1.57 seg) a 1.0 rad/s
      comando_Vx = 0.0; comando_Vy = 0.0; comando_W = 1.0;
    }
    else if (tiempo >= 12140 && tiempo < 12640) {
      // 5. Toque lateral de precisión (0.5 seg) a 0.25 m/s (derecha)
      comando_Vx = 0.0; comando_Vy = 0.25; comando_W = 0.0;
    }
    else if (tiempo > 13500) {
      ensayo_finalizado = true;
    }

    // Le pasamos las órdenes a la Cinemática y ella calcula las referencias (ref_FL, ref_FR...)
    calcular_cinematica(comando_Vx, comando_Vy, comando_W);

    // 2. LEER ENCODERS
    portENTER_CRITICAL(&mux);
    long current_FL = cont_FL; long current_FR = cont_FR;
    long current_RL = cont_RL; long current_RR = cont_RR;
    portEXIT_CRITICAL(&mux);

    long delta_FL = current_FL - prev_FL; long delta_FR = current_FR - prev_FR;
    long delta_RL = current_RL - prev_RL; long delta_RR = current_RR - prev_RR;

    prev_FL = current_FL; prev_FR = current_FR;
    prev_RL = current_RL; prev_RR = current_RR;

    // 3. CALCULAR VELOCIDADES
    float rads_FL = -(((float)delta_FL * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;
    float rads_FR = (((float)delta_FR * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;
    float rads_RL = -(((float)delta_RL * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;
    float rads_RR = (((float)delta_RR * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;

    float v_FL = rads_FL * RADIO_RUEDA; float v_FR = rads_FR * RADIO_RUEDA;
    float v_RL = rads_RL * RADIO_RUEDA; float v_RR = rads_RR * RADIO_RUEDA;

    // 4. APLICAR PID
    int pwm_FL = calcular_pwm_rueda(ref_FL, v_FL, errSum_FL, base_FL);
    int pwm_FR = calcular_pwm_rueda(ref_FR, v_FR, errSum_FR, base_FR);
    int pwm_RL = calcular_pwm_rueda(ref_RL, v_RL, errSum_RL, base_RL);
    int pwm_RR = calcular_pwm_rueda(ref_RR, v_RR, errSum_RR, base_RR);

    // 5. ESCRIBIR PWM
    digitalWrite(FL_DIR, (pwm_FL >= 0) ? LOW : HIGH); ledcWrite(FL_PWM, abs(pwm_FL));
    digitalWrite(FR_DIR, (pwm_FR >= 0) ? HIGH : LOW); ledcWrite(FR_PWM, abs(pwm_FR));
    digitalWrite(RL_DIR, (pwm_RL >= 0) ? LOW : HIGH); ledcWrite(RL_PWM, abs(pwm_RL));
    digitalWrite(RR_DIR, (pwm_RR >= 0) ? HIGH : LOW); ledcWrite(RR_PWM, abs(pwm_RR));
  }
}