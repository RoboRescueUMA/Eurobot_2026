#include <Arduino.h>

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ================================================================
//  ZONA DE CONFIGURACIÓN
// ================================================================
enum Motores { M_FL, M_FR, M_RL, M_RR };
const Motores MOTOR_A_PROBAR = M_FL; 

float Kp = 159.7; 
float Ki = 1078;

// ================================================================
//  PINES Y CONFIGURACIÓN PWM
// ================================================
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

// ================================================================
//  CONSTANTES
// ================================================================
const float RADIO_RUEDA = 0.0325; 
const int PPR_MOTOR = 11;          
const int GEAR_RATIO = 34;         
const int COUNTS_PER_REV = (PPR_MOTOR * GEAR_RATIO) * 4; 
const float rads_por_cuenta = (2.0f * PI) / (float)COUNTS_PER_REV;

const unsigned long ventana_us = 40000; 

// ================================================================
//  VARIABLES DE ESTADO
// ================================================================
volatile long cont_FL = 0, cont_FR = 0, cont_RL = 0, cont_RR = 0;
long prev_FL = 0, prev_FR = 0, prev_RL = 0, prev_RR = 0;

unsigned long lastWindowUs = 0;
unsigned long t_inicio_prueba = 0; // Para medir el tiempo desde que empieza el PID
int pwm_base = 0;

double errSum = 0;
double lastErr = 0;
float vel_deseada_ms = 0.0;

bool ensayo_finalizado = false; // <--- CONDICIÓN DE PARADA

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

void setup() {
  Serial.begin(115200);
  delay(2000); 

  switch (MOTOR_A_PROBAR) {
    case M_FL: pwm_base = 89; break;
    case M_FR: pwm_base = 47; break;
    case M_RL: pwm_base = 84; break;
    case M_RR: pwm_base = 78; break;
  }
  
  pinMode(FL_DIR, OUTPUT); pinMode(RL_DIR, OUTPUT);
  pinMode(FR_DIR, OUTPUT); pinMode(RR_DIR, OUTPUT);

  ledcAttach(FL_PWM, freq, resolution);
  ledcAttach(FR_PWM, freq, resolution);
  ledcAttach(RL_PWM, freq, resolution);
  ledcAttach(RR_PWM, freq, resolution);

  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isr_FL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_ENC_B), isr_FL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isr_FR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_B), isr_FR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isr_RL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_B), isr_RL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isr_RR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_B), isr_RR_B, CHANGE);

  lastWindowUs = micros();
  t_inicio_prueba = millis(); // Guardamos el momento exacto en que empieza el control
}

void loop() {
  // Si el ensayo ha terminado, forzamos motores a cero y salimos del loop
  if (ensayo_finalizado) {
    ledcWrite(FL_PWM, 0); ledcWrite(FR_PWM, 0);
    ledcWrite(RL_PWM, 0); ledcWrite(RR_PWM, 0);
    return; 
  }

  unsigned long ahora_us = micros();

  if ((ahora_us - lastWindowUs) >= ventana_us) {
    unsigned long ventana_real_us = ahora_us - lastWindowUs;
    lastWindowUs = ahora_us;

    // 1. GENERAR PERFIL DE VELOCIDADES (Una sola ejecución)
    unsigned long tiempo_desde_inicio = millis() - t_inicio_prueba;
    int fase = tiempo_desde_inicio / 3000; 

    switch (fase) {
      case 0: vel_deseada_ms = 0.0;  break; 
      case 1: vel_deseada_ms = 0.15;  break; 
      case 2: vel_deseada_ms = 0.15;  break; 
      case 3: vel_deseada_ms = 0.15;  break; 
      case 4: vel_deseada_ms = 0.15; break; 
      case 5: vel_deseada_ms = 0.0;  break; 
      default: 
        ensayo_finalizado = true; // <--- SE ACTIVA AL PASAR LOS 18 SEGUNDOS
        vel_deseada_ms = 0.0;
        break;
    }

    // 2. LEER ENCODERS
    portENTER_CRITICAL(&mux);
    long current_FL = cont_FL; long current_FR = cont_FR;
    long current_RL = cont_RL; long current_RR = cont_RR;
    portEXIT_CRITICAL(&mux);

    long delta_FL = current_FL - prev_FL;
    long delta_FR = current_FR - prev_FR;
    long delta_RL = current_RL - prev_RL;
    long delta_RR = current_RR - prev_RR;

    prev_FL = current_FL; prev_FR = current_FR;
    prev_RL = current_RL; prev_RR = current_RR;

    float cps_FL = ((float)delta_FL * 1000000.0f) / (float)ventana_real_us;
    float cps_FR = ((float)delta_FR * 1000000.0f) / (float)ventana_real_us;
    float cps_RL = ((float)delta_RL * 1000000.0f) / (float)ventana_real_us;
    float cps_RR = ((float)delta_RR * 1000000.0f) / (float)ventana_real_us;

    // Selección dinámica del motor para calcular la velocidad real
    float rads_actual = 0.0;
    switch (MOTOR_A_PROBAR) {
      case M_FL: rads_actual = -(cps_FL * rads_por_cuenta); break;
      case M_FR: rads_actual = cps_FR * rads_por_cuenta; break;
      case M_RL: rads_actual = -(cps_RL * rads_por_cuenta); break;
      case M_RR: rads_actual = cps_RR * rads_por_cuenta; break;
    }
    
    float m_s_actual = rads_actual * RADIO_RUEDA;

    // 3. PID
    double timeChange_s = (double)ventana_real_us / 1000000.0; 
    double error = vel_deseada_ms - m_s_actual;
    
    if (vel_deseada_ms == 0.0 && fase != 0) { // Si es la fase de parada final
       errSum = 0;
    } else {
       errSum += (error * timeChange_s);
    }

    double limite_integral = (255.0 - pwm_base) / Ki; 
    if (errSum > limite_integral) errSum = limite_integral;
    else if (errSum < -limite_integral) errSum = -limite_integral;

    double Output = (Kp * error) + (Ki * errSum);
    lastErr = error;

    // 4. SALIDA MOTORES (Con prohibición de marcha atrás en movimiento)
    int pwm_final = 0;
    bool adelante = true;

    if (vel_deseada_ms == 0.0) {
      pwm_final = 0;
      errSum = 0; 
    } 
    else if (vel_deseada_ms > 0.0) { // Si quiero ir hacia ADELANTE
      if (Output > 0) {
        pwm_final = pwm_base + (int)Output;
        adelante = true;
      } else {
        // Me pasé de velocidad. En vez de frenar con marcha atrás, apago el motor.
        pwm_final = 0; 
        adelante = true; 
      }
    }
    else if (vel_deseada_ms < 0.0) { // Si quiero ir hacia ATRÁS
      if (Output < 0) {
        pwm_final = pwm_base + (int)abs(Output);
        adelante = false;
      } else {
        // Me pasé de velocidad yendo hacia atrás. Apago el motor.
        pwm_final = 0; 
        adelante = false; 
      }
    }

    if (pwm_final > 255) pwm_final = 255;

    // Apagar todos por seguridad
    ledcWrite(FL_PWM, 0); ledcWrite(FR_PWM, 0); 
    ledcWrite(RL_PWM, 0); ledcWrite(RR_PWM, 0);

    // Aplicar solo al motor seleccionado
    if (MOTOR_A_PROBAR == M_FL) {
      digitalWrite(FL_DIR, adelante ? LOW : HIGH);
      ledcWrite(FL_PWM, pwm_final);
    } 
    else if (MOTOR_A_PROBAR == M_FR) {
      digitalWrite(FR_DIR, adelante ? HIGH : LOW);
      ledcWrite(FR_PWM, pwm_final);
    } 
    else if (MOTOR_A_PROBAR == M_RL) {
      digitalWrite(RL_DIR, adelante ? LOW : HIGH);
      ledcWrite(RL_PWM, pwm_final);
    } 
    else if (MOTOR_A_PROBAR == M_RR) {
      digitalWrite(RR_DIR, adelante ? HIGH : LOW);
      ledcWrite(RR_PWM, pwm_final);
    }

    // ==========================================
    // 5. SERIAL PLOTTER (DATOS CRUDOS)
    // ==========================================
    int pwm_real_enviado = adelante ? pwm_final : -pwm_final;
    if (pwm_final == 0) pwm_real_enviado = 0; 

    Serial.print("Ref:"); Serial.print(vel_deseada_ms, 3); Serial.print(" ");
    Serial.print("v:"); Serial.print(m_s_actual, 3); Serial.print(" ");
    Serial.print("PWM:"); Serial.println(pwm_real_enviado);
  }
}