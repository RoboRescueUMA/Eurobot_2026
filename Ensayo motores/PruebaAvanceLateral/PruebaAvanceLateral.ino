#include <Arduino.h>

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ================================================================
//  ZONA DE CONFIGURACIÓN DEL CHASIS
// ================================================================
float Kp = 159.7; 
float Ki = 1078;

// Compensación de fricción individual de tus 4 motores
const int base_FL = 89;
const int base_FR = 47;
const int base_RL = 84;
const int base_RR = 78;

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

// ================================================================
//  CONSTANTES
// ================================================================
const float RADIO_RUEDA = 0.0325; 
const int PPR_MOTOR = 11;          
const int GEAR_RATIO = 34;         
const int COUNTS_PER_REV = (PPR_MOTOR * GEAR_RATIO) * 4; 
const float rads_por_cuenta = (2.0f * PI) / (float)COUNTS_PER_REV;

const unsigned long ventana_us = 40000; // 40ms

// ================================================================
//  VARIABLES DE ESTADO
// ================================================================
volatile long cont_FL = 0, cont_FR = 0, cont_RL = 0, cont_RR = 0;
long prev_FL = 0, prev_FR = 0, prev_RL = 0, prev_RR = 0;

unsigned long lastWindowUs = 0;
unsigned long t_inicio_prueba = 0; 

// Variables PID independientes para cada rueda
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
//  FUNCIÓN PID BIDIRECCIONAL (CORREGIDA ANTI-OSCILACIONES)
// ================================================================
int calcular_pwm_rueda(float v_deseada, float v_actual, double &memoria_integral, int pwm_base_motor) {
  if (v_deseada == 0.0) {
    memoria_integral = 0;
    return 0;
  }

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
  } 
  else if (v_deseada < 0.0) {
    if (Output < 0) pwm_final = -(pwm_base_motor + (int)abs(Output));
    else pwm_final = 0; 
  }

  if (pwm_final > 255) pwm_final = 255;
  if (pwm_final < -255) pwm_final = -255;
  
  return pwm_final;
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(3000); // 3 segundos para poner el robot en el suelo

  pinMode(FL_DIR, OUTPUT); pinMode(RL_DIR, OUTPUT);
  pinMode(FR_DIR, OUTPUT); pinMode(RR_DIR, OUTPUT);

  digitalWrite(FL_DIR, LOW);
  digitalWrite(RL_DIR, LOW);
  digitalWrite(FR_DIR, HIGH); 
  digitalWrite(RR_DIR, HIGH);

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
  t_inicio_prueba = millis(); 
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  if (ensayo_finalizado) {
    ledcWrite(FL_PWM, 0); ledcWrite(FR_PWM, 0);
    ledcWrite(RL_PWM, 0); ledcWrite(RR_PWM, 0);
    return; 
  }

  unsigned long ahora_us = micros();

  if ((ahora_us - lastWindowUs) >= ventana_us) {
    unsigned long ventana_real_us = ahora_us - lastWindowUs;
    lastWindowUs = ahora_us;

    // 1. GENERAR MÁQUINA DE ESTADOS (Secuencia de movimientos)
    unsigned long tiempo_desde_inicio = millis() - t_inicio_prueba;
    
    // Cada bloque dura 4000ms (3000ms moviéndose + 1000ms frenando)
    int fase = tiempo_desde_inicio / 4000; 
    bool en_movimiento = (tiempo_desde_inicio % 4000) < 3000;

    float ref_FL = 0.0, ref_FR = 0.0, ref_RL = 0.0, ref_RR = 0.0;
    float v_crucero = 0.1; // Velocidad de prueba para todos los movimientos

    if (en_movimiento) {
      switch (fase) {
        case 0: // 1. Diagonal Adelante-Derecha
          ref_FL = v_crucero;  ref_FR = 0.0; 
          ref_RL = 0.0;        ref_RR = v_crucero;
          break;
        case 1: // 2. Diagonal Adelante-Izquierda
          ref_FL = 0.0;        ref_FR = v_crucero; 
          ref_RL = v_crucero;  ref_RR = 0.0;
          break;
        case 2: // 3. Diagonal Atrás-Derecha
          ref_FL = 0.0;        ref_FR = -v_crucero; 
          ref_RL = -v_crucero; ref_RR = 0.0;
          break;
        case 3: // 4. Diagonal Atrás-Izquierda
          ref_FL = -v_crucero; ref_FR = 0.0; 
          ref_RL = 0.0;        ref_RR = -v_crucero;
          break;
        case 4: // 5. Lateral Puro Derecha
          ref_FL = v_crucero;  ref_FR = -v_crucero; 
          ref_RL = -v_crucero; ref_RR = v_crucero;
          break;
        case 5: // 6. Lateral Puro Izquierda
          ref_FL = -v_crucero; ref_FR = v_crucero; 
          ref_RL = v_crucero;  ref_RR = -v_crucero;
          break;
        default:
          ensayo_finalizado = true;
          break;
      }
    } // Si no está "en_movimiento", todas las referencias valen 0.0 automáticamente

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

    // 3. CALCULAR VELOCIDADES INDIVIDUALES
    float rads_FL = -(((float)delta_FL * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;
    float rads_FR = (((float)delta_FR * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;
    float rads_RL = -(((float)delta_RL * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;
    float rads_RR = (((float)delta_RR * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;

    float v_FL = rads_FL * RADIO_RUEDA;
    float v_FR = rads_FR * RADIO_RUEDA;
    float v_RL = rads_RL * RADIO_RUEDA;
    float v_RR = rads_RR * RADIO_RUEDA;

    // 4. APLICAR PID A CADA RUEDA CON SU REFERENCIA INDIVIDUAL
    int pwm_FL = calcular_pwm_rueda(ref_FL, v_FL, errSum_FL, base_FL);
    int pwm_FR = calcular_pwm_rueda(ref_FR, v_FR, errSum_FR, base_FR);
    int pwm_RL = calcular_pwm_rueda(ref_RL, v_RL, errSum_RL, base_RL);
    int pwm_RR = calcular_pwm_rueda(ref_RR, v_RR, errSum_RR, base_RR);

    // 5. ESCRIBIR PWM Y DIRECCIÓN FÍSICA
    digitalWrite(FL_DIR, (pwm_FL >= 0) ? LOW : HIGH);
    ledcWrite(FL_PWM, abs(pwm_FL));

    digitalWrite(FR_DIR, (pwm_FR >= 0) ? HIGH : LOW);
    ledcWrite(FR_PWM, abs(pwm_FR));

    digitalWrite(RL_DIR, (pwm_RL >= 0) ? LOW : HIGH);
    ledcWrite(RL_PWM, abs(pwm_RL));

    digitalWrite(RR_DIR, (pwm_RR >= 0) ? HIGH : LOW);
    ledcWrite(RR_PWM, abs(pwm_RR));

    // 6. IMPRIMIR DATOS (Monitor Serie)
    // Para no saturar con líneas, solo imprimimos la fase y si está parado o moviéndose
    if (!ensayo_finalizado) {
        Serial.print("Fase: "); Serial.print(fase);
        Serial.print(" | Estado: "); Serial.println(en_movimiento ? "Moviendo" : "Pausa");
    } else {
        Serial.println("Prueba finalizada.");
    }
  }
}