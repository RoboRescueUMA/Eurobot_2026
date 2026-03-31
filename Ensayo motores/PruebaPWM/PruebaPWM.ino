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
const float radio = 0.0325f; // Radio 65mm

// Resolución 2X (Solo interrupción Canal A)
const int PPR_MOTOR = 11;          
const int GEAR_RATIO = 34;         
const int COUNTS_PER_REV = (PPR_MOTOR * GEAR_RATIO) * 2; // 748 cuentas por vuelta
const float rads_por_cuenta = (2.0f * PI) / (float)COUNTS_PER_REV;

// ================================================================
//  VARIABLES DE ENCODERS Y ESTADO
// ================================================================
volatile long cont_FL = 0;
volatile long cont_FR = 0;
volatile long cont_RL = 0;
volatile long cont_RR = 0;

long prev_FL = 0, prev_FR = 0, prev_RL = 0, prev_RR = 0;
float vel_m_s_FL = 0.0, vel_m_s_FR = 0.0, vel_m_s_RL = 0.0, vel_m_s_RR = 0.0;

const unsigned long ventana_us = 40000; // 40ms de refresco
unsigned long lastWindowUs = 0;

int pwm_prueba = 0;
unsigned long ultimo_paso_rampa = 0;

// VARIABLES DE SEGURIDAD PARA EL CABLE CORTO
bool ensayo_terminado = false;
int ciclos_en_movimiento = 0;

// ================================================================
//  INTERRUPCIONES (Resolución 2X)
// ================================================================
void IRAM_ATTR isr_FL_A() {
  if (digitalRead(FL_ENC_A) == digitalRead(FL_ENC_B)) cont_FL++; else cont_FL--;
}
void IRAM_ATTR isr_FR_A() {
  if (digitalRead(FR_ENC_A) == digitalRead(FR_ENC_B)) cont_FR++; else cont_FR--;
}
void IRAM_ATTR isr_RL_A() {
  if (digitalRead(RL_ENC_A) == digitalRead(RL_ENC_B)) cont_RL++; else cont_RL--;
}
void IRAM_ATTR isr_RR_A() {
  if (digitalRead(RR_ENC_A) == digitalRead(RR_ENC_B)) cont_RR++; else cont_RR--;
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(2000); // Tiempo para abrir el monitor serie
  Serial.println("=======================================");
  Serial.println("  INICIANDO ENSAYO DE ZONA MUERTA (v3.x)");
  Serial.println("=======================================");

  // Configurar DIR (Invertimos el lado izquierdo para que avance recto)
  pinMode(FL_DIR, OUTPUT); digitalWrite(FL_DIR, LOW);  // Invertido (Izquierda)
  pinMode(RL_DIR, OUTPUT); digitalWrite(RL_DIR, LOW);  // Invertido (Izquierda)
  
  pinMode(FR_DIR, OUTPUT); digitalWrite(FR_DIR, HIGH); // Normal (Derecha)
  pinMode(RR_DIR, OUTPUT); digitalWrite(RR_DIR, HIGH); // Normal (Derecha)

  // Sintaxis ESP32 Core v3.x (Se usa ledcAttach directo al pin)
  ledcAttach(FL_PWM, freq, resolution);
  ledcAttach(FR_PWM, freq, resolution);
  ledcAttach(RL_PWM, freq, resolution);
  ledcAttach(RR_PWM, freq, resolution);

  // Asegurar motores parados inicialmente
  ledcWrite(FL_PWM, 0); ledcWrite(FR_PWM, 0);
  ledcWrite(RL_PWM, 0); ledcWrite(RR_PWM, 0);

  pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(RL_ENC_A, INPUT_PULLUP); pinMode(RL_ENC_B, INPUT_PULLUP);
  pinMode(RR_ENC_A, INPUT_PULLUP); pinMode(RR_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isr_FL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isr_FR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isr_RL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isr_RR_A, CHANGE);

  lastWindowUs = micros();
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  // 0. FRENO DE EMERGENCIA (Si el ensayo ha terminado, bloqueamos aquí)
  if (ensayo_terminado) {
    ledcWrite(FL_PWM, 0); ledcWrite(FR_PWM, 0);
    ledcWrite(RL_PWM, 0); ledcWrite(RR_PWM, 0);
    delay(500);
    return; // No ejecuta nada más del loop
  }

  unsigned long ahora_us = micros();
  unsigned long ahora_ms = millis();

  // 1. GENERADOR DE RAMPA DE PWM
  if (ahora_ms - ultimo_paso_rampa >= 100) { 
    pwm_prueba += 1; 

    // Tope de seguridad duro por si los encoders fallan o patinan
    if (pwm_prueba >= 150) { 
      ensayo_terminado = true;
      Serial.println("\n--- TOPE DE SEGURIDAD (PWM 150) ALCANZADO ---");
    }

    // Aplicar a los motores
    ledcWrite(FL_PWM, pwm_prueba);
    ledcWrite(FR_PWM, pwm_prueba);
    ledcWrite(RL_PWM, pwm_prueba);
    ledcWrite(RR_PWM, pwm_prueba);
    
    ultimo_paso_rampa = ahora_ms;
  }

  // 2. CÁLCULO ROBUSTO DE VELOCIDAD
  if ((ahora_us - lastWindowUs) >= ventana_us) {
    
    // Proteger lectura de variables volátiles
    portENTER_CRITICAL(&mux);
    long current_FL = cont_FL;
    long current_FR = cont_FR;
    long current_RL = cont_RL;
    long current_RR = cont_RR;
    portEXIT_CRITICAL(&mux);

    unsigned long ventana_real_us = ahora_us - lastWindowUs;
    if (ventana_real_us == 0) ventana_real_us = 1;

    // Deltas de pulsos
    long delta_FL = current_FL - prev_FL;
    long delta_FR = current_FR - prev_FR;
    long delta_RL = current_RL - prev_RL;
    long delta_RR = current_RR - prev_RR;

    prev_FL = current_FL;
    prev_FR = current_FR;
    prev_RL = current_RL;
    prev_RR = current_RR;

    // Calcular cps
    float cps_FL = ((float)delta_FL * 1000000.0f) / (float)ventana_real_us;
    float cps_FR = ((float)delta_FR * 1000000.0f) / (float)ventana_real_us;
    float cps_RL = ((float)delta_RL * 1000000.0f) / (float)ventana_real_us;
    float cps_RR = ((float)delta_RR * 1000000.0f) / (float)ventana_real_us;

    // Pasarlo a m/s (INVERTIMOS EL SIGNO DE FL Y RL AQUÍ)
    vel_m_s_FL = -(cps_FL * rads_por_cuenta * radio);
    vel_m_s_FR = cps_FR * rads_por_cuenta * radio;
    vel_m_s_RL = -(cps_RL * rads_por_cuenta * radio);
    vel_m_s_RR = cps_RR * rads_por_cuenta * radio;

    // 3. IMPRESIÓN DE DATOS
    Serial.print("PWM:"); Serial.print(pwm_prueba);
    Serial.print(" V_FL:"); Serial.print(vel_m_s_FL, 3);
    Serial.print(" V_FR:"); Serial.print(vel_m_s_FR, 3);
    Serial.print(" V_RL:"); Serial.print(vel_m_s_RL, 3);
    Serial.print(" V_RR:"); Serial.println(vel_m_s_RR, 3);

    // 4. LÓGICA DE PARADA AUTOMÁTICA
    // Usamos abs() por si en algún momento patinan hacia atrás, pero la lectura normal ya será positiva
    if (abs(vel_m_s_FL) > 0.05 && abs(vel_m_s_FR) > 0.05 && abs(vel_m_s_RL) > 0.05 && abs(vel_m_s_RR) > 0.05) {
      ciclos_en_movimiento++;
      
      // Dejamos que imprima unas cuantas lineas más (200ms) para que veas el salto claro y cortamos
      if (ciclos_en_movimiento > 20) {
        ensayo_terminado = true;
        Serial.println("\n=======================================");
        Serial.println("  MOVIMIENTO DETECTADO Y REGISTRADO");
        Serial.println("  MOTORES APAGADOS POR SEGURIDAD");
        Serial.println("  Pulsa RESET (EN) en la placa para repetir");
        Serial.println("=======================================");
      }
    } else {
      // Si se atasca o baja la velocidad por fricción, reseteamos el contador
      ciclos_en_movimiento = 0;
    }

    lastWindowUs = ahora_us;
  }
}