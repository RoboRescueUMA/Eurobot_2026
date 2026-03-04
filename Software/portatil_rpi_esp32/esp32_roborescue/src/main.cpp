#include <Arduino.h>
#include <math.h>  // Para sqrt() en cálculo de L
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

// ================================================================
//  CONFIGURACIÓN DE PINES - INTERCAMBIADOS FRENTE/ATRÁS
// ================================================================

// DRIVER 1 (AHORA DELANTERO - los motores que SÍ funcionan bien)
// Motor 1: Frontal Izquierda (FL)
#define FL_PWM 27   
#define FL_DIR 14  

// Motor 2: Frontal Derecha (FR)
#define FR_PWM 25   
#define FR_DIR 26   // Cambiado de 16 a 26 (16 conflicto con RL_ENC_A)

// DRIVER 2 (AHORA TRASERO - los motores que necesitaban más potencia)
// Motor 3: Trasero Izquierda (RL)
#define RL_PWM 32
#define RL_DIR 33

// Motor 4: Trasero Derecha (RR)
#define RR_PWM 18
#define RR_DIR 19

// ================================================================
//  PINES ENCODERS (Canales A y B)
// ================================================================
// Pines GPIO disponibles en ESP32 DevKit v1 (30 pines)
// EVITANDO pines sensibles al boot: 0, 2, 12, 15
// EVITANDO: GPIO 1,3 (Serial), 6-11 (Flash), pines ya usados por motores
#define FL_ENC_A 21
#define FL_ENC_B 22
#define FR_ENC_A 34
#define FR_ENC_B 35
#define RL_ENC_A 16
#define RL_ENC_B 17
#define RR_ENC_A 23
#define RR_ENC_B 4

// Configuración PWM
const int freq = 1000;
const int resolution = 8;
const int ch_RL = 0; 
const int ch_RR = 1;
const int ch_FL = 2;
const int ch_FR = 3;

// ================================================================
//  CONFIGURACIÓN ENCODERS
// ================================================================
// Especificaciones del motor
const int PPR_MOTOR = 11;           // Pulsos por revolución del motor
const int GEAR_RATIO = 34;          // Relación de reducción 1:34
const int PPR_WHEEL = PPR_MOTOR * GEAR_RATIO;  // 11 * 34 = 374 pulsos/vuelta en rueda
const int COUNTS_PER_REV = PPR_WHEEL * 4;      // Con cuadratura x4 = 1496 counts/vuelta

// Variables de conteo de encoders (volatile porque se usan en ISR)
volatile long enc_count_FL = 0;
volatile long enc_count_FR = 0;
volatile long enc_count_RL = 0;
volatile long enc_count_RR = 0;

// Variables para cálculo de velocidad
unsigned long last_encoder_time = 0;
const unsigned long ENCODER_SAMPLE_TIME = 50; // Calcular velocidad cada 50ms

// Velocidades calculadas (en pulsos/seg)
float vel_FL = 0.0;
float vel_FR = 0.0;
float vel_RL = 0.0;
float vel_RR = 0.0;

// ================================================================
//  CONTROL PID
// ================================================================
// Velocidades deseadas (en m/s) - calculadas por cinemática
float target_ms_FL = 0.0;
float target_ms_FR = 0.0;
float target_ms_RL = 0.0;
float target_ms_RR = 0.0;

// PWM actual de cada motor (-255 a 255, con signo)
int pwm_FL = 0;
int pwm_FR = 0;
int pwm_RL = 0;
int pwm_RR = 0;

// Ganancias PID - calculadas por IMC con modelo G(s) = 0.68 / (0.15s + 1)
// K=0.68, tau=0.15s, lambda=tau → Kp=tau/(K*lambda)=1.47, Ki=Kp/tau=9.8, Kd=0
const float Kp = 1.47;
const float Ki = 9.8;
const float Kd = 0.0;

// Velocidad máxima del motor (m/s) - medida empíricamente a PWM=255
const float MAX_VEL = 0.513;  // m/s a PWM=255

// Velocidad máxima esperada en RPM (para normalizar)
// Solo usado en test de motores para calcular v_ref de referencia
const float MAX_RPM = 150.0;

// Variables internas PID (una por motor)
float integral_FL = 0.0, integral_FR = 0.0, integral_RL = 0.0, integral_RR = 0.0;
float prev_error_FL = 0.0, prev_error_FR = 0.0, prev_error_RL = 0.0, prev_error_RR = 0.0;

// Límite anti-windup del integrador (en m/s * s)
const float INTEGRAL_MAX = 1.0;

// ================================================================
//  GEOMETRÍA DEL ROBOT (para cinemática)
// ================================================================
// Distancia del centro del robot a las ruedas (en metros)
const float Lx = 0.10;  // TODO: Medir distancia centro → rueda (eje X)
const float Ly = 0.14;  // TODO: Medir distancia centro → rueda (eje Y)
// Para X-Drive: distancia efectiva = diagonal
const float L = Lx + Ly; 

// ================================================================
//  INTERRUPCIONES ENCODERS (ISR)
// ================================================================
// ISR para Front Left (FL)
void IRAM_ATTR isr_FL_A() {
  if (digitalRead(FL_ENC_A) == digitalRead(FL_ENC_B)) {
    enc_count_FL++;
  } else {
    enc_count_FL--;
  }
}

// ISR para Front Right (FR)
void IRAM_ATTR isr_FR_A() {
  if (digitalRead(FR_ENC_A) == digitalRead(FR_ENC_B)) {
    enc_count_FR++;
  } else {
    enc_count_FR--;
  }
}

// ISR para Rear Left (RL)
void IRAM_ATTR isr_RL_A() {
  if (digitalRead(RL_ENC_A) == digitalRead(RL_ENC_B)) {
    enc_count_RL++;
  } else {
    enc_count_RL--;
  }
}

// ISR para Rear Right (RR)
void IRAM_ATTR isr_RR_A() {
  if (digitalRead(RR_ENC_A) == digitalRead(RR_ENC_B)) {
    enc_count_RR++;
  } else {
    enc_count_RR--;
  }
} 

// Variables ROS
rcl_subscription_t subscriber;
rcl_publisher_t publisher_encoder_vel;
rcl_publisher_t publisher_debug;
geometry_msgs__msg__Twist msg;
std_msgs__msg__Float32MultiArray encoder_msg;
std_msgs__msg__Float32MultiArray debug_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;
bool micro_ros_connected = false;
unsigned long last_msg_time = 0;

// ================================================================
//  CONTROLADOR MOTOR (DFRobot Logic: PWM + DIR)
// ================================================================
void set_motor(int channel, int dir_pin, float speed, bool invert_dir = false, const char* motor_name = "", int* pwm_out = NULL) {
  // Limitar
  if (speed > 1.0) speed = 1.0;
  if (speed < -1.0) speed = -1.0;

  int pwm = fabs(speed) * 255;
  
  // PWM MÍNIMO para vencer fricción (ajustado por pruebas empíricas)
  // Movimiento lateral requiere más PWM que movimiento adelante/atrás
  const int PWM_MIN = 80;
  if (pwm > 0 && pwm < PWM_MIN) {
    pwm = PWM_MIN;
  }

  // DIRECCIÓN (con opción de invertir)
  bool direction = (speed > 0);
  if (invert_dir) direction = !direction;
  
  digitalWrite(dir_pin, direction ? HIGH : LOW);
  
  ledcWrite(channel, pwm);
  
  // Guardar PWM actual si se proporciona puntero
  if (pwm_out != NULL) {
    *pwm_out = direction ? pwm : -pwm;  // Guardar con signo
  }
}

// Forward declaration (definida más abajo)
float vel_to_pwm(float v_target);

// ================================================================
//  CALLBACK CINEMÁTICA MECANUM CON CONTROL P
// ================================================================
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_msg_time = millis(); // Resetear watchdog

  float x = msg->linear.x;  // Adelante (+) / Atrás (-) [m/s]
  float y = msg->linear.y;   // Izquierda (+) / Derecha (-) [m/s]
  float w = msg->angular.z;  // Giro antihorario (+) / horario (-) [rad/s]
  
  // Convertir velocidad angular (rad/s) a velocidad tangencial (m/s)
  // w positivo = giro antihorario, pero las ruedas giran al revés
  float z = -w * L/L;  // Invertir signo de rotación y escalar por distancia

  // MODO DEBUG: Activar solo un motor a la vez
  // Descomentar UNA línea para probar cada motor individualmente
  
  // set_motor(ch_FL, FL_DIR, x, false, "FL", &pwm_FL);  // Probar FL solo
  // set_motor(ch_FR, FR_DIR, x, false, "FR", &pwm_FR);  // Probar FR solo
  // set_motor(ch_RL, RL_DIR, x, false, "RL", &pwm_RL);  // Probar RL solo
  // set_motor(ch_RR, RR_DIR, x, false, "RR", &pwm_RR);  // Probar RR solo
  
  // Cinemática X-Drive con z invertido
  float fl = -(x - y + z);  // Front Left
  float fr = x + y - z;  // Front Right
  float rl = -(x + y + z);  // Rear Left
  float rr = x - y - z;  // Rear Right

  // Sin compensación de FL

  // Normalizar
  float max_val = max(fabs(fl), max(fabs(fr), max(fabs(rl), fabs(rr))));
  if (max_val > 1.0) {
    fl /= max_val; fr /= max_val; rl /= max_val; rr /= max_val;
  }

  // Calcular velocidades objetivo en m/s (limitadas a MAX_VEL)
  target_ms_FL = fl * MAX_VEL;
  target_ms_FR = fr * MAX_VEL;
  target_ms_RL = rl * MAX_VEL;
  target_ms_RR = rr * MAX_VEL;

  // DEBUG: Comentado - Serial no disponible mientras micro-ROS activo
  // Ver datos en /roborescue/encoder_debug topic
  // static unsigned long last_callback_print = 0;
  // unsigned long now = millis();
  // if (now - last_callback_print > 1000) {
  //   Serial.println("\n=== CINEMÁTICA ===");
  //   Serial.print("Input: x="); Serial.print(x); ...
  //   last_callback_print = now;
  // }

  // Enviar comandos a motores y guardar PWM actual
  set_motor(ch_FL, FL_DIR, fl, false, "FL", &pwm_FL);
  set_motor(ch_FR, FR_DIR, fr, false, "FR", &pwm_FR);
  set_motor(ch_RL, RL_DIR, rl, false, "RL", &pwm_RL);
  set_motor(ch_RR, RR_DIR, rr, false, "RR", &pwm_RR);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("========================================");
  Serial.println("   INICIANDO ESP32 MECANUM");
  Serial.println("========================================");

  // PASO 1: Configurar pines de dirección como OUTPUT y ponerlos en LOW
  pinMode(RL_DIR, OUTPUT); digitalWrite(RL_DIR, LOW);
  pinMode(RR_DIR, OUTPUT); digitalWrite(RR_DIR, LOW);
  pinMode(FL_DIR, OUTPUT); digitalWrite(FL_DIR, LOW);
  pinMode(FR_DIR, OUTPUT); digitalWrite(FR_DIR, LOW);
  
  // PASO 2: Configurar PWM
  ledcSetup(ch_RL, freq, resolution); ledcAttachPin(RL_PWM, ch_RL);
  ledcSetup(ch_RR, freq, resolution); ledcAttachPin(RR_PWM, ch_RR);
  ledcSetup(ch_FL, freq, resolution); ledcAttachPin(FL_PWM, ch_FL);
  ledcSetup(ch_FR, freq, resolution); ledcAttachPin(FR_PWM, ch_FR);

  // PASO 3: Asegurar PWM en 0
  ledcWrite(ch_RL, 0);
  ledcWrite(ch_RR, 0);
  ledcWrite(ch_FL, 0);
  ledcWrite(ch_FR, 0);
  
  Serial.println("✅ Hardware configurado");
  
  // PASO 4: Configurar pines de encoders como INPUT con PULLUP
  Serial.println("Configurando encoders...");
  pinMode(FL_ENC_A, INPUT_PULLUP);
  pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP);
  pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(RL_ENC_A, INPUT_PULLUP);
  pinMode(RL_ENC_B, INPUT_PULLUP);
  pinMode(RR_ENC_A, INPUT_PULLUP);
  pinMode(RR_ENC_B, INPUT_PULLUP);
  
  // PASO 5: Configurar interrupciones para encoders (solo canal A)
  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isr_FL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isr_FR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isr_RL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isr_RR_A, CHANGE);
  
  Serial.println("✅ Encoders configurados");
  
  // ============================================================
  // CONFIGURACIÓN MICRO-ROS CON DOMAIN ID Y NAMESPACE
  // ============================================================
  
  // Configurar transporte micro-ROS
  set_microros_serial_transports(Serial);

  // A partir de aquí Serial está ocupado con micro-ROS — NO usar Serial.print

  // Inicialización Micro-ROS
  // Serial.println("\nInicializando Micro-ROS...");
  allocator = rcl_get_default_allocator();
  
  // 1. Inicializar las opciones de init
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  
  // 2. CONFIGURAR DOMAIN ID = 17 (igual que RPI y laptop)
  // Serial.println("📡 Configurando ROS_DOMAIN_ID = 17");
  rcl_init_options_set_domain_id(&init_options, 17);
  
  // 3. Inicializar rclc_support CON las opciones configuradas
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  
  // 4. Inicializar nodo CON namespace 'roborescue'
  const char * node_name = "esp32_mecanum";
  const char * node_namespace = "roborescue";
  
  // Serial.print("🤖 Inicializando nodo: roborescue/esp32_mecanum");
  
  //rclc_node_init_default(&node, node_name, node_namespace, &support);
  rclc_node_init_default(&node, node_name, node_namespace, &support);
  // 5. Suscribirse al topic /roborescue/cmd_vel
  rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");
  
  // 6. Crear publisher para velocidades de encoders
  // Inicializar mensaje de encoders (4 floats: FL, FR, RL, RR en RPM)
  encoder_msg.data.capacity = 4;
  encoder_msg.data.size = 4;
  encoder_msg.data.data = (float*) malloc(encoder_msg.data.capacity * sizeof(float));
  
  rclc_publisher_init_default(
    &publisher_encoder_vel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "encoder_velocities");
  
  // Publisher para debug (8 floats: 4 target_rpm + 4 actual_rpm)
  debug_msg.data.capacity = 8;
  debug_msg.data.size = 8;
  debug_msg.data.data = (float*) malloc(debug_msg.data.capacity * sizeof(float));
  
  rclc_publisher_init_default(
    &publisher_debug, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "encoder_debug");
     
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  
  // Serial: micro-ROS activo, debug disponible en /roborescue/encoder_debug
  
  last_encoder_time = millis();
}

// ================================================================
//  FEEDFORWARD NO LINEAL: v (m/s) → PWM
//  Basado en medición empírica de FL a 4 puntos
//  (PWM/255, v_SS): (0.298, 0.253), (0.498, 0.376), (0.698, 0.440), (1.0, 0.513)
// ================================================================
float vel_to_pwm(float v_target) {
  if (v_target == 0.0) return 0.0;
  float sign = (v_target > 0) ? 1.0 : -1.0;
  float v = fabs(v_target);

  // Interpolación lineal por tramos (v → PWM normalizado 0-1)
  float u;
  if      (v <= 0.253) u = (v / 0.253) * 0.298;
  else if (v <= 0.376) u = 0.298 + (v - 0.253) / (0.376 - 0.253) * (0.498 - 0.298);
  else if (v <= 0.440) u = 0.498 + (v - 0.376) / (0.440 - 0.376) * (0.698 - 0.498);
  else if (v <= 0.513) u = 0.698 + (v - 0.440) / (0.513 - 0.440) * (1.000 - 0.698);
  else                 u = 1.0;

  return sign * u * 255.0;
}

// ================================================================
//  FUNCIÓN PARA CALCULAR Y PUBLICAR VELOCIDADES DE ENCODERS
//  CON CONTROL PROPORCIONAL (P)
// ================================================================
void update_encoder_velocities() {
  unsigned long current_time = millis();
  float dt = (current_time - last_encoder_time) / 1000.0;  // Tiempo en segundos
  
  if (dt >= ENCODER_SAMPLE_TIME / 1000.0) {
    // Calcular velocidad en pulsos/segundo
    vel_FL = enc_count_FL / dt;
    vel_FR = enc_count_FR / dt;
    vel_RL = enc_count_RL / dt;
    vel_RR = enc_count_RR / dt;
    
    // Convertir counts a velocidad en m/s
    const float RPM_TO_MS = (2.0 * 3.14159 * 0.0325) / 60.0;
    float rpm_FL = (vel_FL * 60.0) / COUNTS_PER_REV;
    float rpm_FR = (vel_FR * 60.0) / COUNTS_PER_REV;
    float rpm_RL = (vel_RL * 60.0) / COUNTS_PER_REV;
    float rpm_RR = (vel_RR * 60.0) / COUNTS_PER_REV;
    float vel_ms_FL = rpm_FL * RPM_TO_MS;
    float vel_ms_FR = rpm_FR * RPM_TO_MS;
    float vel_ms_RL = rpm_RL * RPM_TO_MS;
    float vel_ms_RR = rpm_RR * RPM_TO_MS;

    // ============================================================
    // CONTROL PID: salida directa en PWM (no acumulativa)
    // Modelo: G(s) = 0.68 / (0.15s + 1), Kp=1.47, Ki=9.8, Kd=0
    // ============================================================
    float error_FL = target_ms_FL - vel_ms_FL;
    float error_FR = target_ms_FR - vel_ms_FR;
    float error_RL = target_ms_RL - vel_ms_RL;
    float error_RR = target_ms_RR - vel_ms_RR;

    // Anti-windup: solo integrar si no estamos saturados
    if (abs(pwm_FL) < 255) integral_FL = constrain(integral_FL + error_FL * dt, -INTEGRAL_MAX, INTEGRAL_MAX);
    if (abs(pwm_FR) < 255) integral_FR = constrain(integral_FR + error_FR * dt, -INTEGRAL_MAX, INTEGRAL_MAX);
    if (abs(pwm_RL) < 255) integral_RL = constrain(integral_RL + error_RL * dt, -INTEGRAL_MAX, INTEGRAL_MAX);
    if (abs(pwm_RR) < 255) integral_RR = constrain(integral_RR + error_RR * dt, -INTEGRAL_MAX, INTEGRAL_MAX);

    float deriv_FL = (error_FL - prev_error_FL) / dt;
    float deriv_FR = (error_FR - prev_error_FR) / dt;
    float deriv_RL = (error_RL - prev_error_RL) / dt;
    float deriv_RR = (error_RR - prev_error_RR) / dt;

    prev_error_FL = error_FL;
    prev_error_FR = error_FR;
    prev_error_RL = error_RL;
    prev_error_RR = error_RR;

    // Salida PID: feedforward no lineal + corrección PI
    float ff_FL = vel_to_pwm(target_ms_FL);
    float ff_FR = vel_to_pwm(target_ms_FR);
    float ff_RL = vel_to_pwm(target_ms_RL);
    float ff_RR = vel_to_pwm(target_ms_RR);

    pwm_FL = constrain((int)(ff_FL + Kp*error_FL + Ki*integral_FL + Kd*deriv_FL), -255, 255);
    pwm_FR = constrain((int)(ff_FR + Kp*error_FR + Ki*integral_FR + Kd*deriv_FR), -255, 255);
    pwm_RL = constrain((int)(ff_RL + Kp*error_RL + Ki*integral_RL + Kd*deriv_RL), -255, 255);
    pwm_RR = constrain((int)(ff_RR + Kp*error_RR + Ki*integral_RR + Kd*deriv_RR), -255, 255);

    // Aplicar PWM corregido a motores
    ledcWrite(ch_FL, fabs(pwm_FL));
    digitalWrite(FL_DIR, pwm_FL > 0 ? HIGH : LOW);

    ledcWrite(ch_FR, fabs(pwm_FR));
    digitalWrite(FR_DIR, pwm_FR > 0 ? HIGH : LOW);

    ledcWrite(ch_RL, fabs(pwm_RL));
    digitalWrite(RL_DIR, pwm_RL > 0 ? HIGH : LOW);

    ledcWrite(ch_RR, fabs(pwm_RR));
    digitalWrite(RR_DIR, pwm_RR > 0 ? HIGH : LOW);
    
    // Publicar velocidades en ROS (en m/s)
    encoder_msg.data.data[0] = vel_ms_FL;
    encoder_msg.data.data[1] = vel_ms_FR;
    encoder_msg.data.data[2] = vel_ms_RL;
    encoder_msg.data.data[3] = vel_ms_RR;
    
    rcl_publish(&publisher_encoder_vel, &encoder_msg, NULL);
    
    // Debug: [target_FL, target_FR, target_RL, target_RR, actual_FL, actual_FR, actual_RL, actual_RR] en m/s
    debug_msg.data.data[0] = target_ms_FL;
    debug_msg.data.data[1] = target_ms_FR;
    debug_msg.data.data[2] = target_ms_RL;
    debug_msg.data.data[3] = target_ms_RR;
    debug_msg.data.data[4] = vel_ms_FL;
    debug_msg.data.data[5] = vel_ms_FR;
    debug_msg.data.data[6] = vel_ms_RL;
    debug_msg.data.data[7] = vel_ms_RR;
    
    rcl_publish(&publisher_debug, &debug_msg, NULL);
    
     // DEBUG: Imprimir en serial cada 500ms (COMENTADO mientras micro-ROS activo)
     // Data is published via /roborescue/encoder_debug topic instead
     // static unsigned long last_debug_print = 0;
     // if (current_time - last_debug_print > 500) {
     //   Serial.print("Target: [");
     //   Serial.print(target_rpm_FL); Serial.print(", ");
     //   Serial.print(target_rpm_FR); Serial.print(", ");
     //   Serial.print(target_rpm_RL); Serial.print(", ");
     //   Serial.print(target_rpm_RR); Serial.print("] | Actual: [");
     //   Serial.print(rpm_FL); Serial.print(", ");
     //   Serial.print(rpm_FR); Serial.print(", ");
     //   Serial.print(rpm_RL); Serial.print(", ");
     //   Serial.print(rpm_RR); Serial.print("] | PWM: [");
     //   Serial.print(pwm_FL); Serial.print(", ");
     //   Serial.print(pwm_FR); Serial.print(", ");
     //   Serial.print(pwm_RL); Serial.print(", ");
     //   Serial.print(pwm_RR); Serial.println("]");
     //   last_debug_print = current_time;
     // }
    
    // Resetear contadores
    enc_count_FL = 0;
    enc_count_FR = 0;
    enc_count_RL = 0;
    enc_count_RR = 0;
    
    last_encoder_time = current_time;
  }
}

void loop() {
  // Ejecutar ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  
  // Actualizar y publicar velocidades de encoders
  update_encoder_velocities();

  // SAFETY: Si no llegan comandos en 0.5s, parar todo
  if (millis() - last_msg_time > 500) {
    // Resetear targets, PWM e integrales PID
    target_ms_FL = 0; target_ms_FR = 0; target_ms_RL = 0; target_ms_RR = 0;
    pwm_FL = 0; pwm_FR = 0; pwm_RL = 0; pwm_RR = 0;
    integral_FL = 0; integral_FR = 0; integral_RL = 0; integral_RR = 0;
    prev_error_FL = 0; prev_error_FR = 0; prev_error_RL = 0; prev_error_RR = 0;
    
    set_motor(ch_FL, FL_DIR, 0); set_motor(ch_FR, FR_DIR, 0);
    set_motor(ch_RL, RL_DIR, 0); set_motor(ch_RR, RR_DIR, 0);
  }
}
