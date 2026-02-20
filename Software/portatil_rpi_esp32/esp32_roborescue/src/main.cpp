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
#define FL_PWM 25   
#define FL_DIR 26  

// Motor 2: Frontal Derecha (FR)
#define FR_PWM 27   
#define FR_DIR 14   

// DRIVER 2 (AHORA TRASERO - los motores que necesitaban más potencia)
// Motor 3: Trasero Izquierda (RL)
#define RL_PWM 18
#define RL_DIR 19

// Motor 4: Trasero Derecha (RR)
#define RR_PWM 32
#define RR_DIR 33

// ================================================================
//  PINES ENCODERS (Canales A y B)
// ================================================================
// Pines GPIO disponibles en ESP32 DevKit v1 (30 pines)
// Evitando: GPIO 1,3 (Serial), 6-11 (Flash), pines ya usados por motores
#define FL_ENC_A 34
#define FL_ENC_B 35
#define FR_ENC_A 12
#define FR_ENC_B 13
#define RL_ENC_A 15
#define RL_ENC_B 4
#define RR_ENC_A 16
#define RR_ENC_B 17

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
geometry_msgs__msg__Twist msg;
std_msgs__msg__Float32MultiArray encoder_msg;
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
void set_motor(int channel, int dir_pin, float speed, bool invert_dir = false, const char* motor_name = "") {
  // Limitar
  if (speed > 1.0) speed = 1.0;
  if (speed < -1.0) speed = -1.0;

  int pwm = abs(speed) * 255;
  
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
}

// ================================================================
//  CALLBACK CINEMÁTICA MECANUM
// ================================================================
// PRUEBAS ESPERADAS (con z invertido):
// 1. linear.x = +0.3 → Robot avanza ADELANTE
// 2. linear.y = +0.3 → Robot se mueve a la IZQUIERDA (strafe)
// 3. angular.z = +0.5 → Robot gira HORARIO (sentido de las agujas del reloj)
//    NOTA: z invertido porque hardware requiere signo opuesto
// 4. Combinado: x=0.3, y=0.3 → Robot avanza en DIAGONAL (adelante-izquierda)
// 
// IMPORTANTE: Antes de probar, medir y actualizar Lx, Ly (líneas 40-42)
// ================================================================
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_msg_time = millis(); // Resetear watchdog

  float x = -msg->linear.x;  // Adelante (+) / Atrás (-) [m/s] - INVERTIDO por swap de pines
  float y = msg->linear.y;   // Izquierda (+) / Derecha (-) [m/s]
  float w = msg->angular.z;  // Giro antihorario (+) / horario (-) [rad/s]
  
  // Convertir velocidad angular (rad/s) a velocidad tangencial (m/s)
  // w positivo = giro antihorario, pero las ruedas giran al revés
  float z = -w * L/L;  // Invertir signo de rotación y escalar por distancia

  // MODO DEBUG: Activar solo un motor a la vez
  // Descomentar UNA línea para probar cada motor individualmente
  
  // set_motor(ch_FL, FL_DIR, x, false, "FL");  // Probar FL solo
  // set_motor(ch_FR, FR_DIR, x, false, "FR");  // Probar FR solo
  // set_motor(ch_RL, RL_DIR, x, false, "RL");  // Probar RL solo
  // set_motor(ch_RR, RR_DIR, x, false, "RR");  // Probar RR solo
  
  // Cinemática X-Drive con z invertido
  float fl = x - y + z;  // Front Left
  float fr = x + y - z;  // Front Right
  float rl = x + y + z;  // Rear Left
  float rr = x - y - z;  // Rear Right

  // COMPENSACIÓN: FL necesita más potencia (ajustar factor según necesites)
  fl *= 1.15;  // 15% más potencia para FL

  // Normalizar
  float max_val = max(abs(fl), max(abs(fr), max(abs(rl), abs(rr))));
  if (max_val > 1.0) {
    fl /= max_val; fr /= max_val; rl /= max_val; rr /= max_val;
  }

  set_motor(ch_FL, FL_DIR, fl, false, "FL");
  set_motor(ch_FR, FR_DIR, fr, false, "FR");
  set_motor(ch_RL, RL_DIR, rl, false, "RL");
  set_motor(ch_RR, RR_DIR, rr, false, "RR");
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
  delay(500);
  
  // ============================================================
  // CONFIGURACIÓN MICRO-ROS CON DOMAIN ID Y NAMESPACE
  // ============================================================
  
  // Configurar transporte micro-ROS
  set_microros_serial_transports(Serial);

  // Inicialización Micro-ROS
  Serial.println("\nInicializando Micro-ROS...");
  allocator = rcl_get_default_allocator();
  
  // 1. Inicializar las opciones de init
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  
  // 2. CONFIGURAR DOMAIN ID = 17 (igual que RPI y laptop)
  Serial.println("📡 Configurando ROS_DOMAIN_ID = 17");
  rcl_init_options_set_domain_id(&init_options, 17);
  
  // 3. Inicializar rclc_support CON las opciones configuradas
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  
  // 4. Inicializar nodo CON namespace 'roborescue'
  const char * node_name = "esp32_mecanum";
  const char * node_namespace = "roborescue";
  
  Serial.print("🤖 Inicializando nodo: ");
  Serial.print(node_namespace);
  Serial.print("/");
  Serial.println(node_name);
  
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
    
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  
  Serial.println("========================================");
  Serial.println("   ✅ MICRO-ROS INICIADO CORRECTAMENTE");
  Serial.println("========================================");
  Serial.print("   Domain ID: 17\n");
  Serial.print("   Namespace: roborescue\n");
  Serial.print("   Subscrito: /roborescue/cmd_vel\n");
  Serial.print("   Publicando: /roborescue/encoder_velocities\n");
  Serial.println("========================================\n");
  
  last_encoder_time = millis();
}

// ================================================================
//  FUNCIÓN PARA CALCULAR Y PUBLICAR VELOCIDADES DE ENCODERS
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
    
    // Convertir a RPM: (pulsos/seg) * (60 seg/min) / (pulsos/rev)
    float rpm_FL = (vel_FL * 60.0) / COUNTS_PER_REV;
    float rpm_FR = (vel_FR * 60.0) / COUNTS_PER_REV;
    float rpm_RL = (vel_RL * 60.0) / COUNTS_PER_REV;
    float rpm_RR = (vel_RR * 60.0) / COUNTS_PER_REV;
    
    // Publicar velocidades en ROS (en RPM)
    encoder_msg.data.data[0] = rpm_FL;
    encoder_msg.data.data[1] = rpm_FR;
    encoder_msg.data.data[2] = rpm_RL;
    encoder_msg.data.data[3] = rpm_RR;
    
    rcl_publish(&publisher_encoder_vel, &encoder_msg, NULL);
    
    // DEBUG: Imprimir en serial cada 500ms
    static unsigned long last_debug_print = 0;
    if (current_time - last_debug_print > 500) {
      Serial.print("Encoders RPM -> FL:");
      Serial.print(rpm_FL);
      Serial.print(" FR:");
      Serial.print(rpm_FR);
      Serial.print(" RL:");
      Serial.print(rpm_RL);
      Serial.print(" RR:");
      Serial.println(rpm_RR);
      last_debug_print = current_time;
    }
    
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
    set_motor(ch_FL, FL_DIR, 0); set_motor(ch_FR, FR_DIR, 0);
    set_motor(ch_RL, RL_DIR, 0); set_motor(ch_RR, RR_DIR, 0);
  }
}
