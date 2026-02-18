#include <Arduino.h>
#include <math.h>  // Para sqrt() en cálculo de L
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

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

// Configuración PWM
const int freq = 1000;
const int resolution = 8;
const int ch_RL = 0; 
const int ch_RR = 1;
const int ch_FL = 2;
const int ch_FR = 3;

// ================================================================
//  GEOMETRÍA DEL ROBOT (para cinemática)
// ================================================================
// Distancia del centro del robot a las ruedas (en metros)
const float Lx = 0.10;  // TODO: Medir distancia centro → rueda (eje X)
const float Ly = 0.14;  // TODO: Medir distancia centro → rueda (eje Y)
// Para X-Drive: distancia efectiva = diagonal
const float L = Lx + Ly; 

// Variables ROS
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
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
    
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  
  Serial.println("========================================");
  Serial.println("   ✅ MICRO-ROS INICIADO CORRECTAMENTE");
  Serial.println("========================================");
  Serial.print("   Domain ID: 17\n");
  Serial.print("   Namespace: roborescue\n");
  Serial.print("   Esperando en: /roborescue/cmd_vel\n");
  Serial.println("========================================\n");
}

void loop() {
  // Ejecutar ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // SAFETY: Si no llegan comandos en 0.5s, parar todo
  if (millis() - last_msg_time > 500) {
    set_motor(ch_FL, FL_DIR, 0); set_motor(ch_FR, FR_DIR, 0);
    set_motor(ch_RL, RL_DIR, 0); set_motor(ch_RR, RR_DIR, 0);
  }
}
