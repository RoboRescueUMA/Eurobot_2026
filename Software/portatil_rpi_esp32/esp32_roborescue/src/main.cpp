#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// ================================================================
//  CONFIGURACIÓN DE PINES (CORREGIDA - NO USAR 34/35/36/39)
// ================================================================

// DRIVER 1 (TRASERO)
// Motor 1: Trasero Izquierda (RL)
#define RL_PWM 25   
#define RL_DIR 26  

// Motor 2: Trasero Derecha (RR)
#define RR_PWM 27   
#define RR_DIR 14   

// DRIVER 2 (DELANTERO)
// Motor 3: Frontal Izquierda (FL) -> CAMBIADO (34/35 eran solo input)
#define FL_PWM 18  // Antes 34
#define FL_DIR 19  // Antes 35

// Motor 4: Frontal Derecha (FR)
#define FR_PWM 32
#define FR_DIR 33

// Configuración PWM
const int freq = 1000;
const int resolution = 8;
const int ch_RL = 0; 
const int ch_RR = 1;
const int ch_FL = 2;
const int ch_FR = 3;

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
void set_motor(int channel, int dir_pin, float speed) {
  // Limitar
  if (speed > 1.0) speed = 1.0;
  if (speed < -1.0) speed = -1.0;

  int pwm = abs(speed) * 255;
  
  // ZONA MUERTA (Importante para no quemar motores con poco voltaje)
  if (pwm < 25) pwm = 0;

  // DIRECCIÓN (Ajustar HIGH/LOW según tu cableado si va al revés)
  if (speed > 0) {
    digitalWrite(dir_pin, HIGH);
  } else {
    digitalWrite(dir_pin, LOW);
  }
  
  ledcWrite(channel, pwm);
}

// ================================================================
//  CALLBACK CINEMÁTICA MECANUM
// ================================================================
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_msg_time = millis(); // Resetear watchdog

  float x = msg->linear.x; 
  float y = msg->linear.y; 
  float z = msg->angular.z; 

  // Cinemática X-Drive
  float fl = x + y + z;
  float fr = x - y - z;
  float rl = x - y + z;
  float rr = x + y - z;

  // Normalizar
  float max_val = max(abs(fl), max(abs(fr), max(abs(rl), abs(rr))));
  if (max_val > 1.0) {
    fl /= max_val; fr /= max_val; rl /= max_val; rr /= max_val;
  }

  set_motor(ch_FL, FL_DIR, fl);
  set_motor(ch_FR, FR_DIR, fr);
  set_motor(ch_RL, RL_DIR, rl);
  set_motor(ch_RR, RR_DIR, rr);
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
