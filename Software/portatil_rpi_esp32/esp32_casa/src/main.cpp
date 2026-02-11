#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// --- OBJETOS MICRO-ROS ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ==========================================
// CONFIGURACIÓN DE PINES (DRIVER 1 - TRASERO)
// ==========================================
// Motor 0 (Trasero Izquierda)
#define M0_ENA 32
#define M0_IN1 33
#define M0_IN2 25

// Motor 1 (Trasero Derecha)
#define M1_IN3 26
#define M1_IN4 27
#define M1_ENB 14

// ==========================================
// CONFIGURACIÓN DE PINES (DRIVER 2 - FRONTAL)
// ==========================================
// Motor 2 (Frontal Izquierda) - Pines sugeridos
#define M2_ENA 15
#define M2_IN1 2
#define M2_IN2 4

// Motor 3 (Frontal Derecha) - GPIOs confiables (cambiado de 16/17 por problemas)
#define M3_IN3 18 // GPIO confiable (era 16 RX2)
#define M3_IN4 19 // GPIO confiable (era 17 TX2)
#define M3_ENB 5  // Sin cambios  

// CONFIGURACIÓN PWM (Velocidad)
const int freq = 1000;
const int resolution = 8; // 0-255
// Canales PWM (Uno para cada pin Enable)
const int ch_M0 = 0;
const int ch_M1 = 1;
const int ch_M2 = 2;
const int ch_M3 = 3;

// ==========================================
// FUNCIÓN MAGISTRAL: CONTROL INDIVIDUAL
// ==========================================
/**
 * id: 0=TrasIzq, 1=TrasDer, 2=FrontIzq, 3=FrontDer
 * speed: de -1.0 (máxima atrás) a 1.0 (máxima adelante). 0.0 es paro.
 */
void controlar_motor(int id, float speed) {
  
  // 1. Convertir float (-1 a 1) a PWM (0 a 255)
  int pwm_value = abs(speed) * 255;
  if (pwm_value > 255) pwm_value = 255;
  
  // Umbral mínimo para evitar zumbidos (deadzone)
  if (pwm_value < 20) {
    pwm_value = 0;
    speed = 0; 
  }

  // Variables para los pines a usar
  int pin_in1, pin_in2, canal_pwm;

  // Asignación de pines según el ID del motor
  switch(id) {
    case 0: pin_in1 = M0_IN1; pin_in2 = M0_IN2; canal_pwm = ch_M0; break;
    case 1: pin_in1 = M1_IN3; pin_in2 = M1_IN4; canal_pwm = ch_M1; break;
    case 2: pin_in1 = M2_IN1; pin_in2 = M2_IN2; canal_pwm = ch_M2; break;
    case 3: pin_in1 = M3_IN3; pin_in2 = M3_IN4; canal_pwm = ch_M3; break;
    default: return; // ID incorrecto
  }

  // 2. Lógica de Dirección (H-Bridge)
  if (speed > 0) {
    // Hacia ADELANTE
    digitalWrite(pin_in1, HIGH);
    digitalWrite(pin_in2, LOW);
  } 
  else if (speed < 0) {
    // Hacia ATRÁS
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, HIGH);
  } 
  else {
    // PARADO (Freno)
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, LOW);
  }

  // 3. Aplicar Velocidad al canal PWM correspondiente
  ledcWrite(canal_pwm, pwm_value);
}

// ==========================================
// CALLBACK ROS: AQUÍ HACES TUS MEZCLAS
// ==========================================
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // Recogemos los inputs del mando/nav
  float x = msg->linear.x;  // Avanzar/Retroceder
  float y = msg->linear.y;  // Desplazamiento Lateral (Strafe) -> SOLO OMNI/MECANUM
  float z = msg->angular.z; // Giro

  // ---------------------------------------------------------
  // AQUÍ ES DONDE TÚ HACES LAS COMBINACIONES
  // ---------------------------------------------------------
  // Ejemplo BÁSICO (Tipo Tanque/Diferencial) - Modifícalo para Omni
  
  // Para ruedas Mecanum estándar (configuración X), las fórmulas son:
  float speed_fl = x - y - z; // Frontal Izq (Motor 2)
  float speed_fr = x + y + z; // Frontal Der (Motor 3)
  float speed_rl = x + y - z; // Trasera Izq (Motor 0)
  float speed_rr = x - y + z; // Trasera Der (Motor 1)

  // Normalizar si alguna velocidad supera 1.0 (para no perder proporción)
  float max_val = max(abs(speed_fl), max(abs(speed_fr), max(abs(speed_rl), abs(speed_rr))));
  if (max_val > 1.0) {
    speed_fl /= max_val;
    speed_fr /= max_val;
    speed_rl /= max_val;
    speed_rr /= max_val;
  }

  // Enviamos a cada motor individualmente
  controlar_motor(2, speed_fl); // Frontal Izq
  controlar_motor(3, speed_fr); // Frontal Der
  controlar_motor(0, speed_rl); // Trasera Izq
  controlar_motor(1, speed_rr); // Trasera Der
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial); 
  
  // 1. Configurar Pines de Dirección (IN1, IN2...) como Salida
  // Driver 1
  pinMode(M0_IN1, OUTPUT); pinMode(M0_IN2, OUTPUT);
  pinMode(M1_IN3, OUTPUT); pinMode(M1_IN4, OUTPUT);
  // Driver 2
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN3, OUTPUT); pinMode(M3_IN4, OUTPUT);

  // 2. Configurar PWM (ledc) para los pines ENA/ENB
  // Driver 1
  ledcSetup(ch_M0, freq, resolution); ledcAttachPin(M0_ENA, ch_M0);
  ledcSetup(ch_M1, freq, resolution); ledcAttachPin(M1_ENB, ch_M1);
  // Driver 2
  ledcSetup(ch_M2, freq, resolution); ledcAttachPin(M2_ENA, ch_M2);
  ledcSetup(ch_M3, freq, resolution); ledcAttachPin(M3_ENB, ch_M3);

  delay(2000);

  // Configuración micro-ROS (Igual que antes)
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "microros_esp32_omni", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}