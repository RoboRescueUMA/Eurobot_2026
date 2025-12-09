#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- TUS PINES EXACTOS ---
// Motor A (Izquierda - ENA)
#define ENA 32
#define IN1 33
#define IN2 25

// Motor B (Derecha - ENB)
#define IN3 26
#define IN4 27
#define ENB 14

// Función para mover motores
void move_motors(float x, float z) {
  // Si la velocidad es muy baja, paramos todo
  if (abs(x) < 0.1 && abs(z) < 0.1) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    // Opcional: Cortar energía también
    digitalWrite(ENA, LOW); 
    digitalWrite(ENB, LOW);
    return;
  }

  // ACTIVAMOS LOS MOTORES (Velocidad Máxima)
  // Nota: Más adelante usaremos PWM aquí para regular velocidad
  digitalWrite(ENA, HIGH); 
  digitalWrite(ENB, HIGH);

  // Lógica de movimiento básica
  // AVANZAR
  if (x > 0.1) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  }
  // RETROCEDER
  else if (x < -0.1) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  }
  // GIRO IZQUIERDA (Sobre su eje)
  else if (z > 0.1) { 
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // Motor A atrás
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Motor B adelante
  }
  // GIRO DERECHA (Sobre su eje)
  else if (z < -0.1) { 
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Motor A adelante
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // Motor B atrás
  }
}

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  move_motors(msg->linear.x, msg->angular.z);
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial); 
  
  // Configurar TODOS los pines como salida
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Inicialmente parados
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  digitalWrite(ENA, LOW); digitalWrite(ENB, LOW);

  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "microros_esp32_node", "", &support);

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