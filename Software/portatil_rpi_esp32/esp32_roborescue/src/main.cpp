#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ================================================================
//  ZONA DE CONFIGURACIÓN DEL CHASIS Y CINEMÁTICA
// ================================================================

// Parámetros PID individuales por motor (Overshoot=1.5%, ts=0.52s)
const float Kp_FL = 218.9; const float Ki_FL = 1488.0;
const float Kp_FR = 129.4; const float Ki_FR = 1130.0;
const float Kp_RL = 200.8; const float Ki_RL = 1435.0;
const float Kp_RR = 159.7; const float Ki_RR = 1078.0;

// Compensación de fricción
const int base_FL = 89; const int base_FR = 47;
const int base_RL = 84; const int base_RR = 78;

// Geometría del robot (en metros)
const float LX = 0.130;  
const float LY = 0.075;  
const float K_GEOMETRIA = LX + LY; // 0.205

// ================================================================
//  PINES Y CONFIGURACIÓN HARDWARE
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

// Canales PWM clásicos (v2.x)
const int ch_FL = 0;
const int ch_FR = 1;
const int ch_RL = 2;
const int ch_RR = 3;

const int freq = 1000;
const int resolution = 8;
const float RADIO_RUEDA = 0.0325; 
const int COUNTS_PER_REV = (11 * 34) * 4; 
const float rads_por_cuenta = (2.0f * PI) / (float)COUNTS_PER_REV;
const unsigned long ventana_us = 40000; // 40ms control loop

// ================================================================
//  VARIABLES DE ESTADO (PID y Encoders)
// ================================================================
volatile long cont_FL = 0, cont_FR = 0, cont_RL = 0, cont_RR = 0;
long prev_FL = 0, prev_FR = 0, prev_RL = 0, prev_RR = 0;
unsigned long lastWindowUs = 0;
double errSum_FL = 0, errSum_FR = 0, errSum_RL = 0, errSum_RR = 0;

// Referencias calculadas por la cinemática para cada rueda
float ref_FL = 0.0, ref_FR = 0.0, ref_RL = 0.0, ref_RR = 0.0;

// Comandos globales recibidos de la Raspberry
float comando_Vx = 0.0;
float comando_Vy = 0.0;
float comando_W  = 0.0;

// ================================================================
//  VARIABLES MICRO-ROS
// ================================================================
rcl_subscription_t subscriber;
rcl_publisher_t publisher_encoder_vel;
geometry_msgs__msg__Twist msg;
std_msgs__msg__Float32MultiArray encoder_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;

unsigned long last_msg_time = 0;

// ================================================================
//  INTERRUPCIONES DE ENCODERS
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
//  CALLBACK MICRO-ROS (La Oreja del Robot)
// ================================================================
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_msg_time = millis(); // Reseteamos el perro guardián
  
  // Guardamos las velocidades ordenadas por la Raspberry
  comando_Vx = msg->linear.x;
  comando_Vy = msg->linear.y;
  comando_W  = msg->angular.z;
}

// ================================================================
//  FUNCIÓN DE CINEMÁTICA MECANUM
// ================================================================
void calcular_cinematica(float Vx, float Vy, float W) {
  ref_FL = Vx + Vy - (W * K_GEOMETRIA);
  ref_FR = Vx - Vy + (W * K_GEOMETRIA);
  ref_RL = Vx - Vy - (W * K_GEOMETRIA);
  ref_RR = Vx + Vy + (W * K_GEOMETRIA);
}

// ================================================================
//  FUNCIÓN PID BIDIRECCIONAL
// ================================================================
int calcular_pwm_rueda(float v_deseada, float v_actual, double &memoria_integral, int pwm_base_motor, float kp, float ki) {
  if (v_deseada == 0.0) { memoria_integral = 0; return 0; }
  double dt = (double)ventana_us / 1000000.0;
  double error = v_deseada - v_actual;
  memoria_integral += (error * dt);

  double limite_integral = (255.0 - pwm_base_motor) / ki; 
  if (memoria_integral > limite_integral) memoria_integral = limite_integral;
  else if (memoria_integral < -limite_integral) memoria_integral = -limite_integral;

  double Output = (kp * error) + (ki * memoria_integral);
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
//  SETUP
// ================================================================
void setup() {
  // 1. Configurar Hardware Motores
  pinMode(FL_DIR, OUTPUT); pinMode(RL_DIR, OUTPUT);
  pinMode(FR_DIR, OUTPUT); pinMode(RR_DIR, OUTPUT);
  
  // Sintaxis clásica para PWM
  ledcSetup(ch_FL, freq, resolution); ledcAttachPin(FL_PWM, ch_FL);
  ledcSetup(ch_FR, freq, resolution); ledcAttachPin(FR_PWM, ch_FR);
  ledcSetup(ch_RL, freq, resolution); ledcAttachPin(RL_PWM, ch_RL);
  ledcSetup(ch_RR, freq, resolution); ledcAttachPin(RR_PWM, ch_RR);

  // Asegurar PWM a 0 en el arranque
  ledcWrite(ch_FL, 0); ledcWrite(ch_FR, 0);
  ledcWrite(ch_RL, 0); ledcWrite(ch_RR, 0);

  // 2. Configurar Encoders
  pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(RL_ENC_A, INPUT_PULLUP); pinMode(RL_ENC_B, INPUT_PULLUP);
  pinMode(RR_ENC_A, INPUT_PULLUP); pinMode(RR_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isr_FL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_ENC_B), isr_FL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isr_FR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_B), isr_FR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isr_RL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_B), isr_RL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isr_RR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_B), isr_RR_B, CHANGE);

  // 3. Configurar Micro-ROS
  Serial.begin(115200);
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 17);
  
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, "esp32_mecanum", "roborescue", &support);
  
  // Suscriptor de Comandos
  rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");
  
  // Publicador de Odometría (4 floats: vel m/s de cada rueda)
  encoder_msg.data.capacity = 4;
  encoder_msg.data.size = 4;
  encoder_msg.data.data = (float*) malloc(encoder_msg.data.capacity * sizeof(float));
  
  rclc_publisher_init_default(
    &publisher_encoder_vel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "encoder_velocities");
      
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  
  lastWindowUs = micros();
  last_msg_time = millis();
}

// ================================================================
//  LOOP MAESTRO
// ================================================================
void loop() {
  // 1. Escuchar a la Raspberry Pi
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // 2. Perro Guardián (Si falla la conexión por > 500ms, frenar)
  if (millis() - last_msg_time > 500) {
    comando_Vx = 0.0;
    comando_Vy = 0.0;
    comando_W  = 0.0;
  }

  // 3. Bucle de Control a 40ms
  unsigned long ahora_us = micros();
  if ((ahora_us - lastWindowUs) >= ventana_us) {
    unsigned long ventana_real_us = ahora_us - lastWindowUs;
    lastWindowUs = ahora_us;

    // A. Traducir comando a velocidades de rueda
    calcular_cinematica(comando_Vx, comando_Vy, comando_W);

    // B. Leer Encoders de forma segura
    portENTER_CRITICAL(&mux);
    long current_FL = cont_FL; long current_FR = cont_FR;
    long current_RL = cont_RL; long current_RR = cont_RR;
    portEXIT_CRITICAL(&mux);

    long delta_FL = current_FL - prev_FL; long delta_FR = current_FR - prev_FR;
    long delta_RL = current_RL - prev_RL; long delta_RR = current_RR - prev_RR;

    prev_FL = current_FL; prev_FR = current_FR;
    prev_RL = current_RL; prev_RR = current_RR;

    // C. Calcular velocidades actuales en m/s
    float rads_FL = -(((float)delta_FL * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;
    float rads_FR = (((float)delta_FR * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;
    float rads_RL = -(((float)delta_RL * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;
    float rads_RR = (((float)delta_RR * 1000000.0f) / (float)ventana_real_us) * rads_por_cuenta;

    float v_FL = rads_FL * RADIO_RUEDA; float v_FR = rads_FR * RADIO_RUEDA;
    float v_RL = rads_RL * RADIO_RUEDA; float v_RR = rads_RR * RADIO_RUEDA;

    // D. Calcular PIDs
    int pwm_FL = calcular_pwm_rueda(ref_FL, v_FL, errSum_FL, base_FL, Kp_FL, Ki_FL);
    int pwm_FR = calcular_pwm_rueda(ref_FR, v_FR, errSum_FR, base_FR, Kp_FR, Ki_FR);
    int pwm_RL = calcular_pwm_rueda(ref_RL, v_RL, errSum_RL, base_RL, Kp_RL, Ki_RL);
    int pwm_RR = calcular_pwm_rueda(ref_RR, v_RR, errSum_RR, base_RR, Kp_RR, Ki_RR);

    // E. Mover hardware (usando la sintaxis clásica de canales)
    digitalWrite(FL_DIR, (pwm_FL >= 0) ? LOW : HIGH); ledcWrite(ch_FL, abs(pwm_FL));
    digitalWrite(FR_DIR, (pwm_FR >= 0) ? HIGH : LOW); ledcWrite(ch_FR, abs(pwm_FR));
    digitalWrite(RL_DIR, (pwm_RL >= 0) ? LOW : HIGH); ledcWrite(ch_RL, abs(pwm_RL));
    digitalWrite(RR_DIR, (pwm_RR >= 0) ? HIGH : LOW); ledcWrite(ch_RR, abs(pwm_RR));

    // F. Informar a la Raspberry Pi
    encoder_msg.data.data[0] = v_FL;
    encoder_msg.data.data[1] = v_FR;
    encoder_msg.data.data[2] = v_RL;
    encoder_msg.data.data[3] = v_RR;
    rcl_publish(&publisher_encoder_vel, &encoder_msg, NULL);
  }
}