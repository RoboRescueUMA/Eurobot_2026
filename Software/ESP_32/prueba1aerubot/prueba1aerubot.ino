// ================ PINES Y CANALES MOTORES =================

#define In1_fl 22 // Front left wheel
#define In2_fl 21
#define PWM_fl 0 // Canal

#define In1_fr 17 // Front right wheel
#define In2_fr 16
#define PWM_fr 1 // Canal

#define In1_br 19 // Back right wheel
#define In2_br 18   
#define PWM_br 2 // Canal

#define In1_bl 25 // Back left wheel
#define In2_bl 23
#define PWM_bl 3 // Canal

// ====================== PINES ENCODERS ====================

#define EN_FL_A 26// Front left wheel
#define EN_FL_B 27

#define EN_FR_A 34 // Front right wheel
#define EN_FR_B 35

#define EN_BR_A 32// Back right wheel
#define EN_BR_B 33

#define EN_BL_A 36 // Back left wheel
#define EN_BL_B 39

// ===================== PARAMETROS ROBOT =====================
const float ancho = 0.2; //(m)
const float largo = 0.153; //(m)
const float R = (ancho + largo) / 2.0;

// ===================== PWM =====================
const int pwm_max = 255;
const int frecuencia = 8000;
const int resolucion = 8;

// ===================== ENCODERS =====================
const int MAX_RPM = 350; // Máximo de revoluciones por minuto
const int TICKS_REV = 360; // CHANGE FOR THE ACTUAL VALUE

volatile long ticks_fl = 0;
volatile long ticks_fr = 0;
volatile long ticks_br = 0;
volatile long ticks_bl = 0;

// ===================== PID STRUCT =====================
struct PID { // PArámetros para corrigir errores en velocidad
  float P, I, D; // Proporcional (Reaccion a errores puntuales), integral (Reaccion a errores persistentes) y derivative (Rapidez de cambio de error)
  float prev_error;
  float integral; // error - prev_error
};

PID pid_fl = {1.25, 0.01, 0.05, 0, 0};
PID pid_fr = {1.25, 0.01, 0.05, 0, 0};
PID pid_br = {1.25, 0.01, 0.05, 0, 0};
PID pid_bl = {1.25, 0.01, 0.05, 0, 0};


// ===================== INTERRUPCIONES ENCODERS =====================
void IRAM_ATTR enc_fl() {
  if (digitalRead(EN_FL_A) == digitalRead(EN_FL_B)) ticks_fl++;
  else ticks_fl--;
}

void IRAM_ATTR enc_fr() {
  if (digitalRead(EN_FR_A) == digitalRead(EN_FR_B)) ticks_fr++;
  else ticks_fr--;
}

void IRAM_ATTR enc_br() {
  if (digitalRead(EN_BR_A) == digitalRead(EN_BR_B)) ticks_br++;
  else ticks_br--;
}

void IRAM_ATTR enc_bl() {
  if (digitalRead(EN_BL_A) == digitalRead(EN_BL_B)) ticks_bl++;
  else ticks_bl--;
}

// ===================== FUNCIONES NECESARIAS =====================
float ticks_to_rpm  (volatile long &ticks, float time)
{
  noInterrupts();
  long t = ticks;
  ticks = 0;
  interrupts();
  return (t / (float)TICKS_REV) * (60.0 / time); // vueltas por minuto
}

// -========================== FUNCION PID ==================================
float calcular_PID(PID &pid, float objetivo, float real, float dt) {
  float error = objetivo - real;
  pid.integral += error * dt;
  float derivative = (error - pid.prev_error) / dt;
  pid.prev_error = error;

  return pid.P * error + pid.I * pid.integral + pid.D * derivative;
}


// =========================== VOIDS CONTROL RUEDAS =========================

int control_rueda(PID &pid, volatile long &ticks, float vel_norm, float dt, int rpm_obj)
{
  float rpm_real = ticks_to_rpm(ticks, dt);
  float rpm_rueda = fabs(vel_norm) * MAX_RPM;
  float salida = calcular_PID(pid, rpm_obj, rpm_real, dt);
  return constrain((int)salida, 0, pwm_max);
}

void wheel_forward(int pin1, int pin2, int velocidad) {
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
  ledcWrite(pin1, velocidad);
}

void wheel_backwards(int pin1, int pin2, int velocidad) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  ledcWrite(pin2, velocidad);
}

void move_wheel (int pin1, int pin2, int pwm, int vel, float vf)
{
  if (vf>0){
    wheel_forward (pin1, pin2, vel);
  }else if (vf<0){
    wheel_backwards(pin1, pin2, vel);
  }else{wheel_forward (pin1, pin2, 0);}
}

//================== VOID MOVIMIENTO VECTORIAL =====================
// x --> Movimiento lateral (right(+) and left (-))
// y --> Movimiento vertical (Forward(+) and Backwardws(-))
// omega --> Rotación (Counterclockwise(+) and Clockwise(-))

void move (float x, float y, float omega)
{
  // Ecuaciones por cada rueda
  float Vfl = y + x + omega * R;
  float Vfr = y - x - omega * R;
  float Vbr = y + x - omega * R; 
  float Vbl = y - x + omega * R;

  // Normalizamos (Para que ninguna rueda supere el máximo, miramos cual es la mayor y escalamos)
  float maxV = fabs(Vfl);
  maxV = max(maxV, fabs(Vfr));
  maxV = max(maxV, fabs(Vbr));
  maxV = max(maxV, fabs(Vbl));

  if (maxV > 1.0) {
    Vfl /= maxV;
    Vfr /= maxV;
    Vbr /= maxV;
    Vbl /= maxV;
  }

  // Calculamos dt
  static unsigned long lastTime = millis(); // Última vez que se ejecutó move()
  float dt = (millis() - lastTime) / 1000.0; // Tiempo transcurrido desde lastTime
  if (dt <= 0) return; // Evitamnos errores por dividir entre 0 o negativo 



  // Pasamos a PWM y controlamos el posible error
  int pwm_fl = control_rueda(pid_fl, ticks_fl, Vfl, dt, In1_fl);
  int pwm_fr = control_rueda(pid_fr, ticks_fr, Vfr, dt, In1_fr);
  int pwm_br = control_rueda(pid_br, ticks_br, Vbr, dt, In1_br);
  int pwm_bl = control_rueda(pid_bl, ticks_bl, Vbl, dt, In1_bl);

  // ---- MOVEMOS LAS RUEDAS ----
  move_wheel (In1_fl, In2_fl, PWM_fl, pwm_fl, Vfl); // Rueda frontal izquierda
  move_wheel (In1_fr, In2_fr, PWM_fr, pwm_fr, Vfr); // Rueda frontal derecha
  move_wheel (In1_br, In2_br, PWM_br, pwm_br, Vbr); // Rueda trasera derecha
  move_wheel (In1_bl, In2_bl, PWM_bl, pwm_bl, Vbl); // Rueda trasera izquierda
 
}

void setup() {
  pinMode(In1_fl, OUTPUT); pinMode(In2_fl, OUTPUT);
  pinMode(In1_fr, OUTPUT); pinMode(In2_fr, OUTPUT);
  pinMode(In1_br, OUTPUT); pinMode(In2_br, OUTPUT);
  pinMode(In1_bl, OUTPUT); pinMode(In2_bl, OUTPUT);

  pinMode(EN_FL_A, INPUT_PULLUP); pinMode(EN_FL_B, INPUT_PULLUP);
  pinMode(EN_FR_A, INPUT_PULLUP); pinMode(EN_FR_B, INPUT_PULLUP);
  pinMode(EN_BR_A, INPUT_PULLUP); pinMode(EN_BR_B, INPUT_PULLUP);
  pinMode(EN_BL_A, INPUT_PULLUP); pinMode(EN_BL_B, INPUT_PULLUP);

  ledcAttach(In1_fl, frecuencia, resolucion);
  ledcAttach(In2_fl, frecuencia, resolucion);
  ledcAttach(In1_fr, frecuencia, resolucion);
  ledcAttach(In2_fr, frecuencia, resolucion);
  ledcAttach(In1_br, frecuencia, resolucion);
  ledcAttach(In2_br, frecuencia, resolucion);
  ledcAttach(In1_bl, frecuencia, resolucion);
  ledcAttach(In2_bl, frecuencia, resolucion);

  attachInterrupt(digitalPinToInterrupt(EN_FL_A), enc_fl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EN_FR_A), enc_fr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EN_BR_A), enc_br, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EN_BL_A), enc_bl, CHANGE);

  
  
}

  Serial.begin(115200);
  Serial.println("Robot Inicializadose...");
}

void loop(){
  move(0.5, 0, 0);
}

