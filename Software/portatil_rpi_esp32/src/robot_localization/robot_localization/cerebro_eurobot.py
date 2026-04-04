#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String
import math
import time
from enum import Enum

try:
    from gpiozero import Button
    HARDWARE_DISPONIBLE = True
except ImportError:
    HARDWARE_DISPONIBLE = False

# ================================================================
#  PARÁMETROS GLOBALES DE COMPETICIÓN
# ================================================================
TIEMPO_TOTAL   = 100.0  # Segundos que dura la partida
TIEMPO_RETORNO =  20.0  # Segundos restantes a los que abortamos y volvemos
ZONA_ENTREGA_Y = 160.0  # Coordenada Y a partir de la cual la pieza está en el nido (cm)

# --- CONFIGURACIÓN DE VELOCIDADES ---
VEL_MAX_RECTO  = 0.35   # m/s (Velocidad máxima de avance)
VEL_MIN_RECTO  = 0.10   # m/s (Mínimo para vencer la inercia)
VEL_MAX_GIRO   = 0.50   # rad/s (Giro suave para no pasarse del ángulo)
VEL_MIN_GIRO   = 0.15   # rad/s (Giro mínimo)
VEL_EMPUJE     = 0.25   # m/s (Velocidad ciega y bruta para arrastrar piezas)

# ================================================================
#  MÁQUINA DE ESTADOS (LINEAL Y BUCLE DE GARRA)
# ================================================================
class EstadoRobot(Enum):
    ESPERANDO_TIRETTE = 0
    CALCULAR_RUTAS = 1
    
    # --- MISIÓN 1: EMPUJAR PRIMER GRUPO ---
    M1_RODEAR_PIEZAS = 2
    M1_POSICIONAR_DETRAS = 3
    M1_ENCARAR_NIDO = 4
    M1_EMPUJAR_AL_NIDO = 5
    
    # --- MISIÓN 2: BUCLE DE LA GARRA ---
    M2_EVALUAR_PIEZA = 6
    M2_IR_PUNTO_PREPARACION = 7
    M2_BAJAR_Y_ABRIR_GARRA = 8
    M2_ESPERAR_GARRA_ABIERTA = 9
    M2_APROXIMAR_PIEZA = 10
    M2_CERRAR_GARRA = 11
    M2_ESPERAR_GARRA_CERRADA = 12
    M2_IR_PUNTO_ENTREGA = 13
    M2_SOLTAR_PIEZA = 14
    M2_ESPERAR_GARRA_SOLTAR = 15
    
    # --- FIN DE PARTIDA ---
    RETORNO_EMERGENCIA = 16
    FIN_PARTIDA = 17

class CerebroEurobot(Node):
    def __init__(self):
        super().__init__("cerebro_eurobot")

        # --- Variables de Estado ---
        self.estado_actual = EstadoRobot.ESPERANDO_TIRETTE
        self.robot_pose = None
        self.equipo = "DESCONOCIDO"
        
        self.t_inicio = None
        self.t_espera = 0.0 # Cronómetro para las pausas mecánicas de la garra
        self.emergencia_activa = False
        
        # --- Variables de la Misión 2 ---
        self.lista_piezas = []
        self.indice_pieza = 0

        # --- Hardware (Cordón de arranque) ---
        if HARDWARE_DISPONIBLE:
            self.tirette = Button(pin=17, pull_up=True, bounce_time=0.05)
            self.get_logger().info("✅ Hardware GPIO detectado. Esperando cordón...")
        else:
            self.tirette = None
            self.timer_simulacion = self.create_timer(3.0, self._simular_tirette)
            self.get_logger().warn("⚠️ Modo Simulación. Arranque automático en 3s...")

        # --- Subscriptores y Publicadores (QoS = 1 para control en tiempo real) ---
        self.sub_robot = self.create_subscription(Pose2D, "/roborescue/robot_pose", self.robot_callback, 1)
        self.pub_cmd = self.create_publisher(Twist, "/roborescue/cmd_vel", 1)
        self.pub_garra = self.create_publisher(String, "/roborescue/cmd_garra", 10) # Órdenes para el servo
        self.pub_est = self.create_publisher(String, "/roborescue/robot_estado", 10)
        
        # --- Timers (El latido del programa) ---
        self.timer_fsm = self.create_timer(0.05, self.maquina_de_estados_loop) # 20 Hz
        self.timer_watchdog = self.create_timer(0.5, self.reloj_partida)       # 2 Hz

    # ================================================================
    #  CALLBACKS Y RELOJ DE PARTIDA
    # ================================================================
    def robot_callback(self, msg: Pose2D):
        self.robot_pose = msg

    def reloj_partida(self):
        if self.t_inicio is None or self.estado_actual in [EstadoRobot.FIN_PARTIDA, EstadoRobot.ESPERANDO_TIRETTE]:
            return

        tiempo_restante = TIEMPO_TOTAL - (time.time() - self.t_inicio)
        
        msg_est = String()
        msg_est.data = f"{self.estado_actual.name} | {max(tiempo_restante, 0):.1f}s"
        self.pub_est.publish(msg_est)

        if tiempo_restante <= TIEMPO_RETORNO and not self.emergencia_activa:
            self.get_logger().warn(f"🚨 {tiempo_restante:.0f}s RESTANTES. ¡ABORTANDO Y VOLVIENDO A BASE!")
            self.emergencia_activa = True
            self.estado_actual = EstadoRobot.RETORNO_EMERGENCIA

        if tiempo_restante <= 0.0:
            self.get_logger().warn("⏰ ¡FIN DEL TIEMPO! Apagando motores.")
            self.estado_actual = EstadoRobot.FIN_PARTIDA

    def _simular_tirette(self):
        if self.estado_actual == EstadoRobot.ESPERANDO_TIRETTE and self.robot_pose is not None:
            self.get_logger().info("¡Tirette virtual activado!")
            self.t_inicio = time.time()
            self.estado_actual = EstadoRobot.CALCULAR_RUTAS
            self.timer_simulacion.cancel()

    # ================================================================
    #  GENERACIÓN DE RUTAS (Variables explícitas y Listas)
    # ================================================================
    def calcular_waypoints(self):
        self.equipo = "AZUL" if self.robot_pose.x > 150.0 else "AMARILLO"
        self.get_logger().info(f"🔵🟡 Equipo detectado: {self.equipo}")

        if self.equipo == "AMARILLO":
            # --- MISIÓN 1 (Empujar a lo bruto) ---
            self.m1_rodeo_x = 55.0
            self.m1_rodeo_y = 80.0
            self.m1_pos_x   = 12.0
            self.m1_pos_y   = 75.0
            
            # --- MISIÓN 2 (Bucle de 4 Piezas con la garra) ---
            self.lista_piezas = [
                # Pieza 1 -> Al Nido
                {"prep_x": 15.0, "prep_y": 60.0, "cap_x": 15.0, "cap_y": 70.0, "ent_x": 40.0, "ent_y": 185.0},
                # Pieza 2 -> Al Nido
                {"prep_x": 15.0, "prep_y": 60.0, "cap_x": 15.0, "cap_y": 75.0, "ent_x": 60.0, "ent_y": 185.0},
                # Pieza 3 -> A la Despensa
                {"prep_x": 15.0, "prep_y": 60.0, "cap_x": 15.0, "cap_y": 80.0, "ent_x": 20.0, "ent_y": 120.0},
                # Pieza 4 -> A la Despensa
                {"prep_x": 15.0, "prep_y": 60.0, "cap_x": 15.0, "cap_y": 85.0, "ent_x": 20.0, "ent_y": 100.0}
            ]
            
            # --- ZONA SEGURA ---
            self.park_x = 12.0
            self.park_y = 160.0
            
        else: # EQUIPO AZUL (Rellenar espejo más adelante)
            self.m1_rodeo_x = 245.0; self.m1_rodeo_y = 80.0
            self.m1_pos_x   = 288.0; self.m1_pos_y   = 75.0
            self.lista_piezas = [
                {"prep_x": 285.0, "prep_y": 60.0, "cap_x": 285.0, "cap_y": 70.0, "ent_x": 260.0, "ent_y": 185.0}
            ]
            self.park_x     = 288.0; self.park_y     = 160.0

    # ================================================================
    #  HERRAMIENTAS DE MOVIMIENTO ROBUSTO
    # ================================================================
    def normalizar_angulo(self, angulo):
        """Mantiene el ángulo siempre entre -180 y 180 grados (Lógica humana)"""
        while angulo > 180.0:
            angulo -= 360.0
        while angulo < -180.0:
            angulo += 360.0  # Bug crítico arreglado
        return angulo
    
    def girar_absoluto(self, target_theta, tolerancia=5.0):
        if self.robot_pose is None: return False
        
        error_theta = target_theta - self.robot_pose.theta
        error_theta = self.normalizar_angulo(error_theta)

        if abs(error_theta) < tolerancia:
            self.parar_motores()
            return True
            
        w = max(VEL_MIN_GIRO, min(VEL_MAX_GIRO, abs(0.015 * error_theta)))
        w = w if error_theta > 0 else -w
        self.enviar_velocidad(vx=0.0, vy=0.0, w=w)
        return False

    def avanzar_recto(self, target_x, target_y, tolerancia=8.0):
        if self.robot_pose is None: return False
        
        error_x = target_x - self.robot_pose.x
        error_y = target_y - self.robot_pose.y
        distancia = math.hypot(error_x, error_y)
        
        if distancia < tolerancia:
            self.parar_motores()
            return True
        
        # Corrección angular para no desviarse
        angulo_objetivo = math.degrees(math.atan2(error_y, error_x))
        error_angular = angulo_objetivo - self.robot_pose.theta
        error_angular = self.normalizar_angulo(error_angular)
        w_correccion = 0.015 * error_angular 
        
        # Avance lineal
        theta_rad = math.radians(self.robot_pose.theta)
        error_local_x = error_x * math.cos(theta_rad) + error_y * math.sin(theta_rad)
        direccion = 1.0 if error_local_x > 0 else -1.0
        vx = direccion * max(VEL_MIN_RECTO, min(VEL_MAX_RECTO, abs(0.02 * distancia)))
        
        self.enviar_velocidad(vx=vx, vy=0.0, w=w_correccion)
        return False

    def ir_a_punto_como_tanque(self, target_x, target_y):
        if self.robot_pose is None: return False
        
        # Parche anti-oscilación (Danza de la muerte solucionada)
        distancia = math.hypot(target_x - self.robot_pose.x, target_y - self.robot_pose.y)
        if distancia < 8.0:
            self.parar_motores()
            return True

        angulo_objetivo = math.degrees(math.atan2(target_y - self.robot_pose.y, target_x - self.robot_pose.x))
        
        if not self.girar_absoluto(angulo_objetivo, tolerancia=2.0): # 2º de tolerancia para encarar
            return False 
        
        if not self.avanzar_recto(target_x, target_y, tolerancia=8.0):
            return False 
            
        return True 

    def enviar_velocidad(self, vx, vy, w):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)

    def parar_motores(self):
        self.enviar_velocidad(0.0, 0.0, 0.0)

    # ================================================================
    #  EL GUION PRINCIPAL DE LA PARTIDA (FSM)
    # ================================================================
    def maquina_de_estados_loop(self):
        if self.estado_actual == EstadoRobot.FIN_PARTIDA:
            self.parar_motores()
            return

        # 0. INICIO
        if self.estado_actual == EstadoRobot.ESPERANDO_TIRETTE:
            self.parar_motores()
            if HARDWARE_DISPONIBLE and self.tirette.is_pressed and self.robot_pose is not None:
                self.t_inicio = time.time()
                self.estado_actual = EstadoRobot.CALCULAR_RUTAS

        elif self.estado_actual == EstadoRobot.CALCULAR_RUTAS:
            self.calcular_waypoints()
            self.estado_actual = EstadoRobot.M1_RODEAR_PIEZAS

        # ==========================================================
        # MISIÓN 1: EMPUJAR PRIMER GRUPO (Lineal)
        # ==========================================================
        elif self.estado_actual == EstadoRobot.M1_RODEAR_PIEZAS:
            if self.ir_a_punto_como_tanque(self.m1_rodeo_x, self.m1_rodeo_y):
                self.estado_actual = EstadoRobot.M1_POSICIONAR_DETRAS

        elif self.estado_actual == EstadoRobot.M1_POSICIONAR_DETRAS:
            if self.ir_a_punto_como_tanque(self.m1_pos_x, self.m1_pos_y):
                self.estado_actual = EstadoRobot.M1_ENCARAR_NIDO

        elif self.estado_actual == EstadoRobot.M1_ENCARAR_NIDO:
            if self.girar_absoluto(90.0):
                self.get_logger().info("¡Alineado! Iniciando empuje de excavadora...")
                self.estado_actual = EstadoRobot.M1_EMPUJAR_AL_NIDO

        elif self.estado_actual == EstadoRobot.M1_EMPUJAR_AL_NIDO:
            if self.robot_pose.y >= ZONA_ENTREGA_Y:
                self.parar_motores()
                self.get_logger().info("✅ Misión 1 completada. Pasando a garra.")
                # Saltamos al jefe de obra de la Misión 2
                self.estado_actual = EstadoRobot.M2_EVALUAR_PIEZA
            else:
                self.enviar_velocidad(vx=VEL_EMPUJE, vy=0.0, w=0.0)

        # ==========================================================
        # MISIÓN 2: BUCLE DE LA GARRA (Lista de Piezas)
        # ==========================================================
        elif self.estado_actual == EstadoRobot.M2_EVALUAR_PIEZA:
            if self.indice_pieza < len(self.lista_piezas):
                self.get_logger().info(f"Yendo a por la pieza {self.indice_pieza + 1} de {len(self.lista_piezas)}...")
                self.estado_actual = EstadoRobot.M2_IR_PUNTO_PREPARACION
            else:
                self.get_logger().info("✅ ¡Todas las piezas recogidas! Volviendo a base.")
                self.estado_actual = EstadoRobot.RETORNO_EMERGENCIA

        elif self.estado_actual == EstadoRobot.M2_IR_PUNTO_PREPARACION:
            pieza = self.lista_piezas[self.indice_pieza]
            if self.ir_a_punto_como_tanque(pieza["prep_x"], pieza["prep_y"]):
                self.parar_motores()
                self.estado_actual = EstadoRobot.M2_BAJAR_Y_ABRIR_GARRA

        elif self.estado_actual == EstadoRobot.M2_BAJAR_Y_ABRIR_GARRA:
            self.get_logger().info("Preparando Garra: Mandando señal de ABRIR...")
            msg = String()
            msg.data = "BAJAR_ABRIR"
            self.pub_garra.publish(msg)
            
            self.t_espera = time.time()
            self.estado_actual = EstadoRobot.M2_ESPERAR_GARRA_ABIERTA

        elif self.estado_actual == EstadoRobot.M2_ESPERAR_GARRA_ABIERTA:
            if time.time() - self.t_espera > 1.0:
                self.estado_actual = EstadoRobot.M2_APROXIMAR_PIEZA

        elif self.estado_actual == EstadoRobot.M2_APROXIMAR_PIEZA:
            pieza = self.lista_piezas[self.indice_pieza]
            if self.ir_a_punto_como_tanque(pieza["cap_x"], pieza["cap_y"]):
                self.parar_motores()
                self.estado_actual = EstadoRobot.M2_CERRAR_GARRA

        elif self.estado_actual == EstadoRobot.M2_CERRAR_GARRA:
            self.get_logger().info("¡Pieza alcanzada! Mandando señal de CERRAR...")
            msg = String()
            msg.data = "CERRAR"
            self.pub_garra.publish(msg)
            
            self.t_espera = time.time()
            self.estado_actual = EstadoRobot.M2_ESPERAR_GARRA_CERRADA

        elif self.estado_actual == EstadoRobot.M2_ESPERAR_GARRA_CERRADA:
            if time.time() - self.t_espera > 1.0:
                self.estado_actual = EstadoRobot.M2_IR_PUNTO_ENTREGA

        elif self.estado_actual == EstadoRobot.M2_IR_PUNTO_ENTREGA:
            pieza = self.lista_piezas[self.indice_pieza]
            if self.ir_a_punto_como_tanque(pieza["ent_x"], pieza["ent_y"]):
                self.parar_motores()
                self.estado_actual = EstadoRobot.M2_SOLTAR_PIEZA

        elif self.estado_actual == EstadoRobot.M2_SOLTAR_PIEZA:
            self.get_logger().info("Zona de entrega alcanzada. Soltando pieza...")
            msg = String()
            msg.data = "SUBIR_ABRIR" 
            self.pub_garra.publish(msg)
            
            self.t_espera = time.time()
            self.estado_actual = EstadoRobot.M2_ESPERAR_GARRA_SOLTAR

        elif self.estado_actual == EstadoRobot.M2_ESPERAR_GARRA_SOLTAR:
            if time.time() - self.t_espera > 1.0:
                self.get_logger().info("Pieza soltada con éxito.")
                self.indice_pieza += 1  # Tachamos la pieza de la lista
                self.estado_actual = EstadoRobot.M2_EVALUAR_PIEZA # Volvemos al inicio del bucle

        # ==========================================================
        # RETORNO A BASE / FIN
        # ==========================================================
        elif self.estado_actual == EstadoRobot.RETORNO_EMERGENCIA:
            if self.ir_a_punto_como_tanque(self.park_x, self.park_y):
                self.get_logger().info("🏁 Robot aparcado. Fin de operaciones.")
                self.estado_actual = EstadoRobot.FIN_PARTIDA

def main(args=None):
    rclpy.init(args=args)
    nodo = CerebroEurobot()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Cierre forzado. Apagando motores...")
    finally:
        nodo.parar_motores()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()