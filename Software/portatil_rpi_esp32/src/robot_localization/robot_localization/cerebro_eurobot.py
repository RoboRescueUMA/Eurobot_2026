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
ZONA_ENTREGA_Y = 185.0  # Coordenada Y a partir de la cual la pieza está en el nido (cm)

# ================================================================
#  MÁQUINA DE ESTADOS
# ================================================================
class EstadoRobot(Enum):
    ESPERANDO_TIRETTE = 0
    CALCULAR_RUTAS = 1
    RODEAR_PIEZAS = 2
    POSICIONAR_DETRAS = 3
    EMPUJAR_AL_NIDO = 4
    EVALUAR_SIGUIENTE = 5
    RETORNO_EMERGENCIA = 6
    FIN_PARTIDA = 7

class CerebroEurobot(Node):
    def __init__(self):
        super().__init__("cerebro_eurobot")

        # --- Variables de Estado ---
        self.estado_actual = EstadoRobot.ESPERANDO_TIRETTE
        self.robot_pose = None
        self.equipo = "DESCONOCIDO"
        
        self.t_inicio = None
        self.emergencia_activa = False
        
        self.rutas_mision = []
        self.mision_actual = 0

        # --- Hardware (Cordón de arranque) ---
        if HARDWARE_DISPONIBLE:
            self.tirette = Button(pin=17, pull_up=True, bounce_time=0.05)
            self.get_logger().info("✅ Hardware GPIO detectado. Esperando cordón...")
        else:
            self.tirette = None
            self.timer_simulacion = self.create_timer(3.0, self._simular_tirette)
            self.get_logger().warn("⚠️ Modo Simulación. Arranque automático en 3s...")

        # --- Subscriptores y Publicadores ---
        self.sub_robot = self.create_subscription(Pose2D, "/roborescue/robot_pose", self.robot_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, "/roborescue/cmd_vel", 10)
        self.pub_est = self.create_publisher(String, "/roborescue/robot_estado", 10)
        
        # --- Timers (El latido del programa) ---
        self.timer_fsm = self.create_timer(0.05, self.maquina_de_estados_loop) # 20 Hz para movimiento
        self.timer_watchdog = self.create_timer(0.5, self.reloj_partida)       # 2 Hz para el árbitro

    # ================================================================
    #  CALLBACKS Y RELOJ DE PARTIDA
    # ================================================================
    def robot_callback(self, msg: Pose2D):
        self.robot_pose = msg

    def reloj_partida(self):
        if self.t_inicio is None or self.estado_actual in [EstadoRobot.FIN_PARTIDA, EstadoRobot.ESPERANDO_TIRETTE]:
            return

        tiempo_restante = TIEMPO_TOTAL - (time.time() - self.t_inicio)
        
        # Publicar estado para poder leerlo desde el portátil
        msg_est = String()
        msg_est.data = f"{self.estado_actual.name} | {max(tiempo_restante, 0):.1f}s"
        self.pub_est.publish(msg_est)

        # Si quedan menos de 20 segundos y no estamos ya volviendo, abortar y aparcar
        if tiempo_restante <= TIEMPO_RETORNO and not self.emergencia_activa:
            self.get_logger().warn(f"🚨 {tiempo_restante:.0f}s RESTANTES. ¡ABORTANDO Y VOLVIENDO A BASE!")
            self.emergencia_activa = True
            self.estado_actual = EstadoRobot.RETORNO_EMERGENCIA

        # A los 100 segundos, corte de corriente obligatorio
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
    #  GENERACIÓN DE RUTAS (Coordenadas Explícitas en cm)
    # ================================================================
    def calcular_waypoints(self):
        # Auto-detectar equipo basado en la posición inicial (Mitad del campo = X:150)
        self.equipo = "AZUL" if self.robot_pose.x > 150.0 else "AMARILLO"
        self.get_logger().info(f"🔵🟡 Equipo detectado: {self.equipo}")

        # Definimos LOS PUNTOS EXACTOS a los que irá el CENTRO del robot.
        # Puedes modificar estos números a tu gusto tras medir en la pista.
        if self.equipo == "AZUL":
            self.rutas_mision = [
                # Misión 1: Primer grupo de piezas
                {"rodeo_x": 260.0, "rodeo_y": 10.0,  # Punto para esquivarlas por la derecha
                 "pos_x": 235.0,   "pos_y": 50.0},   # Posición exacta detrás de ellas
                
                # Misión 2: Segundo grupo de piezas
                {"rodeo_x": 285.0, "rodeo_y": 10.0,  
                 "pos_x": 265.0,   "pos_y": 20.0}    
            ]
            self.park_x = 280.0
            self.park_y = 20.0
            
        else: # EQUIPO AMARILLO (Espejo manual)
            self.rutas_mision = [
                # Misión 1: Primer grupo de piezas
                {"rodeo_x": 40.0,  "rodeo_y": 10.0,  # Punto para esquivarlas por la izquierda
                 "pos_x": 65.0,    "pos_y": 50.0},   # Posición exacta detrás de ellas
                
                # Misión 2: Segundo grupo de piezas
                {"rodeo_x": 15.0,  "rodeo_y": 10.0,  
                 "pos_x": 35.0,    "pos_y": 20.0}    
            ]
            self.park_x = 20.0
            self.park_y = 20.0

    # ================================================================
    #  HERRAMIENTAS DE MOVIMIENTO ROBUSTO (Modo Tanque)
    # ================================================================
    def girar_absoluto(self, target_theta, tolerancia=5.0):
        """Rota sobre sí mismo. Devuelve True si ha terminado."""
        if self.robot_pose is None: return False
        error_theta = (target_theta - self.robot_pose.theta + 180) % 360 - 180
        if abs(error_theta) < tolerancia:
            self.parar_motores()
            return True
            
        # Gira con control proporcional. Mínimo 0.2 para vencer fricción
        w = max(0.2, min(1.0, abs(0.03 * error_theta)))
        w = w if error_theta > 0 else -w
        self.enviar_velocidad(vx=0.0, vy=0.0, w=w)
        return False

    def avanzar_recto(self, target_x, target_y, tolerancia=8.0):
        """Solo avanza. Asume que ya está encarado hacia el punto."""
        if self.robot_pose is None: return False
        error_x = target_x - self.robot_pose.x
        error_y = target_y - self.robot_pose.y
        distancia = math.hypot(error_x, error_y)
        if distancia < tolerancia:
            self.parar_motores()
            return True
        
        # Calcula si el objetivo está por delante o por detrás
        theta_rad = math.radians(self.robot_pose.theta)
        error_local_x = error_x * math.cos(theta_rad) + error_y * math.sin(theta_rad)
        direccion = 1.0 if error_local_x > 0 else -1.0

        # Avanza con control proporcional (traduce distancia en cm a m/s)
        vx = direccion * max(0.1, min(0.35, abs(0.02 * distancia)))
        self.enviar_velocidad(vx=vx, vy=0.0, w=0.0)
        return False

    def ir_a_punto_como_tanque(self, target_x, target_y):
        """La función maestra: Primero gira hacia el punto, luego avanza rectilíneo"""
        if self.robot_pose is None: return False
        # Calculamos qué ángulo necesita para mirar al punto exacto
        angulo_objetivo = math.degrees(math.atan2(target_y - self.robot_pose.y, target_x - self.robot_pose.x))
        
        # 1. Girar
        if not self.girar_absoluto(angulo_objetivo, tolerancia=5.0):
            return False # Aún no ha terminado de girar
        
        # 2. Avanzar
        if not self.avanzar_recto(target_x, target_y, tolerancia=8.0):
            return False # Aún no ha terminado de avanzar
            
        return True # ¡Llegamos al punto!

    def enviar_velocidad(self, vx, vy, w):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy  # Se mantendrá a 0.0 gracias al modo tanque
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

    def parar_motores(self):
        self.enviar_velocidad(0.0, 0.0, 0.0)

    # ================================================================
    #  EL GUION PRINCIPAL DE LA PARTIDA (La Receta)
    # ================================================================
    def maquina_de_estados_loop(self):
        # Evitar hacer nada si el juego ha terminado
        if self.estado_actual == EstadoRobot.FIN_PARTIDA:
            self.parar_motores()
            return

        # 0. ESPERAR CORDÓN (Y asegurarse de que la cámara nos ve)
        if self.estado_actual == EstadoRobot.ESPERANDO_TIRETTE:
            self.parar_motores()
            if HARDWARE_DISPONIBLE and self.tirette.is_pressed and self.robot_pose is not None:
                self.t_inicio = time.time()
                self.estado_actual = EstadoRobot.CALCULAR_RUTAS

        # 1. AUTO-CONFIGURAR RUTAS
        elif self.estado_actual == EstadoRobot.CALCULAR_RUTAS:
            self.calcular_waypoints()
            self.estado_actual = EstadoRobot.RODEAR_PIEZAS
            
        # 2. RODEAR PIEZAS POR EL EXTERIOR
        elif self.estado_actual == EstadoRobot.RODEAR_PIEZAS:
            mision = self.rutas_mision[self.mision_actual]
            if self.ir_a_punto_como_tanque(mision["rodeo_x"], mision["rodeo_y"]):
                self.get_logger().info("Punto de rodeo alcanzado. Entrando detrás de las piezas...")
                self.estado_actual = EstadoRobot.POSICIONAR_DETRAS

        # 3. COLOCARSE DETRÁS DE LAS PIEZAS Y ENCARAR EL NIDO
        elif self.estado_actual == EstadoRobot.POSICIONAR_DETRAS:
            mision = self.rutas_mision[self.mision_actual]
            # Vamos detrás de las piezas
            if self.ir_a_punto_como_tanque(mision["pos_x"], mision["pos_y"]):
                # Una vez detrás, giramos forzosamente hacia el Norte (90 grados, hacia el nido)
                if self.girar_absoluto(90.0):
                    self.get_logger().info("Alineación a 90º completada. ¡INICIANDO EMPUJE CERRADO!")
                    self.estado_actual = EstadoRobot.EMPUJAR_AL_NIDO

        # 4. EMPUJE CONTINUO CONTROLADO POR CÁMARA
        elif self.estado_actual == EstadoRobot.EMPUJAR_AL_NIDO:
            # Empujamos recto manteniendo el Norte hasta cruzar la ZONA_ENTREGA_Y
            if self.robot_pose.y >= ZONA_ENTREGA_Y:
                self.parar_motores()
                self.get_logger().info(f"✅ ¡Grupo {self.mision_actual + 1} entregado en el nido!")
                self.estado_actual = EstadoRobot.EVALUAR_SIGUIENTE
            else:
                # Mantenemos W=0 para forzar empuje totalmente recto.
                self.enviar_velocidad(vx=0.25, vy=0.0, w=0.0)

        # 5. PASAR AL SIGUIENTE GRUPO
        elif self.estado_actual == EstadoRobot.EVALUAR_SIGUIENTE:
            self.mision_actual += 1
            if self.mision_actual < len(self.rutas_mision):
                self.get_logger().info("Yendo a por el siguiente grupo de piezas...")
                self.estado_actual = EstadoRobot.RODEAR_PIEZAS
            else:
                self.get_logger().info("Misiones completadas antes de tiempo. Volviendo a base.")
                self.estado_actual = EstadoRobot.RETORNO_EMERGENCIA

        # 6. RETORNO DE EMERGENCIA / FIN DE LA LISTA
        elif self.estado_actual == EstadoRobot.RETORNO_EMERGENCIA:
            if self.ir_a_punto_como_tanque(self.park_x, self.park_y):
                self.get_logger().info("Robot aparcado de forma segura.")
                self.estado_actual = EstadoRobot.FIN_PARTIDA

def main(args=None):
    rclpy.init(args=args)
    nodo = CerebroEurobot()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Cierre forzado. Apagando motores...")
    finally:
        nodo.parar_motores() # Freno de emergencia al cerrar
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()