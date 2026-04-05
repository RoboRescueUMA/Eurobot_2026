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
TIEMPO_TOTAL   = 100.0
TIEMPO_RETORNO =  20.0
ZONA_ENTREGA_Y = 160.0

VEL_MAX_RECTO  = 0.30
VEL_MIN_RECTO  = 0.15
VEL_MAX_GIRO   = 0.50
VEL_MIN_GIRO   = 0.15
# VEL_EMPUJE     = 0.25

# ================================================================
#  MÁQUINA DE ESTADOS
# ================================================================
class EstadoRobot(Enum):
    ESPERANDO_TIRETTE = 0
    CALCULAR_RUTAS = 1
    M1_RODEAR_PIEZAS = 2
    M1_POSICIONAR_DETRAS = 3
    M1_ENCARAR_NIDO = 4
    M1_EMPUJAR_AL_NIDO = 5
    
    # --- Misión 2: Secuencia Expandida ---
    M2_EVALUAR_PIEZA = 6
    M2_IR_PUNTO_PREPARACION = 7
    M2_BAJAR_GARRA = 8
    M2_ESPERAR_BAJAR = 9
    M2_ABRIR_GARRA = 10
    M2_ESPERAR_ABRIR = 11
    M2_APROXIMAR_PIEZA = 12
    M2_CERRAR_GARRA = 13
    M2_ESPERAR_CERRAR = 14
    M2_SUBIR_GARRA = 15
    M2_ESPERAR_SUBIR = 16
    M2_RETROCEDER_INTERMEDIO = 17
    M2_IR_PUNTO_ENTREGA = 18
    M2_SOLTAR_PIEZA = 19
    M2_ESPERAR_SOLTAR = 20
    
    RETORNO_EMERGENCIA = 21
    FIN_PARTIDA = 22

class CerebroEurobot(Node):
    def __init__(self):
        super().__init__("cerebro_eurobot")

        self.estado_actual = EstadoRobot.ESPERANDO_TIRETTE
        self.robot_pose = None
        self.equipo = "DESCONOCIDO"
        self.t_inicio = None
        self.emergencia_activa = False
        self.lista_piezas = []
        self.indice_pieza = 0

        self.enemigo_cerca = False
        self.accion_garra_completada = False
        self.DISTANCIA_SEGURIDAD = 35.0

        if HARDWARE_DISPONIBLE:
            self.tirette = Button(pin=17, pull_up=True, bounce_time=0.05)
        else:
            self.tirette = None
            self.timer_simulacion = self.create_timer(3.0, self._simular_tirette)

        self.sub_robot = self.create_subscription(Pose2D, "/roborescue/robot_pose", self.robot_callback, 1)
        self.sub_enemigo = self.create_subscription(Pose2D, "/roborescue/enemigo_pose", self.enemigo_callback, 1)
        self.sub_estado_garra = self.create_subscription(String, "/roborescue/garra_status", self.garra_status_callback, 1)
        
        self.pub_cmd = self.create_publisher(Twist, "/roborescue/cmd_vel", 1)
        self.pub_garra = self.create_publisher(String, "/roborescue/cmd_garra", 1) 
        self.pub_est = self.create_publisher(String, "/roborescue/robot_estado", 1)
        
        self.timer_fsm = self.create_timer(0.05, self.maquina_de_estados_loop)
        self.timer_watchdog = self.create_timer(0.5, self.reloj_partida)

    def robot_callback(self, msg: Pose2D): self.robot_pose = msg
    def garra_status_callback(self, msg: String):
        if msg.data == "LISTO": self.accion_garra_completada = True

    def enemigo_callback(self, msg: Pose2D):
        if self.robot_pose is None: return
        distancia = math.hypot(msg.x - self.robot_pose.x, msg.y - self.robot_pose.y)
        self.enemigo_cerca = distancia < self.DISTANCIA_SEGURIDAD

    def reloj_partida(self):
        if self.t_inicio is None or self.estado_actual in [EstadoRobot.FIN_PARTIDA, EstadoRobot.ESPERANDO_TIRETTE]: return
        tiempo_restante = TIEMPO_TOTAL - (time.time() - self.t_inicio)
        if tiempo_restante <= TIEMPO_RETORNO and not self.emergencia_activa:
            self.emergencia_activa = True
            self.estado_actual = EstadoRobot.RETORNO_EMERGENCIA
        if tiempo_restante <= 0.0: self.estado_actual = EstadoRobot.FIN_PARTIDA

    def _simular_tirette(self):
        if self.estado_actual == EstadoRobot.ESPERANDO_TIRETTE and self.robot_pose is not None:
            self.t_inicio = time.time(); self.estado_actual = EstadoRobot.CALCULAR_RUTAS; self.timer_simulacion.cancel()

    def calcular_waypoints(self):
        self.equipo = "AZUL" if self.robot_pose.x > 150.0 else "AMARILLO"
        if self.equipo == "AMARILLO":
            self.m1_rodeo_x = 55.0; self.m1_rodeo_y = 80.0
            self.m1_pos_x   = 12.0; self.m1_pos_y   = 75.0
            self.lista_piezas = [
                {"prep_x": 15.0, "prep_y": 60.0, "cap_x": 15.0, "cap_y": 70.0, "ent_x": 15.0, "ent_y": 170.0},
                {"prep_x": 15.0, "prep_y": 60.0, "cap_x": 15.0, "cap_y": 75.0, "ent_x": 15.0, "ent_y": 170.0},
                {"prep_x": 15.0, "prep_y": 60.0, "cap_x": 15.0, "cap_y": 80.0, "inter_x": 40.0, "inter_y": 80.0, "ent_x": 15.0, "ent_y": 80.0},
                {"prep_x": 15.0, "prep_y": 60.0, "cap_x": 15.0, "cap_y": 85.0, "inter_x": 40.0, "inter_y": 100.0, "ent_x": 20.0, "ent_y": 100.0}
            ]
            self.park_x = 12.0; self.park_y = 160.0
        else:
            self.m1_rodeo_x = 245.0; self.m1_rodeo_y = 80.0
            self.m1_pos_x   = 288.0; self.m1_pos_y   = 75.0
            self.lista_piezas = [{"prep_x": 285.0, "prep_y": 60.0, "cap_x": 285.0, "cap_y": 70.0, "ent_x": 260.0, "ent_y": 185.0}]
            self.park_x = 288.0; self.park_y = 160.0

    def normalizar_angulo(self, angulo):
        while angulo > 180.0: angulo -= 360.0
        while angulo < -180.0: angulo += 360.0 
        return angulo
    
    def girar_absoluto(self, target_theta, tolerancia=5.0):
        if self.robot_pose is None: return False
        error_theta = self.normalizar_angulo(target_theta - self.robot_pose.theta)
        if abs(error_theta) < tolerancia: self.parar_motores(); return True
        w = max(VEL_MIN_GIRO, min(VEL_MAX_GIRO, abs(0.015 * error_theta)))
        self.enviar_velocidad(0.0, 0.0, w if error_theta > 0 else -w); return False

    def avanzar_recto(self, target_x, target_y, tolerancia=8.0, sentido=1):
        if self.robot_pose is None: return False
        error_x = target_x - self.robot_pose.x; error_y = target_y - self.robot_pose.y
        distancia = math.hypot(error_x, error_y)
        if distancia < tolerancia: self.parar_motores(); return True
        angulo_obj = math.degrees(math.atan2(error_y, error_x))
        if sentido == -1: angulo_obj = self.normalizar_angulo(angulo_obj + 180.0)
        w_corr = 0.015 * self.normalizar_angulo(angulo_obj - self.robot_pose.theta)
        vx = sentido * max(VEL_MIN_RECTO, min(VEL_MAX_RECTO, abs(0.02 * distancia)))
        self.enviar_velocidad(vx, 0.0, w_corr); return False

    def ir_a_punto_como_tanque(self, target_x, target_y, sentido=1):
        if self.robot_pose is None: return False
        if math.hypot(target_x - self.robot_pose.x, target_y - self.robot_pose.y) < 8.0: self.parar_motores(); return True
        angulo_obj = math.degrees(math.atan2(target_y - self.robot_pose.y, target_x - self.robot_pose.x))
        if sentido == -1: angulo_obj = self.normalizar_angulo(angulo_obj + 180.0)
        if not self.girar_absoluto(angulo_obj, 2.0): return False 
        return self.avanzar_recto(target_x, target_y, 8.0, sentido)

    def enviar_velocidad(self, vx, vy, w):
        cmd = Twist(); cmd.linear.x = float(vx); cmd.linear.y = float(vy); cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)

    def parar_motores(self): self.enviar_velocidad(0.0, 0.0, 0.0)

    # ================================================================
    #  EL GUION PRINCIPAL
    # ================================================================
    def maquina_de_estados_loop(self):
        if self.enemigo_cerca: self.parar_motores(); return
        if self.estado_actual == EstadoRobot.FIN_PARTIDA: self.parar_motores(); return

        if self.estado_actual == EstadoRobot.ESPERANDO_TIRETTE:
            if HARDWARE_DISPONIBLE and self.tirette.is_pressed and self.robot_pose is not None:
                self.t_inicio = time.time(); self.estado_actual = EstadoRobot.CALCULAR_RUTAS
        elif self.estado_actual == EstadoRobot.CALCULAR_RUTAS:
            self.calcular_waypoints(); self.estado_actual = EstadoRobot.M1_RODEAR_PIEZAS
        elif self.estado_actual == EstadoRobot.M1_RODEAR_PIEZAS:
            if self.ir_a_punto_como_tanque(self.m1_rodeo_x, self.m1_rodeo_y, -1): self.estado_actual = EstadoRobot.M1_POSICIONAR_DETRAS
        elif self.estado_actual == EstadoRobot.M1_POSICIONAR_DETRAS:
            if self.ir_a_punto_como_tanque(self.m1_pos_x, self.m1_pos_y, -1): self.estado_actual = EstadoRobot.M1_ENCARAR_NIDO
        elif self.estado_actual == EstadoRobot.M1_ENCARAR_NIDO:
            if self.girar_absoluto(90.0): self.estado_actual = EstadoRobot.M1_EMPUJAR_AL_NIDO
        elif self.estado_actual == EstadoRobot.M1_EMPUJAR_AL_NIDO:
            if self.robot_pose.y >= ZONA_ENTREGA_Y: 
                self.parar_motores(); self.estado_actual = EstadoRobot.M2_EVALUAR_PIEZA
            else: self.ir_a_punto_como_tanque(self.park_x, self.park_y, -1)

        # ==========================================================
        # MISIÓN 2: BUCLE DE LA GARRA
        # ==========================================================
        elif self.estado_actual == EstadoRobot.M2_EVALUAR_PIEZA:
            if self.indice_pieza < len(self.lista_piezas): self.estado_actual = EstadoRobot.M2_IR_PUNTO_PREPARACION
            else: self.estado_actual = EstadoRobot.RETORNO_EMERGENCIA

        elif self.estado_actual == EstadoRobot.M2_IR_PUNTO_PREPARACION:
            p = self.lista_piezas[self.indice_pieza]
            if self.ir_a_punto_como_tanque(p["prep_x"], p["prep_y"], 1): self.estado_actual = EstadoRobot.M2_BAJAR_GARRA

        # --- 1. BAJAR ---
        elif self.estado_actual == EstadoRobot.M2_BAJAR_GARRA:
            self.accion_garra_completada = False
            # Ej: 0º Elevación (abajo), 0º Apertura (cerrada), 0º Giro (recto)
            self.pub_garra.publish(String(data="0,0,0"))
            self.estado_actual = EstadoRobot.M2_ESPERAR_BAJAR

        elif self.estado_actual == EstadoRobot.M2_ESPERAR_BAJAR:
            if self.accion_garra_completada: self.estado_actual = EstadoRobot.M2_ABRIR_GARRA

        # --- 2. ABRIR ---
        elif self.estado_actual == EstadoRobot.M2_ABRIR_GARRA:
            self.accion_garra_completada = False
            # Ej: 0º Elevación (sigue abajo), 90º Apertura (abierta), 0º Giro
            self.pub_garra.publish(String(data="0,90,0"))
            self.estado_actual = EstadoRobot.M2_ESPERAR_ABRIR

        elif self.estado_actual == EstadoRobot.M2_ESPERAR_ABRIR:
            if self.accion_garra_completada: self.estado_actual = EstadoRobot.M2_APROXIMAR_PIEZA

        # --- 3. APROXIMAR ---
        elif self.estado_actual == EstadoRobot.M2_APROXIMAR_PIEZA:
            p = self.lista_piezas[self.indice_pieza]
            if self.ir_a_punto_como_tanque(p["cap_x"], p["cap_y"], 1): self.estado_actual = EstadoRobot.M2_CERRAR_GARRA

        # --- 4. CERRAR ---
        elif self.estado_actual == EstadoRobot.M2_CERRAR_GARRA:
            self.accion_garra_completada = False
            # Ej: 0º Elevación (abajo), 0º Apertura (cerrada), 0º Giro
            self.pub_garra.publish(String(data="0,0,0"))
            self.estado_actual = EstadoRobot.M2_ESPERAR_CERRAR

        elif self.estado_actual == EstadoRobot.M2_ESPERAR_CERRAR:
            if self.accion_garra_completada: self.estado_actual = EstadoRobot.M2_SUBIR_GARRA

        # --- 5. SUBIR ---
        elif self.estado_actual == EstadoRobot.M2_SUBIR_GARRA:
            self.accion_garra_completada = False
            # Ej: 180º Elevación (arriba), 0º Apertura (cerrada), 0º Giro
            self.pub_garra.publish(String(data="180,0,0"))
            self.estado_actual = EstadoRobot.M2_ESPERAR_SUBIR

        elif self.estado_actual == EstadoRobot.M2_ESPERAR_SUBIR:
            if self.accion_garra_completada:
                if self.indice_pieza >= 2: self.estado_actual = EstadoRobot.M2_RETROCEDER_INTERMEDIO
                else: self.estado_actual = EstadoRobot.M2_IR_PUNTO_ENTREGA

        # --- NAVEGACIÓN A ENTREGA ---
        elif self.estado_actual == EstadoRobot.M2_RETROCEDER_INTERMEDIO:
            p = self.lista_piezas[self.indice_pieza]
            if self.ir_a_punto_como_tanque(p["inter_x"], p["inter_y"], -1): 
                self.estado_actual = EstadoRobot.M2_IR_PUNTO_ENTREGA

        elif self.estado_actual == EstadoRobot.M2_IR_PUNTO_ENTREGA:
            p = self.lista_piezas[self.indice_pieza]
            if self.ir_a_punto_como_tanque(p["ent_x"], p["ent_y"], 1): self.estado_actual = EstadoRobot.M2_SOLTAR_PIEZA

        # --- SOLTAR ---
        elif self.estado_actual == EstadoRobot.M2_SOLTAR_PIEZA:
            self.accion_garra_completada = False
            # Ej: 180º Elevación (arriba), 90º Apertura (abre para soltar), 0º Giro
            self.pub_garra.publish(String(data="180,90,0"))
            self.estado_actual = EstadoRobot.M2_ESPERAR_GARRA_SOLTAR

        elif self.estado_actual == EstadoRobot.M2_ESPERAR_GARRA_SOLTAR:
            if self.accion_garra_completada:
                self.indice_pieza += 1; self.estado_actual = EstadoRobot.M2_EVALUAR_PIEZA 

        elif self.estado_actual == EstadoRobot.RETORNO_EMERGENCIA:
            if self.ir_a_punto_como_tanque(self.park_x, self.park_y, -1): self.estado_actual = EstadoRobot.FIN_PARTIDA

def main(args=None):
    rclpy.init(args=args); nodo = CerebroEurobot()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally: nodo.parar_motores(); nodo.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()