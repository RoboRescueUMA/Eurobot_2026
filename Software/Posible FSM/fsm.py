#!/usr/bin/env python3

import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped, Pose2D, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum, auto
 
 
# ══════════════════════════════════════════════════════════════════════════════
#  PARÁMETROS GLOBALES
# ══════════════════════════════════════════════════════════════════════════════
 
# Tiempo total de partida y margen para el retorno de emergencia
TIEMPO_TOTAL   = 100.0   # segundos
TIEMPO_RETORNO =  20.0   
 
# Geometría del robot  
Lx = 0.10   
Ly = 0.14   
MARGEN_SEGURIDAD = 0.05   # holgura extra alrededor del robot [m]
 
# Velocidad y parámetros del empuje ciego
VEL_EMPUJE       = 0.1   
VEL_RODEO        = 0.1   
FREQ_SPIN        = 0.05
 
# Dimensiones del campo  [m]
CAMPO_X = 3.00
CAMPO_Y = 2.00
 
# Coordenadas aproximadas de los grupos de piezas  (centro del grupo, 4 piezas en fila) 
# MEDIR ESTO EN LA REALIDAD

#  Grupo 1 (más cercano a la zona de salida del robot)
PIEZAS_F1_Y = 0.70   
PIEZAS_F2_Y = 0.40   
 
# Tolerancia de orientación antes de empujar (en grados)
# Si el robot se desvía más de este valor respecto a 90°, se reintenta la alineación
TOLERANCIA_ORIENTACION_DEG = 5.0
# Distancia en Y a la que se considera que el robot ha entrado en la zona de entrega
ZONA_Y = CAMPO_Y - 0.15   
 
# ══════════════════════════════════════════════════════════════════════════════
#  MÁQUINA DE ESTADOS
# ══════════════════════════════════════════════════════════════════════════════
 
class Estado(Enum):
    INIT               = auto()
    ESPERAR_POSE       = auto()
    # Fase 1 – grupo de piezas más cercano a la zona de salida
    F1_RODEAR          = auto()   # rodear las piezas por fuera
    F1_POSICION        = auto()   # alinearse
    F1_EMPUJAR         = auto()   # empuje ciego con la delantera hacia la zona
    # Fase 2 – segundo grupo
    F2_RODEAR          = auto()
    F2_POSICION        = auto()
    F2_EMPUJAR         = auto()
    # Final
    APARCAR            = auto()
    COMPLETADO         = auto()
    RETORNO_EMERGENCIA = auto()
 
 
# ══════════════════════════════════════════════════════════════════════════════
#  NODO PRINCIPAL
# ══════════════════════════════════════════════════════════════════════════════
 
class MisionEurobot(Node):
 
    def __init__(self):
        super().__init__('mision_eurobot')
 
        self.nav      = BasicNavigator()
        self.pub_vel  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_est  = self.create_publisher(String, '/robot_estado', 10)
 
        self.sub_pose = self.create_subscription(
            Pose2D, '/robot_pose', self._pose_cb, 10)
 
        self.pose_actual: Pose2D | None = None
        self.estado                     = Estado.INIT
        self.lado:   str   | None       = None
        self.t_inicio: float | None     = None
        self.emergencia_activa          = False
 
        # Revisamos el estado del robot cada 100 ms 
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info("Ronda iniciada. Esperando pose inicial…")
 
    # ──────────────────────────────────────────────────────────────────────────
    #  CALLBACKS
    # ──────────────────────────────────────────────────────────────────────────
 
    def _pose_cb(self, msg: Pose2D) -> None:
        self.pose_actual = msg
 
    # Publica el estado del robot y lo devuelve al nido si se acerca el tiempo límite
    def _watchdog(self) -> None:
        if self.t_inicio is None:
            return
 
        restante = TIEMPO_TOTAL - (time.time() - self.t_inicio)
 
        # Estados en los que NO se interrumpe  (Ya ha terminado o esta yendo al nido)
        estados_protegidos = {
            #Estado.F1_EMPUJAR, No se si dejarlos protegidos o no
            #Estado.F2_EMPUJAR,
            Estado.APARCAR,
            Estado.COMPLETADO,
            Estado.RETORNO_EMERGENCIA,
        }
 
        if (restante <= TIEMPO_RETORNO
                and not self.emergencia_activa
                and self.estado not in estados_protegidos):
            self.get_logger().warn(
                f"⏰ {restante:.0f} s → RETORNO DE EMERGENCIA")
            self.emergencia_activa = True
            self.nav.cancelTask()
            self.estado = Estado.RETORNO_EMERGENCIA
 
        if restante <= 0 and self.estado != Estado.COMPLETADO:
            self.nav.cancelTask()
            self.estado = Estado.COMPLETADO
 
        msg = String()
        msg.data = f"{self.estado.name} | {max(restante, 0):.1f}s"
        self.pub_est.publish(msg)

 
    # ──────────────────────────────────────────────────────────────────────────
    #  HELPERS DE NAVEGACIÓN
    # ──────────────────────────────────────────────────────────────────────────
 
    def _hacer_pose(self, x: float, y: float, theta_deg: float) -> PoseStamped:
        """Construye un PoseStamped en el frame 'map'."""
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        rad = math.radians(theta_deg)
        ps.pose.orientation.z = math.sin(rad / 2.0)
        ps.pose.orientation.w = math.cos(rad / 2.0)
        return ps
    
    # Vacía el buffer de pose y espera un mensaje nuevo y fresco del topic /robot_pose. P
    # Para APARCAR y quzias empujar piezas. Es una idea, no lo he metido en el resto del codigo
    def obtener_pose_fresca(self, espera_previa: float = 1.5, timeout: float = 5.0) -> Pose2D | None:
 
        # 1. Asegurar que el robot está parado
        self.pub_vel.publish(Twist())
 
        # 2. Esperar a que el frame "en vuelo" se procese
        #    (cubre el RTT red + procesamiento visión en Raspberry)
        self.get_logger().info(f"  Esperando {espera_previa}s para pose fresca…")
        t_espera = time.time() + espera_previa
        while time.time() < t_espera and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
 
        # 3. Vaciar buffer ROS destruyendo y recreando la suscripción
        self.destroy_subscription(self.sub_pose)
        self.pose_actual = None
        self.sub_pose = self.create_subscription(
            Pose2D, '/robot_pose', self._pose_cb, 10)
 
        # 4. Esperar el SIGUIENTE mensaje
        t0 = time.time()
        while self.pose_actual is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > timeout:
                self.get_logger().warn("⚠️ Timeout esperando pose fresca.")
                return None
 
        self.get_logger().info(
            f"  Pose fresca: ({self.pose_actual.x:.3f}, {self.pose_actual.y:.3f})")
        return self.pose_actual
    
    # LLeva con Nav2 hasta el GOAL. Devuelve TRUE si llega, FALSE si hay timeout, emergencia o falla
    def ir_a(self, x: float, y: float, theta_deg: float, timeout: float = 20.0) -> bool:
        
        goal = self._hacer_pose(x, y, theta_deg)
        self.get_logger().info(f"  Nav2 → ({x:.3f}, {y:.3f}, {theta_deg:.0f}°)")
        self.nav.goToPose(goal)
 
        t0 = time.time()
        while not self.nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=FREQ_SPIN)
            if self.emergencia_activa:
                return False
            if time.time() - t0 > timeout:
                self.get_logger().warn("⚠️  Timeout de navegación – Cancelando...")
                self.nav.cancelTask()
                return False
 
        return self.nav.getResult() == TaskResult.SUCCEEDED
 
    # Empuje a velocidad constante (Sin usar nav2, para evitar que se raye)
    def empujar_ciego(self, distancia: float, theta_objetivo_deg: float = 90.0,
                      velocidad: float = VEL_EMPUJE,
                      pos_x: float | None = None, pos_y: float | None = None) -> None:
 
        # PONER SI FUNCIONA LO DE VACIAR EL BUFFER DE LA CÁMARA y despues de revisarlo xq no se si funciona xd
        """"
        pose = self.obtener_pose_fresca(espera_previa=0.5)
        if pose is not None:
            theta_actual_deg = math.degrees(pose.theta)
            error_deg = theta_actual_deg - theta_objetivo_deg
            error_deg = (error_deg + 180) % 360 - 180
            error_deg = abs(error_deg)
 
            if error_deg > TOLERANCIA_ORIENTACION_DEG:
                self.get_logger().warn(
                    f"⚠️  Orientación desviada {error_deg:.1f}° (objetivo {theta_objetivo_deg}°). "
                    f"Reintentando alineación…")
                # Reintenta la alineación en la posición actual o en la indicada
                x_alin = pos_x if pos_x is not None else pose.x
                y_alin = pos_y if pos_y is not None else pose.y
                ok = self.ir_a(x_alin, y_alin, theta_objetivo_deg, timeout=10.0)
                if not ok:
                    self.get_logger().warn(
                        "⚠️  No se pudo corregir la orientación. Empuje cancelado.")
                    return
            else:
                self.get_logger().info(
                    f"  Orientación OK ({theta_actual_deg:.1f}°, error {error_deg:.1f}°).")
        else:
            self.get_logger().warn(
                "⚠️  Sin pose disponible para verificar orientación. Empujando igualmente…")
        """
        # --- Empuje ---
        duracion = abs(distancia) / abs(velocidad)
        self.get_logger().info(
            f"  Empuje ciego: {distancia:.3f} m a {velocidad:.3f} m/s → {duracion:.1f} s")
 
        cmd = Twist()
        cmd.linear.x  = float(velocidad)
        cmd.angular.z = 0.0
 
        # Cancelar cualquier tarea Nav2 activa
        if not self.nav.isTaskComplete():
            self.nav.cancelTask()
 
        t_fin = time.time() + duracion
        while time.time() < t_fin and rclpy.ok():
            self.pub_vel.publish(cmd)
            rclpy.spin_once(self, timeout_sec=FREQ_SPIN)
 
        # Parada de seguridad
        self.pub_vel.publish(Twist())
        self.get_logger().info("🛑 Empuje finalizado.")
 
    def cambiar_estado(self, nuevo: Estado) -> None:
        self.get_logger().info(f"Estado: {self.estado.name} → {nuevo.name}")
        self.estado = nuevo
 
    # ──────────────────────────────────────────────────────────────────────────
    #  WAYPOINTS
    # ──────────────────────────────────────────────────────────────────────────
 
    def _waypoints(self) -> dict:

        # Distancia desde el robot (en posición) hasta la zona de entrega
        # = Y_zona - Y_piezas + algo extra para asegurar que entran
        dist_empuje_f1 = (ZONA_Y - PIEZAS_F1_Y) + Lx + 0.05
        dist_empuje_f2 = (ZONA_Y - PIEZAS_F2_Y) + Lx + 0.05
 
        # Y de posicionamiento: detrás del grupo, con la delantera tocando
        pos_y_f1 = PIEZAS_F1_Y - (Lx + MARGEN_SEGURIDAD)
        pos_y_f2 = PIEZAS_F2_Y - (Lx + MARGEN_SEGURIDAD)
 
        # Y de rodeo: suficiente para pasar por fuera del grupo (4 piezas ≈ 0.30 m)
        rodeo_y  = min(PIEZAS_F2_Y - 0.20, 0.10)   # por debajo del grupo más bajo
 
        if self.lado == "AZUL":
            # AZUL: derecha de la imagen
            x_f1 = 2.35   # grupo 1 CAMBIARLAS MEDIDAS CUANDO MIDA EL MAPA
            x_f2 = 2.65   # grupo 2
            x_rodeo = x_f1 + Ly + MARGEN_SEGURIDAD + 0.10  

            return {
                "f1_rodear":   (x_rodeo,  rodeo_y,   270.0),  # bajar por el lateral
                "f1_posicion": (x_f1,     pos_y_f1,   90.0),  # alinearse detrás F1
                "f2_rodear":   (x_rodeo,  rodeo_y,   270.0),  # volver a bajar
                "f2_posicion": (x_f2,     pos_y_f2,   90.0),  # alinearse detrás F2
                "park":        (2.80,      0.20,      180.0),  
 
                # Distancias de empuje ciego
                "dist_f1":     dist_empuje_f1,
                "dist_f2":     dist_empuje_f2,

                # Posición X de cada grupo: se usa para reintento de alineación
                "x_f1": x_f1,
                "x_f2": x_f2,
                "pos_y_f1": pos_y_f1,
                "pos_y_f2": pos_y_f2,
            }
 
        else:  # AMARILLO: izquierda
            x_f1 = 0.65
            x_f2 = 0.35
            x_rodeo = x_f1 - (Ly + MARGEN_SEGURIDAD + 0.10)  
            return {
                "f1_rodear":   (x_rodeo,  rodeo_y,   270.0),
                "f1_posicion": (x_f1,     pos_y_f1,   90.0),
                "f2_rodear":   (x_rodeo,  rodeo_y,   270.0),
                "f2_posicion": (x_f2,     pos_y_f2,   90.0),
                "park":        (0.20,      0.20,        0.0),
 
                "dist_f1":     dist_empuje_f1,
                "dist_f2":     dist_empuje_f2,

                "x_f1": x_f1,
                "x_f2": x_f2,
                "pos_y_f1": pos_y_f1,
                "pos_y_f2": pos_y_f2,
            }
    
    
    def _manejar_fallo(self, etapa: str) -> bool:
 
        etapas_criticas = {"f1_posicion", "f2_posicion"}
 
        if etapa in etapas_criticas:
            self.get_logger().warn(
                f"🚨 Fallo crítico en '{etapa}'"
                f"Yendo a aparcar.")
            self.emergencia_activa = True
            return False   # señal para que ejecutar() salte al aparcamiento
 
        # Etapa no crítica: loggear y continuar
        self.get_logger().warn(f"⚠️  Fallo en etapa '{etapa}' – continuando.")
        return True
    
    # ──────────────────────────────────────────────────────────────────────────
    #  MISIÓN PRINCIPAL
    # ──────────────────────────────────────────────────────────────────────────
 
    def ejecutar(self) -> None:

        # 1 ── Esperar primera pose
        self.cambiar_estado(Estado.ESPERAR_POSE)
        self.get_logger().info("Esperando /robot_pose inicial…")
        while self.pose_actual is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
 
        # Estimamos en que lado de la arena estamos
        self.lado = "AZUL" if self.pose_actual.x > 1.5 else "AMARILLO"
        self.get_logger().info(f"✅ Color de ronda: {self.lado}  "f"(x₀={self.pose_actual.x:.2f})")
 
        wp = self._waypoints()
        self.t_inicio = time.time()
 
        # ── FASE 1 ────────────────────────────────────────────────────────────
        self.cambiar_estado(Estado.F1_RODEAR)
        if not self.ir_a(*wp["f1_rodear"]):
            self._manejar_fallo("f1_rodear")
 
        if not self.emergencia_activa:
            self.cambiar_estado(Estado.F1_POSICION)
            if not self.ir_a(*wp["f1_posicion"]):
                if not self._manejar_fallo("f1_posicion"):
                    self.emergencia_activa = True
 
        if not self.emergencia_activa:
            self.cambiar_estado(Estado.F1_EMPUJAR)
            self.empujar_ciego(
                wp["dist_f1"],
                theta_objetivo_deg=90.0,
                pos_x=wp["x_f1"],
                pos_y=wp["pos_y_f1"],
            )

        # ── FASE 2 ────────────────────────────────────────────────────────────
        if not self.emergencia_activa:
            self.cambiar_estado(Estado.F2_RODEAR)
            if not self.ir_a(*wp["f2_rodear"]):
                if not self._manejar_fallo("f2_rodear"):
                    self.emergencia_activa = True
 
        if not self.emergencia_activa:
            self.cambiar_estado(Estado.F2_POSICION)
            if not self.ir_a(*wp["f2_posicion"]):
                self._manejar_fallo("f2_posicion")
 
        if not self.emergencia_activa:
            self.cambiar_estado(Estado.F2_EMPUJAR)
            self.empujar_ciego(
                wp["dist_f2"],
                theta_objetivo_deg=90.0,
                pos_x=wp["x_f2"],
                pos_y=wp["pos_y_f2"],
            )
 
        # ── APARCAR ───────────────────────────────────────────────────────────
        self.cambiar_estado(Estado.APARCAR)
        if self.emergencia_activa:
            self.get_logger().warn("🚨 Retorno de emergencia → aparcando directamente.")

        # PONER SI APLICA
        # self.obtener_pose_fresca(espera_previa=1.0)

        # Intentar aparcar con timeout generoso
        ok = self.ir_a(*wp["park"], timeout=TIEMPO_RETORNO - 2.0)
        if not ok:
            self.get_logger().warn("⚠️  No se pudo aparcar en tiempo.")
 
        self.cambiar_estado(Estado.COMPLETADO)
        self.get_logger().info("✅ Misión terminada.")
 
 
# ══════════════════════════════════════════════════════════════════════════════
#  ENTRADA
# ══════════════════════════════════════════════════════════════════════════════
 
def main(args=None) -> None:
    rclpy.init(args=args)
    nodo = MisionEurobot()
    nodo.nav.waitUntilNav2Active()
 
    try:
        nodo.ejecutar()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        nodo.get_logger().error(f"Error en la misión: {e}")
        raise
    finally:
        nodo.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 