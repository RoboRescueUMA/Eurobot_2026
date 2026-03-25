#!/usr/bin/env python3
"""
go_to_xy.py — Nodo ROS 2 para mover el robot a unas coordenadas (x, y) dadas.

Arquitectura:
  - Suscriptor: recibe la posición actual del robot desde la cámara cenital.
  - Action server: acepta un objetivo (x, y) y ejecuta el movimiento.
  - Publicador: envía comandos de velocidad a /cmd_vel (Twist).

Control:
  Controlador proporcional (P) de dos canales independientes:
    · Velocidad lineal  → proporcional a la distancia al objetivo.
    · Velocidad angular → proporcional al error de ángulo (heading).
  Las ruedas omnidireccionales permiten combinar ambas simultáneamente,
  por lo que no es necesario girar antes de avanzar.

Interfaces ROS 2 utilizadas:
  Suscripción : /robot_pose          [geometry_msgs/Pose2D]
  Publicación : /cmd_vel             [geometry_msgs/Twist]
  Action      : go_to_xy             [GoToXY — definido en este paquete]

Uso desde línea de comandos (sin action client):
  ros2 run <paquete> go_to_xy

Uso enviando un goal desde otro nodo o terminal:
  ros2 action send_goal /go_to_xy <paquete>/action/GoToXY \
      "{target_x: 1.0, target_y: 0.5}"
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist, Pose2D

# ---------------------------------------------------------------------------
# NOTA: Necesitas definir el action type GoToXY en tu paquete.
# Crea el fichero action/GoToXY.action con este contenido:
#
#   # Goal
#   float64 target_x
#   float64 target_y
#   ---
#   # Result
#   bool success
#   string message
#   ---
#   # Feedback
#   float64 distance_remaining
#
# Y añade en CMakeLists.txt / package.xml la generación de la acción.
# ---------------------------------------------------------------------------
from Ros2_RPI_ESP.action import GoToXY  # ← Cambia <tu_paquete> por tu paquete real


# ===========================================================================
# PARÁMETROS DE CONTROL — Ajusta estos valores según tu robot
# ===========================================================================

# Ganancia proporcional para velocidad lineal (m/s por metro de error)
KP_LINEAR = 0.6

# Ganancia proporcional para velocidad angular (rad/s por radián de error)
KP_ANGULAR = 1.2

# Velocidad lineal máxima permitida (m/s)
MAX_LINEAR_VEL = 0.3

# Velocidad angular máxima permitida (rad/s)
MAX_ANGULAR_VEL = 1.0

# Velocidad lineal mínima para evitar movimientos imperceptibles (m/s)
MIN_LINEAR_VEL = 0.05

# Distancia al objetivo por debajo de la cual se considera "llegado" (m)
GOAL_TOLERANCE = 0.05  # 5 cm

# Frecuencia del bucle de control (Hz)
CONTROL_RATE_HZ = 20.0

# Topic donde la cámara publica la posición del robot
POSE_TOPIC = "/robot_pose"

# Topic donde se publican los comandos de velocidad
CMD_VEL_TOPIC = "/cmd_vel"

# Nombre de la acción
ACTION_NAME = "go_to_xy"


# ===========================================================================
# NODO PRINCIPAL
# ===========================================================================

class GoToXYNode(Node):

    def __init__(self):
        super().__init__("go_to_xy")

        # --- Posición actual del robot (actualizada por el suscriptor) ---
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_yaw: float = 0.0  # ángulo actual en radianes
        self.pose_received: bool = False  # flag: ¿ya tenemos al menos un dato?

        # --- Callback group compartido para acción + suscriptor ---
        # ReentrantCallbackGroup permite que el execute() del action corra
        # en paralelo con los callbacks del suscriptor.
        cb_group = ReentrantCallbackGroup()

        # --- Suscriptor a la posición del robot ---
        # La cámara publica geometry_msgs/Pose2D con (x, y, theta).
        # Si tu cámara publica otro tipo de mensaje, adapta este callback.
        self.pose_sub = self.create_subscription(
            Pose2D,
            POSE_TOPIC,
            self._pose_callback,
            10,
            callback_group=cb_group,
        )

        # --- Publicador de comandos de velocidad ---
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        # --- Action server ---
        self._action_server = ActionServer(
            self,
            GoToXY,
            ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f"Nodo go_to_xy iniciado.\n"
            f"  Escuchando pose en : {POSE_TOPIC}\n"
            f"  Publicando cmd_vel : {CMD_VEL_TOPIC}\n"
            f"  Tolerancia llegada : {GOAL_TOLERANCE} m"
        )

    # -----------------------------------------------------------------------
    # CALLBACKS DE POSICIÓN
    # -----------------------------------------------------------------------

    def _pose_callback(self, msg: Pose2D):
        """Actualiza la posición actual del robot."""
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_yaw = msg.theta
        self.pose_received = True

    # -----------------------------------------------------------------------
    # CALLBACKS DEL ACTION SERVER
    # -----------------------------------------------------------------------

    def _goal_callback(self, goal_request):
        """Acepta o rechaza un goal entrante."""
        self.get_logger().info(
            f"Goal recibido: ({goal_request.target_x:.3f}, {goal_request.target_y:.3f})"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Acepta cancelaciones."""
        self.get_logger().info("Cancelación solicitada.")
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        """
        Bucle principal de control.
        Se ejecuta mientras el robot no llegue al objetivo o se cancele.
        """
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y

        self.get_logger().info(
            f"Ejecutando movimiento hacia ({target_x:.3f}, {target_y:.3f})"
        )

        # Esperamos a tener datos de posición antes de empezar
        if not self.pose_received:
            self.get_logger().warn("Esperando primer dato de /robot_pose...")
            rate = self.create_rate(10)
            while not self.pose_received and rclpy.ok():
                rate.sleep()

        feedback_msg = GoToXY.Feedback()
        rate = self.create_rate(CONTROL_RATE_HZ)

        # -------------------------------------------------------------------
        # BUCLE DE CONTROL
        # -------------------------------------------------------------------
        while rclpy.ok():

            # 1. Comprobar cancelación
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                self.get_logger().info("Goal cancelado. Robot detenido.")
                return GoToXY.Result(success=False, message="Cancelado por el usuario.")

            # 2. Calcular error de posición
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.hypot(dx, dy)

            # 3. Comprobar si hemos llegado
            if distance < GOAL_TOLERANCE:
                self._stop_robot()
                goal_handle.succeed()
                self.get_logger().info(
                    f"¡Objetivo alcanzado! Distancia residual: {distance:.4f} m"
                )
                return GoToXY.Result(success=True, message="Objetivo alcanzado.")

            # 4. Calcular el ángulo hacia el objetivo
            angle_to_goal = math.atan2(dy, dx)

            # 5. Error angular (diferencia entre heading actual y dirección al goal)
            angle_error = _normalize_angle(angle_to_goal - self.current_yaw)

            # 6. Control proporcional
            linear_vel = _clamp(
                KP_LINEAR * distance,
                MIN_LINEAR_VEL,
                MAX_LINEAR_VEL,
            )
            angular_vel = _clamp(
                KP_ANGULAR * angle_error,
                -MAX_ANGULAR_VEL,
                MAX_ANGULAR_VEL,
            )

            # 7. Publicar comando de velocidad
            cmd = Twist()
            cmd.linear.x = linear_vel     # avance hacia delante
            cmd.angular.z = angular_vel   # giro sobre el eje Z
            self.cmd_pub.publish(cmd)

            # 8. Publicar feedback
            feedback_msg.distance_remaining = distance
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().debug(
                f"dist={distance:.3f}m  angle_err={math.degrees(angle_error):.1f}°  "
                f"vlin={linear_vel:.3f}  vang={angular_vel:.3f}"
            )

            rate.sleep()

        # Si rclpy deja de estar ok (shutdown)
        self._stop_robot()
        goal_handle.abort()
        return GoToXY.Result(success=False, message="Nodo apagado durante la ejecución.")

    # -----------------------------------------------------------------------
    # UTILIDADES INTERNAS
    # -----------------------------------------------------------------------

    def _stop_robot(self):
        """Publica un Twist vacío para detener el robot."""
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Robot detenido.")


# ===========================================================================
# FUNCIONES DE UTILIDAD
# ===========================================================================

def _normalize_angle(angle: float) -> float:
    """
    Normaliza un ángulo al rango [-π, π].
    Necesario para evitar giros de más de 180° cuando el error cruza ±π.
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(value: float, min_val: float, max_val: float) -> float:
    """Limita un valor entre [min_val, max_val]."""
    return max(min_val, min(max_val, value))


# ===========================================================================
# ENTRY POINT
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)

    node = GoToXYNode()

    # MultiThreadedExecutor necesario para que el action server y el
    # suscriptor corran en paralelo (el bucle de control es bloqueante).
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()