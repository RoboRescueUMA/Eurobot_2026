#!/usr/bin/env python3
"""
GoToXY — Nodo ROS2 de navegación autónoma
Adaptado a la arquitectura Eurobot 2026
=========================================
Suscribe a:
  /roborescue/robot_pose  (geometry_msgs/Pose2D)
      x, y  → coordenadas en CENTÍMETROS (sistema absoluto del campo)
      theta → orientación en GRADOS (0° = eje X del campo)

Publica en:
  /roborescue/cmd_vel_laptop  (geometry_msgs/Twist)
      La Raspberry Pi retransmite este tópico como /roborescue/cmd_vel hacia el ESP32.
      Usa los tres campos: linear.x, linear.y (ruedas Mecanum), angular.z

Recibe el objetivo mediante:
  /roborescue/goal_pose  (geometry_msgs/Pose2D)
      x, y en centímetros (mismas unidades que robot_pose)

Uso desde terminal:
  ros2 topic pub --once /roborescue/goal_pose geometry_msgs/msg/Pose2D \
      "{x: 150.0, y: 80.0, theta: 0.0}"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import math


# ──────────────────────────────────────────────────────────
#  Parámetros de control
#
#  NOTA SOBRE UNIDADES:
#  Las poses llegan en centímetros. Las velocidades que publica
#  este nodo son normalizadas [-1.0, 1.0]: el ESP32 las mapea
#  a PWM internamente. No usamos m/s reales aquí.
# ──────────────────────────────────────────────────────────

# Ganancias proporcionales.
# El error llega en cm, la salida es velocidad normalizada.
# Con KP_LINEAR = 0.015: un error de 50 cm da salida 0.75 (bien dentro del rango)
KP_LINEAR  = 0.015   # (velocidad normalizada) / cm
KP_ANGULAR = 1.2     # (velocidad angular normalizada) / rad

# Saturación de salida. El ESP32 espera valores en [-1.0, 1.0]
MAX_LINEAR  = 1.0
MAX_ANGULAR = 1.0

# Tolerancias
DIST_TOLERANCE  = 5.0    # cm — se considera "llegado" cuando dist < este valor
ANGLE_TOLERANCE = 0.15   # rad (~8°) — umbral para corrección angular

# Frecuencia del bucle de control
CONTROL_HZ = 20


# ──────────────────────────────────────────────────────────
#  Nodo principal
# ──────────────────────────────────────────────────────────

class GoToXY(Node):

    def __init__(self):
        super().__init__('go_to_xy')

        # --- Suscripción a la pose actual desde el localizador ArUco ---
        # Publica el portátil tras aplicar la homografía.
        # x, y en cm; theta en GRADOS.
        self.pose_sub = self.create_subscription(
            Pose2D,
            '/roborescue/robot_pose',
            self.pose_callback,
            10
        )

        # --- Suscripción al objetivo ---
        self.goal_sub = self.create_subscription(
            Pose2D,
            '/roborescue/goal_pose',
            self.goal_callback,
            10
        )

        # --- Publicador hacia la Raspberry Pi ---
        # La RPi retransmite este tópico como /roborescue/cmd_vel hacia el ESP32
        # a través del agente micro-ROS.
        self.cmd_pub = self.create_publisher(
            Twist,
            '/roborescue/cmd_vel_laptop',
            10
        )

        # --- Estado interno ---
        self.current_x     = 0.0   # cm
        self.current_y     = 0.0   # cm
        self.current_theta = 0.0   # radianes (convertido al recibir)

        self.goal_x = None         # cm
        self.goal_y = None         # cm

        self.pose_received = False
        self._log_counter  = 0

        # --- Timer del bucle de control ---
        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.control_loop)

        self.get_logger().info(
            'GoToXY (Eurobot 2026) iniciado.\n'
            '  Pose:    /roborescue/robot_pose     (cm + grados)\n'
            '  Salida:  /roborescue/cmd_vel_laptop\n'
            '  Objetivo:/roborescue/goal_pose      (cm)'
        )


    # ──────────────────────────────────────────
    #  Callback: pose actual del robot
    # ──────────────────────────────────────────

    def pose_callback(self, msg: Pose2D):
        """
        Recibe la pose publicada por el localizador ArUco del portátil.

        CONVERSIÓN CRÍTICA:
        El localizador publica theta en GRADOS.
        Todo el cálculo de control necesita radianes, así que convertimos aquí
        una sola vez y el resto del código trabaja siempre en radianes.
        """
        self.current_x     = msg.x
        self.current_y     = msg.y
        self.current_theta = math.radians(msg.theta)   # grados → radianes
        self.pose_received = True


    # ──────────────────────────────────────────
    #  Callback: nuevo objetivo
    # ──────────────────────────────────────────

    def goal_callback(self, msg: Pose2D):
        """
        Activa cuando se publica un nuevo destino en /roborescue/goal_pose.
        Las coordenadas deben estar en centímetros, igual que robot_pose.
        """
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.get_logger().info(
            f'Nuevo objetivo: x={msg.x:.1f} cm, y={msg.y:.1f} cm'
        )


    # ──────────────────────────────────────────
    #  Bucle de control principal (Mecanum)
    # ──────────────────────────────────────────

    def control_loop(self):
        """
        Controlador proporcional para ruedas Mecanum (X-Drive).

        A diferencia de un robot diferencial, las ruedas Mecanum permiten
        moverse en cualquier dirección sin reorientar el chasis. Por eso
        calculamos vx, vy y wz SIMULTÁNEAMENTE en cada iteración,
        sin la lógica de "girar primero, avanzar después".

        El flujo es:
          1. Calcular error en el sistema de coordenadas del MUNDO (dx, dy en cm)
          2. Rotar ese error al sistema de coordenadas del ROBOT (usando theta)
          3. Aplicar ganancia proporcional → vx, vy normalizados
          4. Publicar Twist con los tres componentes
        """

        if not self.pose_received:
            return
        if self.goal_x is None or self.goal_y is None:
            return

        # ── 1. Error en el sistema del mundo ───────────────────────

        dx_world = self.goal_x - self.current_x   # cm
        dy_world = self.goal_y - self.current_y   # cm
        dist     = math.sqrt(dx_world**2 + dy_world**2)   # cm

        # ── Condición de parada ─────────────────────────────────────

        if dist < DIST_TOLERANCE:
            self.stop_robot()
            self.get_logger().info(
                f'Meta alcanzada. Error final: {dist:.1f} cm'
            )
            self.goal_x = None
            self.goal_y = None
            return

        # ── 2. Rotar el error al marco del robot ────────────────────
        #
        # El error (dx_world, dy_world) está en el sistema global del campo.
        # Para que el ESP32 sepa qué ruedas mover, necesitamos ese error
        # en el sistema LOCAL del robot (rotado por -theta).
        #
        # Rotación 2D inversa:
        #   vx_robot =  dx_world · cos(θ) + dy_world · sin(θ)
        #   vy_robot = -dx_world · sin(θ) + dy_world · cos(θ)
        #
        # Interpretación: vx es "cuánto está el objetivo hacia mi frente",
        # vy es "cuánto está el objetivo hacia mi lado izquierdo".

        cos_t = math.cos(self.current_theta)
        sin_t = math.sin(self.current_theta)

        vx_raw =  dx_world * cos_t + dy_world * sin_t
        vy_raw = -dx_world * sin_t + dy_world * cos_t

        # ── 3. Aplicar ganancia y saturar ──────────────────────────

        vx = max(-MAX_LINEAR, min(MAX_LINEAR, KP_LINEAR * vx_raw))
        vy = max(-MAX_LINEAR, min(MAX_LINEAR, KP_LINEAR * vy_raw))

        # ── 4. Corrección angular (opcional) ───────────────────────
        #
        # Con wz = 0 el robot mantiene su orientación mientras navega,
        # aprovechando al máximo el movimiento omnidireccional.
        #
        # Si necesitas que el robot apunte hacia el objetivo mientras avanza,
        # descomenta estas líneas:
        #
        # angle_to_goal = math.atan2(dy_world, dx_world)
        # angle_error   = self.normalize_angle(angle_to_goal - self.current_theta)
        # wz = max(-MAX_ANGULAR, min(MAX_ANGULAR, KP_ANGULAR * angle_error))

        wz = 0.0

        # ── 5. Publicar Twist ───────────────────────────────────────

        cmd = Twist()
        cmd.linear.x  = vx   # avance/retroceso (aprovecha ruedas Mecanum)
        cmd.linear.y  = vy   # desplazamiento lateral (NUEVO respecto a versión anterior)
        cmd.angular.z = wz   # giro sobre el eje vertical

        self.cmd_pub.publish(cmd)

        # Log de debug cada 0.5 s (10 iteraciones a 20 Hz)
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().debug(
                f'dist={dist:.1f}cm  vx={vx:.2f}  vy={vy:.2f}  wz={wz:.2f}'
            )


    # ──────────────────────────────────────────
    #  Utilidades
    # ──────────────────────────────────────────

    def stop_robot(self):
        """Publica Twist vacío → ESP32 entra en zona muerta → motores parados."""
        self.cmd_pub.publish(Twist())

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normaliza un ángulo al rango [-pi, pi]."""
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle


# ──────────────────────────────────────────────────────────
#  Punto de entrada
# ──────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GoToXY()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido por el usuario.')
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()