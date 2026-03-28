#!/usr/bin/env python3
"""
Nodo de odometría para robot omnidireccional de 4 ruedas.

Suscripciones:
  - /encoders      (std_msgs/Float32MultiArray): velocidades [v1, v2, v3, v4] en m/s
  - /initial_pose  (geometry_msgs/Pose2D):       posición inicial (x, y, theta)

Publicaciones:
  - /pose2d        (geometry_msgs/Pose2D):        posición del robot (x, y, theta)

Disposición de ruedas (vista superior):
        FRENTE
    v1 ┌────┐ v2
       │    │
    v4 └────┘ v3
        ATRÁS
"""

import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Float32MultiArray # para velocidades de encoders
from geometry_msgs.msg import Pose2D


# ===================== PARÁMETROS ROBOT =====================
ANCHO = 0.200                    # m
LARGO = 0.153                    # m
R     = (ANCHO + LARGO) / 2.0   # radio cinemático = 0.1765 m

# ===================== PARÁMETROS ENCODER =====================
DIAMETRO_RUEDA   = 0.06                          # m  (6 cm)
CIRCUNFERENCIA   = math.pi * DIAMETRO_RUEDA      # m  ≈ 0.1885 m
TICKS_POR_VUELTA = 360                           # ← ajusta a tu encoder


class Robot_Position(Node):

    def __init__(self):
        super().__init__('Robot_Position')

        # Estado del robot
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0   # 0 rad = mirando hacia +X (frente)

        self.last_time = self.get_clock().now()

        # Publicador
        self.pose2d_pub = self.create_publisher(Pose2D, '/pose2d', 10)

        # Suscriptores
        self.create_subscription(
            Float32MultiArray,
            '/encoders',
            self.encoders_callback,
            10
        )
        self.create_subscription(
            Pose2D,
            '/initial_pose',
            self.initial_pose_callback,
            10
        )

        self.get_logger().info(f'OdomNode listo — R={R:.4f} m')

    # ----------------------------------------------------------
    def initial_pose_callback(self, msg: Pose2D):
        self.x     = msg.x
        self.y     = msg.y
        self.theta = 0.0  
        self.last_time = self.get_clock().now()
        self.get_logger().info(
            f'Pose inicial: x={self.x:.3f} m, y={self.y:.3f} m, '
            f'θ={math.degrees(self.theta):.2f}°'
        )

    # ----------------------------------------------------------
    def encoders_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().error(
                f'Se esperaban 4 velocidades, se recibieron {len(msg.data)}.'
            )
            return

        # Conversión ticks/s → m/s
        # v(m/s) = ticks_por_segundo * (circunferencia / ticks_por_vuelta)
        def ticks_to_ms(ticks_s):
            return ticks_s * (CIRCUNFERENCIA / TICKS_POR_VUELTA)

        v1 = ticks_to_ms(msg.data[0])
        v2 = ticks_to_ms(msg.data[1])
        v3 = ticks_to_ms(msg.data[2])
        v4 = ticks_to_ms(msg.data[3])

        # Δt
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # Cinemática directa → velocidades en marco robot
        vx_r  =  ( v1 + v2 + v3 + v4) / 4.0
        vy_r  =  (-v1 + v2 + v3 - v4) / 4.0
        omega =  (-v1 + v2 - v3 + v4) / (4.0 * R)

        # Integración Euler → marco global
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)

        self.x     += (vx_r * cos_t - vy_r * sin_t) * dt
        self.y     += (vx_r * sin_t + vy_r * cos_t) * dt
        self.theta += omega * dt

        # Normalizar θ a [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publicar
        pose = Pose2D()
        pose.x     = self.x
        pose.y     = self.y
        pose.theta = self.theta
        self.pose2d_pub.publish(pose)


# ----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Robot_Position()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()