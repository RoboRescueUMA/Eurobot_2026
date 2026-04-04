#!/usr/bin/env python3
"""
field_navigator.py - Navegacion autonomo hacia objetivo usando poses absolutas

Flujo:
  1. Suscribe a robot_pose (absoluta del campo)
  2. Suscribe a {target}_pose (blue_box_pose o yellow_box_pose)
  3. Calcula vector error en marco del mundo
  4. Transforma al marco del robot usando robot_theta
  5. Control PI de posicion -> velocidades (m/s)
  6. Publica cmd_vel_laptop

Sistema de coordenadas:
  - Poses en cm (desde field_localizer)
  - Velocidades en m/s (para ESP32)
  - theta en grados

Ejecuta en: LAPTOP
"""

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
import math
import time
from geometry_msgs.msg import Pose2D, Twist


class FieldNavigator(Node):
    def __init__(self):
        super().__init__("field_navigator")

        # --- Parametros ---
        self.declare_parameter("target", "yellow_box")
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("kp_position", 0.8)
        self._declare_param_once("ki_position", 0.1)
        self._declare_param_once("use_angular_control", False)
        self._declare_param_once("kp_angular", 2.0)
        self._declare_param_once("max_angular_speed", 1.5)
        self._declare_param_once("angle_tolerance_deg", 15.0)
        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("min_linear_speed", 0.0)
        self.declare_parameter("detection_timeout", 1.0)
        self.declare_parameter("deceleration_zone", 0.50)
        self.declare_parameter("namespace", "/roborescue")
        self.declare_parameter("command_topic", "cmd_vel_laptop")
        self.declare_parameter("preclear_enabled", True)
        self.declare_parameter(
            "preclear_waypoints",
            "250,77;284,77;278,157",
        )
        self.declare_parameter("preclear_tolerance_cm", 10.0)
        self._declare_param_once("preclear_min_speed", 0.4)
        self._declare_param_once("ki_preclear", 0.05)

        self.target = self.get_parameter("target").get_parameter_value().string_value
        self.goal_tolerance = (
            self.get_parameter("goal_tolerance").get_parameter_value().double_value
        )
        self.kp = self.get_parameter("kp_position").get_parameter_value().double_value
        self.ki = self.get_parameter("ki_position").get_parameter_value().double_value
        self.use_angular_control = (
            self.get_parameter("use_angular_control").get_parameter_value().bool_value
        )
        self.kp_angular = (
            self.get_parameter("kp_angular").get_parameter_value().double_value
        )
        self.max_angular_speed = (
            self.get_parameter("max_angular_speed").get_parameter_value().double_value
        )
        self.angle_tolerance_deg = (
            self.get_parameter("angle_tolerance_deg").get_parameter_value().double_value
        )
        self.max_speed = (
            self.get_parameter("max_linear_speed").get_parameter_value().double_value
        )
        self.min_speed = (
            self.get_parameter("min_linear_speed").get_parameter_value().double_value
        )
        self.timeout = (
            self.get_parameter("detection_timeout").get_parameter_value().double_value
        )
        self.decel_zone = (
            self.get_parameter("deceleration_zone").get_parameter_value().double_value
        )
        ns = self.get_parameter("namespace").get_parameter_value().string_value
        cmd_topic = (
            self.get_parameter("command_topic").get_parameter_value().string_value
        )

        self.preclear_enabled = (
            self.get_parameter("preclear_enabled").get_parameter_value().bool_value
        )
        preclear_str = (
            self.get_parameter("preclear_waypoints").get_parameter_value().string_value
        )
        self.preclear_tolerance_cm = (
            self.get_parameter("preclear_tolerance_cm")
            .get_parameter_value()
            .double_value
        )
        self.preclear_tolerance_m = self.preclear_tolerance_cm / 100.0
        self.preclear_min_speed = (
            self.get_parameter("preclear_min_speed").get_parameter_value().double_value
        )
        self.ki_preclear = (
            self.get_parameter("ki_preclear").get_parameter_value().double_value
        )
        self.preclear_waypoints = self._parse_waypoints(preclear_str)
        if not self.preclear_waypoints:
            self.preclear_enabled = False
        self.preclear_index = 0
        self.preclear_done = not self.preclear_enabled
        self.log_throttle_interval = 2.0
        self._log_timestamps = {"robot": 0.0, "target": 0.0, "status": 0.0}
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.control_dt = 0.05

        # --- Validar target ---
        if self.target not in ["blue_box", "yellow_box"]:
            self.get_logger().error(
                f"Target invalido: {self.target}. Usando 'blue_box'"
            )
            self.target = "blue_box"

        # --- Estado ---
        self.robot_pose = None
        self.target_pose = None
        self.last_robot_time = None
        self.last_target_time = None
        self.goal_reached = False

        # --- Subscripciones ---
        self.sub_robot = self.create_subscription(
            Pose2D, "robot_pose", self.robot_callback, 10
        )
        self.sub_target = self.create_subscription(
            Pose2D, f"{self.target}_pose", self.target_callback, 10
        )

        # --- Publicador ---
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)

        # --- Timer de control (20 Hz) ---
        self.timer = self.create_timer(0.05, self.control_loop)

        # --- Timer de verificacion timeout (10 Hz) ---
        self.timer_timeout = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info(
            f"FieldNavigator listo. Target: {self.target}, "
            f"Tolerancia: {self.goal_tolerance * 100:.0f}cm, "
            f"Kp: {self.kp}, Ki: {self.ki}, "
            f"MaxSpeed: {self.max_speed}m/s, MinSpeed: {self.min_speed}m/s, "
            f"AngularControl: {self.use_angular_control}"
        )
        if self.preclear_enabled:
            self.get_logger().info(
                f"Despeje inicial activado con {len(self.preclear_waypoints)} waypoints"
            )

    def robot_callback(self, msg: Pose2D):
        self.robot_pose = msg
        self.last_robot_time = time.time()

    def target_callback(self, msg: Pose2D):
        self.target_pose = msg
        self.last_target_time = time.time()

    def check_timeout(self):
        now = time.time()
        robot_timeout = (
            self.last_robot_time is None or (now - self.last_robot_time) > self.timeout
        )
        target_timeout = (
            self.last_target_time is None
            or (now - self.last_target_time) > self.timeout
        )

        if not self.preclear_done:
            if robot_timeout:
                self.send_stop()
                self.get_logger().warn("Timeout: sin datos de robot_pose")
            return

        if robot_timeout or target_timeout:
            if not self.goal_reached:
                self.send_stop()
                if robot_timeout:
                    self.get_logger().warn("Timeout: sin datos de robot_pose")
                if target_timeout:
                    self.get_logger().warn(f"Timeout: sin datos de {self.target}_pose")

    def control_loop(self):
        if self.robot_pose is None:
            self._throttled_info("robot", "Esperando robot_pose...")
            return

        robot_x = self.robot_pose.x
        robot_y = self.robot_pose.y
        robot_theta_deg = self.robot_pose.theta

        using_preclear = False
        if not self.preclear_done and self.preclear_waypoints:
            target_x, target_y = self.preclear_waypoints[self.preclear_index]
            using_preclear = True
        else:
            if self.target_pose is None:
                self._throttled_info("target", f"Esperando {self.target}_pose...")
                return
            target_x = self.target_pose.x
            target_y = self.target_pose.y

        error_x_world = target_x - robot_x
        error_y_world = target_y - robot_y
        distancia_cm = math.sqrt(error_x_world**2 + error_y_world**2)
        distancia_m = distancia_cm / 100.0

        if using_preclear and distancia_m < self.preclear_tolerance_m:
            self.send_stop()
            self.integral_x = 0.0
            self.integral_y = 0.0
            self.get_logger().info(
                f"Waypoint {self.preclear_index + 1}/{len(self.preclear_waypoints)} "
                f" alcanzado ({target_x:.0f},{target_y:.0f})cm"
            )
            self.preclear_index += 1
            if self.preclear_index >= len(self.preclear_waypoints):
                self.preclear_done = True
                self.goal_reached = False
                self.get_logger().info("Zona despejada; iniciando fase de exploración")
            return

        if distancia_m < self.goal_tolerance and not using_preclear:
            if not self.goal_reached:
                self.get_logger().info(f"OBJETIVO ALCANZADO: {distancia_m * 100:.1f}cm")
                self.goal_reached = True
                self.send_stop()
            return

        self.goal_reached = False

        theta_rad = math.radians(robot_theta_deg)
        cos_t = math.cos(theta_rad)
        sin_t = math.sin(theta_rad)

        error_x_robot = (error_x_world * cos_t + error_y_world * sin_t) / 100.0
        error_y_robot = (-error_x_world * sin_t + error_y_world * cos_t) / 100.0

        vx = self.kp * error_x_robot
        vy = self.kp * error_y_robot

        ki_use = self.ki_preclear if using_preclear else self.ki
        if not using_preclear:
            self.integral_x = 0.0
            self.integral_y = 0.0
        else:
            self.integral_x += error_x_robot * self.control_dt
            self.integral_y += error_y_robot * self.control_dt
            self.integral_x = max(-1.0, min(1.0, self.integral_x))
            self.integral_y = max(-1.0, min(1.0, self.integral_y))
            vx += ki_use * self.integral_x
            vy += ki_use * self.integral_y

        if self.use_angular_control:
            target_angle_rad = math.atan2(error_y_world, error_x_world)
            robot_angle_rad = math.radians(robot_theta_deg)
            angle_error_rad = target_angle_rad - robot_angle_rad
            angle_error_rad = math.atan2(
                math.sin(angle_error_rad), math.cos(angle_error_rad)
            )
            angle_error_deg = math.degrees(angle_error_rad)

            wz = self.kp_angular * angle_error_rad
            if abs(wz) > self.max_angular_speed:
                wz = self.max_angular_speed * (1 if wz > 0 else -1)

            aligned = abs(angle_error_deg) < self.angle_tolerance_deg
        else:
            wz = 0.0
            angle_error_deg = 0.0
            aligned = True

        if distancia_m < self.decel_zone:
            factor = distancia_m / self.decel_zone
            factor = max(0.5, factor)
            vx *= factor
            vy *= factor

        speed = math.sqrt(vx**2 + vy**2)
        max_speed = self.max_speed
        min_speed = self.min_speed
        if using_preclear:
            min_speed = min(self.preclear_min_speed, min_speed)

        if not aligned:
            vx *= 0.3
            vy *= 0.3

        if speed > max_speed and speed > 1e-6:
            scale = max_speed / speed
            vx *= scale
            vy *= scale
            speed = max_speed
        elif speed < min_speed and speed > 1e-6:
            scale = min_speed / speed
            vx *= scale
            vy *= scale
            speed = min_speed

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self.pub_cmd.publish(cmd)
        self._throttled_info(
            "status",
            f"Navegando: dist={distancia_m * 100:.0f}cm | "
            f"error=({error_x_world:.0f},{error_y_world:.0f})cm | "
            f"angle_err={angle_error_deg:.1f}deg | "
            f"cmd=({vx:.2f},{vy:.2f})m/s {wz:.2f}rad/s",
        )

    def _throttled_info(self, key, message):
        now = time.time()
        last = self._log_timestamps.get(key, 0.0)
        if now - last >= self.log_throttle_interval:
            self._log_timestamps[key] = now
            self.get_logger().info(message)

    def send_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)

    def _parse_waypoints(self, raw: str):
        waypoints = []
        if not raw:
            return waypoints
        for chunk in raw.split(";"):
            chunk = chunk.strip()
            if not chunk:
                continue
            try:
                x_str, y_str = chunk.split(",")
                waypoints.append((float(x_str), float(y_str)))
            except ValueError:
                self.get_logger().warn(
                    f"Waypoint '{chunk}' inválido. Formato esperado x,y"
                )
        return waypoints

    def _declare_param_once(self, name, default_value):
        try:
            self.declare_parameter(name, default_value)
        except ParameterAlreadyDeclaredException:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FieldNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Deteniendo field_navigator...")
    finally:
        node.send_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
