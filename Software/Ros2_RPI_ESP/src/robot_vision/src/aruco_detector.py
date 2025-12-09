#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Suscripción
        self.subscription = self.create_subscription(
            Image,
            '/camera_node/image_raw',
            self.image_callback,
            10)
        
        # Publicador
        self.publisher_ = self.create_publisher(Image, '/camera_node/aruco_image', 10)
        
        self.br = CvBridge()

        # Diccionario ArUco 4x4
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.get_logger().info('Detector ArUco iniciado! Buscando marcadores DICT_4X4_50...')

    def image_callback(self, msg):
        try:
            current_frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error conversion: {e}')
            return

        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(current_frame, corners, ids)
            # Imprimir IDs encontrados
            self.get_logger().info(f'Detectado: {ids.flatten()}')

        self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
