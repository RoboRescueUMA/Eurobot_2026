#!/usr/bin/env python3
"""
Nodo de publicación de cámara IP cenital
Adaptado para usar namespace y ROS_DOMAIN_ID
Ejecuta en: LAPTOP
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import threading
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class ZenitalCameraPublisher(Node):
    def __init__(self):
        super().__init__('zenital_camera_publisher')
        
        # Parámetros configurables
        self.declare_parameter('video_url', 'http://192.168.1.100:8080/video')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('jpeg_quality', 80)
        
        self.video_url = self.get_parameter('video_url').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        
        # Publishers con namespace (se añade automáticamente por launch)
        self.publisher_raw = self.create_publisher(Image, 'zenital/image_raw', 1)
        self.publisher_compressed = self.create_publisher(
            CompressedImage, 'zenital/image_raw/compressed', 1
        )
        
        self.bridge = CvBridge()
        
        self.get_logger().info(f'🎥 Conectando a cámara IP: {self.video_url}')
        self.cap = cv2.VideoCapture(self.video_url)
        
        if not self.cap.isOpened():
            self.get_logger().error('❌ No se puede conectar a la cámara IP')
            self.get_logger().error(f'   Verifica que la URL sea correcta: {self.video_url}')
            self.get_logger().error('   Verifica que la app IPCamera esté activa en el móvil')
            return
            
        # Reducir buffer para baja latencia
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Variables compartidas entre hilos
        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = True

        # Hilo para lectura continua (vaciar buffer)
        self.thread = threading.Thread(target=self.read_camera_thread, daemon=True)
        self.thread.start()

        # Timer para publicación a tasa controlada
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_callback)
        
        self.get_logger().info(f'✅ Sistema de baja latencia iniciado ({self.publish_rate} Hz)')

    def read_camera_thread(self):
        """Bucle de lectura rápida para vaciar buffer de la cámara"""
        while self.running and rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame
            else:
                self.get_logger().warn('⚠️  Fallo al leer frame, reintentando...')

    def publish_callback(self):
        """Publica el último frame disponible"""
        frame_to_send = None
        with self.lock:
            if self.latest_frame is not None:
                frame_to_send = self.latest_frame.copy()
        
        if frame_to_send is not None:
            timestamp = self.get_clock().now().to_msg()

            # Publicar RAW
            try:
                msg_raw = self.bridge.cv2_to_imgmsg(frame_to_send, encoding="bgr8")
                msg_raw.header.stamp = timestamp
                msg_raw.header.frame_id = "camera_zenital"
                self.publisher_raw.publish(msg_raw)
            except Exception as e:
                self.get_logger().error(f'Error en conversión RAW: {e}')

            # Publicar COMPRESSED
            try:
                msg_compressed = CompressedImage()
                msg_compressed.header.stamp = timestamp
                msg_compressed.header.frame_id = "camera_zenital"
                msg_compressed.format = "jpeg"
                
                success, buffer = cv2.imencode(
                    '.jpg', frame_to_send, 
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                )
                
                if success:
                    msg_compressed.data = np.array(buffer).tobytes()
                    self.publisher_compressed.publish(msg_compressed)
            except Exception as e:
                self.get_logger().error(f'Error en compresión: {e}')

    def destroy_node(self):
        """Limpieza al cerrar"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZenitalCameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Deteniendo cámara...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
