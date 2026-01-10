import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class ZenitalCameraPublisher(Node):
    def __init__(self):
        super().__init__('zenital_camera_publisher')
        
        self.declare_parameter('video_url', 'http://192.168.100.60:5000/video_feed')
        self.video_url = self.get_parameter('video_url').get_parameter_value().string_value
        
        self.publisher_raw = self.create_publisher(Image, 'zenital/image_raw', 10)
        self.publisher_compressed = self.create_publisher(CompressedImage, 'zenital/image_raw/compressed', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        
        self.get_logger().info(f'Conectando a cámara cenital en: {self.video_url} ...')
        self.cap = cv2.VideoCapture(self.video_url)
        
        if not self.cap.isOpened():
            self.get_logger().error('¡Error! No se puede conectar.')
        else:
            self.get_logger().info('✅ Conexión exitosa. Forzando envío COMPRESSED.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            timestamp = self.get_clock().now().to_msg()

            # 1. RAW
            # (Lo dejamos siempre activo por si acaso)
            msg_raw = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg_raw.header.stamp = timestamp
            msg_raw.header.frame_id = "camera_link"
            self.publisher_raw.publish(msg_raw)

            # 2. COMPRESSED (SIN CHECK, SIEMPRE ENVÍA)
            # -----------------------------------------------------------
            # Eliminamos el 'if self.publisher_compressed.get_subscription_count()...'
            
            msg_compressed = CompressedImage()
            msg_compressed.header.stamp = timestamp
            msg_compressed.header.frame_id = "camera_link"
            msg_compressed.format = "jpeg"
            
            success, buffer = cv2.imencode('.jpg', frame)
            
            if success:
                msg_compressed.data = np.array(buffer).tobytes()
                self.publisher_compressed.publish(msg_compressed)
                # self.get_logger().info('Enviando...') # Descomenta si quieres ver spam en la terminal

        else:
            self.get_logger().warn('Reconectando...')
            self.cap.release()
            self.cap = cv2.VideoCapture(self.video_url)

def main(args=None):
    rclpy.init(args=args)
    node = ZenitalCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
