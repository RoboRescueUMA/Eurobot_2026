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
        
        # Ajusta la IP aquí si cambia
        self.declare_parameter('video_url', 'http://192.168.100.122:5000/video')
        self.video_url = self.get_parameter('video_url').get_parameter_value().string_value
        
        # QoS = 1 para no acumular basura
        self.publisher_raw = self.create_publisher(Image, 'zenital/image_raw', 1)
        self.publisher_compressed = self.create_publisher(CompressedImage, 'zenital/image_raw/compressed', 1)
        
        self.bridge = CvBridge()
        
        self.get_logger().info(f'Conectando a cámara: {self.video_url} ...')
        self.cap = cv2.VideoCapture(self.video_url)
        
        if not self.cap.isOpened():
            self.get_logger().error('¡Error! No se puede conectar.')
            return
            
        # --- TRUCO ANTI-LAG: BAJAR BUFFER INTERNO ---
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Variables para compartir la última foto entre hilos
        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = True

        # 1. HILO LIMPIADOR (Lee a toda velocidad para vaciar el buffer)
        self.thread = threading.Thread(target=self.read_camera_thread)
        self.thread.daemon = True
        self.thread.start()

        # 2. TEMPORIZADOR DE ENVÍO (Solo envía 10 veces por segundo)
        self.timer = self.create_timer(0.1, self.publish_callback)
        
        self.get_logger().info('✅ Sistema de baja latencia iniciado.')

    def read_camera_thread(self):
        """Este bucle corre a la velocidad máxima de la cámara (30fps) 
           para que no se acumulen imágenes viejas."""
        while self.running and rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame
            else:
                # Si falla, intentamos reconectar
                pass 

    def publish_callback(self):
        # Cogemos la última foto disponible
        frame_to_send = None
        with self.lock:
            if self.latest_frame is not None:
                frame_to_send = self.latest_frame.copy()
        
        if frame_to_send is not None:
            timestamp = self.get_clock().now().to_msg()

            # Publicar RAW (QoS 1 se encarga de tirar si sobra)
            msg_raw = self.bridge.cv2_to_imgmsg(frame_to_send, encoding="bgr8")
            msg_raw.header.stamp = timestamp
            msg_raw.header.frame_id = "camera_link"
            self.publisher_raw.publish(msg_raw)

            # Publicar COMPRESSED
            msg_compressed = CompressedImage()
            msg_compressed.header.stamp = timestamp
            msg_compressed.header.frame_id = "camera_link"
            msg_compressed.format = "jpeg"
            
            # Comprimimos calidad JPG al 50% para velocidad
            success, buffer = cv2.imencode('.jpg', frame_to_send, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            
            if success:
                msg_compressed.data = np.array(buffer).tobytes()
                self.publisher_compressed.publish(msg_compressed)

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        self.cap.release()
        super().destroy_node()

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
