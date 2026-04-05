#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

try:
    from gpiozero import AngularServo
    HARDWARE_DISPONIBLE = True
except ImportError:
    HARDWARE_DISPONIBLE = False

class ControladorGarra(Node):
    def __init__(self):
        super().__init__('controlador_garra')

        self.pin_elevacion = 18 
        self.pin_apertura  = 19 
        self.pin_giro      = 21 
        
        if HARDWARE_DISPONIBLE:
            self.servo_elevacion = AngularServo(self.pin_elevacion, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)
            self.servo_apertura  = AngularServo(self.pin_apertura,  min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)
            self.servo_giro      = AngularServo(self.pin_giro,      min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)
            self.get_logger().info("✅ Hardware de garra inicializado.")
        else:
            self.servo_elevacion = self.servo_apertura = self.servo_giro = None
            self.get_logger().warn("⚠️ MODO SIMULACIÓN. No hay servos, pero responderé 'LISTO'.")

        self.sub_cmd = self.create_subscription(String, "/roborescue/cmd_garra", self.comando_callback, 1)
        self.pub_status = self.create_publisher(String, "/roborescue/garra_status", 1)

        self.movimiento_en_curso = False
        self.tiempo_fin_movimiento = 0.0
        # TIMING DE PRUEBA: Si quieres que las pausas sean más rápidas para las pruebas de navegación, baja esto a 0.5
        self.TIEMPO_TRAYECTORIA_SERVO = 1.0 

        self.timer = self.create_timer(0.1, self.loop_estado_garra)

    def comando_callback(self, msg: String):
        datos = msg.data.split(',')
        if len(datos) != 3: return

        try:
            ang_elevacion = float(datos[0])
            ang_apertura  = float(datos[1])
            ang_giro      = float(datos[2])
            
            self.get_logger().info(f"Comando recibido: {ang_elevacion}º, {ang_apertura}º, {ang_giro}º")
            
            if HARDWARE_DISPONIBLE:
                self.servo_elevacion.angle = ang_elevacion
                self.servo_apertura.angle  = ang_apertura
                self.servo_giro.angle      = ang_giro
            
            self.tiempo_fin_movimiento = time.time() + self.TIEMPO_TRAYECTORIA_SERVO
            self.movimiento_en_curso = True
            
        except ValueError:
            pass

    def loop_estado_garra(self):
        if self.movimiento_en_curso and time.time() >= self.tiempo_fin_movimiento:
            self.movimiento_en_curso = False
            self.get_logger().info("Enviando 'LISTO' al cerebro.")
            msg = String()
            msg.data = "LISTO"
            self.pub_status.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodo = ControladorGarra()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally: nodo.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()