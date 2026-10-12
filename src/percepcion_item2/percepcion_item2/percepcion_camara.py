#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from arm_broker_interfaces.srv import LocalizarObjeto


class PercepcionCamara(Node):
    def __init__(self):
        super().__init__('percepcion_camara')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('area_min_px', 500.0)
        self.camera_index = int(self.get_parameter('camera_index').value)
        self.area_min = float(self.get_parameter('area_min_px').value)
        self.rangos = {
            'rojo': [(np.array([0,103,172]),np.array([2,255,255])),(np.array([170,103,172]),np.array([180,255,255]))],
            'verde': [(np.array([54,109,78]),np.array([77,255,255]))],
            'azul': [(np.array([92,100,62]),np.array([121,251,255]))],
            'amarillo': [(np.array([26,100,91]),np.array([32,255,255]))],
        }
        self.servicio = self.create_service(LocalizarObjeto, '/localizar_objeto', self.localizar_callback)
        self.get_logger().info(f'percepcion_camara lista · camera_index={self.camera_index}')

    def capturar(self):
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        frame = None
        for _ in range(5):
            ok, frame = cap.read()
            if not ok:
                frame = None
                break
        cap.release()
        return frame

    def detectar(self, frame, color):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mascara = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for bajo, alto in self.rangos[color]:
            mascara = cv2.bitwise_or(mascara, cv2.inRange(hsv, bajo, alto))
        kernel = np.ones((5,5), np.uint8)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)
        contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contornos:
            return None
        contorno = max(contornos, key=cv2.contourArea)
        area = float(cv2.contourArea(contorno))
        if area < self.area_min:
            return None
        m = cv2.moments(contorno)
        if abs(m['m00']) < 1e-9:
            return None
        return int(m['m10']/m['m00']), int(m['m01']/m['m00']), area

    def localizar_callback(self, request, response):
        color = request.color.strip().lower()
        if color not in self.rangos:
            response.success = False
            response.message = f'color no soportado: {color}'
            return response
        frame = self.capturar()
        if frame is None:
            response.success = False
            response.message = 'no se pudo abrir/capturar la camara'
            return response
        resultado = self.detectar(frame, color)
        if resultado is None:
            response.success = False
            response.message = f'objeto {color} no detectado'
            return response
        cx, cy, area = resultado
        response.success = True
        # Igual que el repo actual: falta calibracion pixel -> XYZ.
        response.x_mm = 0.0; response.y_mm = 0.0; response.z_mm = 0.0
        response.pixel_x = cx; response.pixel_y = cy; response.area_px = area
        response.message = f'{color} detectado; centro=({cx},{cy}); XYZ pendiente de calibracion'
        self.get_logger().info(f'DETECT color={color} pixel=({cx},{cy}) area={area:.1f}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PercepcionCamara()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
