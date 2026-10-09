#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from arm_broker_interfaces.srv import LocalizarObjeto


class PercepcionCamara(Node):

    def __init__(self):
        super().__init__('percepcion_camara')

        # ============================================================
        # PARAMETROS
        # ============================================================

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('area_min_px', 500.0)

        self.camera_index = int(
            self.get_parameter('camera_index').value
        )

        self.area_min = float(
            self.get_parameter('area_min_px').value
        )

        # ============================================================
        # RANGOS HSV
        # ============================================================

        self.rangos = {

            'rojo': [
                (
                    np.array([0, 103, 172]),
                    np.array([2, 255, 255])
                ),
                (
                    np.array([170, 103, 172]),
                    np.array([180, 255, 255])
                ),
            ],

            'verde': [
                (
                    np.array([45, 20, 45]),
                    np.array([75, 255, 255])
                ),
            ],

            'azul': [
                (
                    np.array([92, 100, 62]),
                    np.array([121, 251, 255])
                ),
            ],

            'amarillo': [
                (
                    np.array([20, 70, 70]),
                    np.array([40, 255, 255])
                ),
            ],
        }

        # ============================================================
        # SERVICIO
        # ============================================================

        self.servicio = self.create_service(
            LocalizarObjeto,
            '/localizar_objeto',
            self.localizar_callback
        )

        self.get_logger().info(
            'percepcion_camara lista · '
            f'camera_index={self.camera_index} · '
            'V4L2/YUYV 640x480@30'
        )

    # ================================================================
    # CAMARA
    # ================================================================

    def capturar(self):

        self.get_logger().info(
            f'Abriendo camara index={self.camera_index}'
        )

        # La cámara USB del JetCobot fue validada usando
        # V4L2 + YUYV + 640x480 @ 30 FPS.
        cap = cv2.VideoCapture(
            self.camera_index,
            cv2.CAP_V4L2
        )

        if not cap.isOpened():

            self.get_logger().error(
                'No se pudo abrir la camara'
            )

            cap.release()
            return None

        # Formato validado físicamente.
        cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*'YUYV')
        )

        cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            640
        )

        cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            480
        )

        cap.set(
            cv2.CAP_PROP_FPS,
            30
        )

        # Reducir frames antiguos si el backend lo permite.
        cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1
        )

        frame = None

        # Leer varios frames para permitir que la cámara se estabilice.
        for intento in range(10):

            ok, frame_actual = cap.read()

            if ok and frame_actual is not None:

                frame = frame_actual

            else:

                self.get_logger().warning(
                    f'Frame no recibido '
                    f'intento={intento + 1}/10'
                )

        cap.release()

        if frame is None:

            self.get_logger().error(
                'La camara abrio pero no entrego frames'
            )

            return None

        self.get_logger().info(
            f'Frame recibido '
            f'{frame.shape[1]}x{frame.shape[0]}'
        )

        return frame

    # ================================================================
    # DETECCION DE COLOR
    # ================================================================

    def detectar(self, frame, color):

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        mascara = np.zeros(
            hsv.shape[:2],
            dtype=np.uint8
        )

        for bajo, alto in self.rangos[color]:

            parcial = cv2.inRange(
                hsv,
                bajo,
                alto
            )

            mascara = cv2.bitwise_or(
                mascara,
                parcial
            )

        # Limpiar ruido.
        kernel = np.ones(
            (5, 5),
            np.uint8
        )

        mascara = cv2.morphologyEx(
            mascara,
            cv2.MORPH_OPEN,
            kernel
        )

        mascara = cv2.morphologyEx(
            mascara,
            cv2.MORPH_CLOSE,
            kernel
        )

        contornos, _ = cv2.findContours(
            mascara,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contornos:
            return None

        contorno = max(
            contornos,
            key=cv2.contourArea
        )

        area = float(
            cv2.contourArea(contorno)
        )

        if area < self.area_min:
            return None

        momentos = cv2.moments(
            contorno
        )

        if abs(momentos['m00']) < 1e-9:
            return None

        cx = int(
            momentos['m10']
            / momentos['m00']
        )

        cy = int(
            momentos['m01']
            / momentos['m00']
        )

        return cx, cy, area

    # ================================================================
    # SERVICIO /localizar_objeto
    # ================================================================

    def localizar_callback(
        self,
        request,
        response
    ):

        color = (
            request.color
            .strip()
            .lower()
        )

        objeto = (
            request.objeto
            .strip()
            .lower()
        )

        self.get_logger().info(
            f'LOCALIZAR objeto={objeto} '
            f'color={color}'
        )

        # ------------------------------------------------------------
        # VALIDAR COLOR
        # ------------------------------------------------------------

        if color not in self.rangos:

            response.success = False

            response.message = (
                f'color no soportado: {color}'
            )

            return response

        # ------------------------------------------------------------
        # CAPTURAR
        # ------------------------------------------------------------

        frame = self.capturar()

        if frame is None:

            response.success = False

            response.message = (
                'no se pudo abrir/capturar '
                'la camara'
            )

            return response

        # ------------------------------------------------------------
        # DETECTAR
        # ------------------------------------------------------------

        resultado = self.detectar(
            frame,
            color
        )

        if resultado is None:

            response.success = False

            response.message = (
                f'objeto {color} '
                'no detectado'
            )

            response.x_mm = 0.0
            response.y_mm = 0.0
            response.z_mm = 0.0

            response.pixel_x = -1
            response.pixel_y = -1
            response.area_px = 0.0

            self.get_logger().warning(
                f'NO_DETECT '
                f'color={color}'
            )

            return response

        # ------------------------------------------------------------
        # RESULTADO
        # ------------------------------------------------------------

        cx, cy, area = resultado

        response.success = True

        # En P2 estamos usando poses articulares J1...J6
        # ya medidas para cada objeto.
        # La cámara solamente valida el color y su presencia.
        response.x_mm = 0.0
        response.y_mm = 0.0
        response.z_mm = 0.0

        response.pixel_x = int(cx)
        response.pixel_y = int(cy)
        response.area_px = float(area)

        response.message = (
            f'{color} detectado; '
            f'centro=({cx},{cy}); '
            f'area={area:.1f}px'
        )

        self.get_logger().info(
            f'DETECT '
            f'color={color} '
            f'pixel=({cx},{cy}) '
            f'area={area:.1f}'
        )

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
