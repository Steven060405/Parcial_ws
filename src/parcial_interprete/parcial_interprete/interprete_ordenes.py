import re
import time

import requests
import rclpy
from rclpy.node import Node

from parcial_interfaces.srv import InterpretarOrden
from .clasificador import clasificar_local


class InterpreteOrdenes(Node):
    def __init__(self):
        super().__init__('interprete_ordenes')

        self.declare_parameter('laya_url', 'http://127.0.0.1:8000')
        self.declare_parameter('laya_preset', 'manual_prueba')
        self.declare_parameter('laya_model', 'typed-decisions')
        self.declare_parameter('timeout_s', 5.0)

        self.laya_url = str(self.get_parameter('laya_url').value).rstrip('/')
        self.laya_preset = str(self.get_parameter('laya_preset').value)
        self.laya_model = str(self.get_parameter('laya_model').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        self.http = requests.Session()
        self.http.trust_env = False

        self.servicio = self.create_service(
            InterpretarOrden,
            '/interpretar_orden',
            self.interpretar_callback,
        )

        self.get_logger().info('interprete_ordenes listo')
        self.get_logger().info(f'LAYA URL: {self.laya_url}')
        self.get_logger().info(f'LAYA preset/identificador: {self.laya_preset}')
        self.get_logger().info(f'LAYA modelo: {self.laya_model}')
        self.get_logger().info(f'Timeout: {self.timeout_s:.2f} s')

    def construir_preguntas(self):
        return {
            'accion': {
                'type': 'choice',
                'instructions': 'Selecciona exactamente la accion ordenada al robot.',
                'criteria': {
                    'recoger': 'Recoger, tomar, agarrar o coger un objeto.',
                    'depositar': 'Colocar, poner, llevar o depositar un objeto.',
                    'detener': 'Detener, parar o cancelar el movimiento.',
                },
            },
            'objeto': {
                'type': 'choice',
                'instructions': 'Selecciona exactamente el objeto mencionado.',
                'criteria': {
                    'cubo': 'El objeto es un cubo.',
                    'bloque': 'El objeto es un bloque.',
                    'botella': 'El objeto es una botella.',
                    'pelota': 'El objeto es una pelota.',
                },
            },
            'color': {
                'type': 'choice',
                'instructions': 'Selecciona exactamente el color mencionado.',
                'criteria': {
                    'rojo': 'Rojo o roja.',
                    'verde': 'Verde.',
                    'azul': 'Azul.',
                    'amarillo': 'Amarillo o amarilla.',
                },
            },
            'prioridad': {
                'type': 'choice',
                'instructions': 'Clasifica solo el nivel de prioridad expresado.',
                'criteria': {
                    'baja': 'Puede hacerse cuando sea posible, sin apuro o prioridad baja.',
                    'normal': 'Orden normal sin urgencia ni indicacion de espera.',
                    'alta': 'Rapido, importante o prioridad alta.',
                    'urgente': 'Emergencia, urgente, inmediatamente o ahora mismo.',
                },
            },
            'permitido': {
                'type': 'choice',
                'instructions': 'Decide si la orden es segura para un brazo robotico.',
                'criteria': {
                    'permitido': 'Recoger, colocar, mover o detener objetos sin causar dano.',
                    'no_permitido': 'Golpear, atacar, lastimar o causar dano.',
                },
            },
        }

    @staticmethod
    def obtener_inferencia_ms(respuesta_http):
        valor = respuesta_http.headers.get('X-Inference-Time-Ms')
        if valor is not None:
            try:
                return float(valor)
            except (TypeError, ValueError):
                pass

        server_timing = respuesta_http.headers.get('Server-Timing', '')
        coincidencia = re.search(r'inference\s*;\s*dur=([0-9.]+)', server_timing)
        if coincidencia:
            try:
                return float(coincidencia.group(1))
            except ValueError:
                pass
        return -1.0

    def consultar_laya(self, frase):
        url = self.laya_url + '/v1/systemone'
        payload = {
            'state': frase,
            'lang': 'es',
            'model': self.laya_model,
            'questions': self.construir_preguntas(),
        }

        inicio_http = time.perf_counter()
        respuesta_http = self.http.post(url, json=payload, timeout=self.timeout_s)
        http_total_ms = (time.perf_counter() - inicio_http) * 1000.0
        respuesta_http.raise_for_status()
        datos = respuesta_http.json()
        respuestas = datos.get('answers', {})

        def choice(nombre, defecto):
            return respuestas.get(nombre, {}).get('choice', defecto)

        prioridad_texto = choice('prioridad', 'normal')
        permitido_texto = choice('permitido', 'no_permitido')
        mapa_prioridad = {'baja': 0, 'normal': 1, 'alta': 2, 'urgente': 3}
        inferencia_laya_ms = self.obtener_inferencia_ms(respuesta_http)
        rtt_red_ms = (
            max(0.0, http_total_ms - inferencia_laya_ms)
            if inferencia_laya_ms >= 0.0
            else -1.0
        )

        return {
            'accion': str(choice('accion', 'desconocida')),
            'objeto': str(choice('objeto', 'desconocido')),
            'color': str(choice('color', 'desconocido')),
            'prioridad': int(mapa_prioridad.get(prioridad_texto, 1)),
            'permitido': bool(permitido_texto == 'permitido'),
            'http_total_ms': float(http_total_ms),
            'inferencia_laya_ms': float(inferencia_laya_ms),
            'rtt_red_ms': float(rtt_red_ms),
            'modelo_respuesta': str(datos.get('model', self.laya_model)),
        }

    def aplicar_keywords(self, frase, response, inicio, detalle, http_total_ms=-1.0):
        resultado = clasificar_local(frase)
        response.accion = resultado['accion']
        response.objeto = resultado['objeto']
        response.color = resultado['color']
        response.prioridad = int(resultado['prioridad'])
        response.permitido = bool(resultado['permitido'])
        response.degradado = True
        response.metodo = 'keywords'
        response.http_total_ms = float(http_total_ms)
        response.inferencia_laya_ms = -1.0
        response.rtt_red_ms = -1.0
        response.detalle = detalle
        response.tiempo_proceso_ms = (time.perf_counter() - inicio) * 1000.0
        return response

    def interpretar_callback(self, request, response):
        inicio = time.perf_counter()
        frase = request.frase.strip()
        self.get_logger().info(f'Frase recibida: {frase}')

        if not frase:
            return self.aplicar_keywords(frase, response, inicio, 'Frase vacia')

        resultado_local = clasificar_local(frase)
        inicio_http = time.perf_counter()

        try:
            resultado_laya = self.consultar_laya(frase)
            prioridad_laya = int(resultado_laya['prioridad'])
            permitido_laya = bool(resultado_laya['permitido'])
            permitido_local = bool(resultado_local['permitido'])
            prioridad_local = int(resultado_local['prioridad'])

            response.accion = resultado_laya['accion']
            response.objeto = resultado_laya['objeto']
            response.color = resultado_laya['color']
            # Requisito: la prioridad viene de la decision del modelo cuando LAYA responde.
            response.prioridad = prioridad_laya
            # Capa de seguridad local: solo puede vetar, nunca volver permitida una orden que LAYA rechazo.
            response.permitido = permitido_laya and permitido_local
            veto_local = permitido_laya and not permitido_local
            response.degradado = False
            response.metodo = 'laya+veto_local' if veto_local else 'laya'
            response.http_total_ms = resultado_laya['http_total_ms']
            response.inferencia_laya_ms = resultado_laya['inferencia_laya_ms']
            response.rtt_red_ms = resultado_laya['rtt_red_ms']
            response.detalle = (
                f'LAYA OK; modelo={resultado_laya["modelo_respuesta"]}; '
                f'preset={self.laya_preset}; prioridad_laya={prioridad_laya}; '
                f'prioridad_local={prioridad_local}; permitido_laya={permitido_laya}; '
                f'permitido_local={permitido_local}; veto_local={veto_local}'
            )

        except requests.exceptions.Timeout:
            http_fallido_ms = (time.perf_counter() - inicio_http) * 1000.0
            self.get_logger().warning('Timeout de LAYA. Activando fallback keywords.')
            return self.aplicar_keywords(
                frase, response, inicio,
                f'Fallback por timeout de LAYA; preset={self.laya_preset}',
                http_total_ms=http_fallido_ms,
            )
        except Exception as error:
            http_fallido_ms = (time.perf_counter() - inicio_http) * 1000.0
            self.get_logger().warning(
                f'Fallo de LAYA. Activando fallback keywords. Error={type(error).__name__}'
            )
            return self.aplicar_keywords(
                frase, response, inicio,
                f'Fallback por fallo de LAYA: {type(error).__name__}; preset={self.laya_preset}',
                http_total_ms=http_fallido_ms,
            )

        response.tiempo_proceso_ms = (time.perf_counter() - inicio) * 1000.0
        self.get_logger().info(
            f'Resultado -> accion={response.accion}, objeto={response.objeto}, '
            f'color={response.color}, prioridad={response.prioridad}, '
            f'permitido={response.permitido}, degradado={response.degradado}, '
            f'metodo={response.metodo}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    nodo = InterpreteOrdenes()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
