#!/usr/bin/env python3
import base64
import json
import mimetypes
import os
import sys
import time
from pathlib import Path

import requests
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from parcial_interfaces.srv import InterpretarOrden
from arm_broker_interfaces.action import OrdenBrazo


class TranscriptorVoz(Node):
    def __init__(self):
        super().__init__('transcriptor_voz')
        self.declare_parameter('audio_file', '')
        self.declare_parameter('google_model', 'gemini-2.5-flash')
        self.declare_parameter('timeout_s', 12.0)
        self.declare_parameter('enqueue_broker', False)
        self.declare_parameter('client_id', 'voz')
        self.declare_parameter('zona_destino', 'zona_entrega')
        self.audio_file = str(self.get_parameter('audio_file').value)
        self.model = str(self.get_parameter('google_model').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.enqueue_broker = bool(self.get_parameter('enqueue_broker').value)
        self.client_id = str(self.get_parameter('client_id').value)
        self.zona_destino = str(self.get_parameter('zona_destino').value)
        self.cli = self.create_client(InterpretarOrden, '/interpretar_orden')
        self.broker = ActionClient(self, OrdenBrazo, 'orden_brazo')
        self.http = requests.Session()
        self.http.trust_env = True

    def transcribir_google(self, archivo):
        api_key = os.getenv('GOOGLE_API_KEY', '').strip()
        if not api_key:
            raise RuntimeError('GOOGLE_API_KEY no definida')
        path = Path(archivo).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(path)
        mime = mimetypes.guess_type(str(path))[0] or 'audio/wav'
        data = base64.b64encode(path.read_bytes()).decode('ascii')
        url = f'https://generativelanguage.googleapis.com/v1beta/models/{self.model}:generateContent'
        payload = {
            'contents': [{
                'parts': [
                    {'text': 'Transcribe exactamente este audio en español. Devuelve solo la transcripción, sin explicaciones.'},
                    {'inline_data': {'mime_type': mime, 'data': data}},
                ]
            }],
            'generationConfig': {'temperature': 0.0},
        }
        t0 = time.perf_counter()
        resp = self.http.post(url, params={'key': api_key}, json=payload, timeout=self.timeout_s)
        trans_ms = (time.perf_counter() - t0) * 1000.0
        resp.raise_for_status()
        obj = resp.json()
        texto = obj['candidates'][0]['content']['parts'][0]['text'].strip()
        if not texto:
            raise RuntimeError('transcripcion vacia')
        return texto, trans_ms

    def interpretar(self, texto):
        if not self.cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/interpretar_orden no disponible')
        req = InterpretarOrden.Request()
        req.frase = texto
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        if not fut.done():
            raise TimeoutError('/interpretar_orden sin respuesta')
        if fut.exception():
            raise RuntimeError(str(fut.exception()))
        return fut.result()

    def correr(self):
        t_total = time.perf_counter()
        modo = 'voz'
        trans_ms = -1.0
        try:
            if not self.audio_file:
                raise RuntimeError('audio_file vacio')
            texto, trans_ms = self.transcribir_google(self.audio_file)
            print(f'TRANSCRIPCION: {texto}')
        except Exception as e:
            modo = 'texto_fallback'
            self.get_logger().warning(f'Fallo transcripcion ({type(e).__name__}: {e}). Modo texto.')
            texto = input('Escriba la orden: ').strip()
        if not texto:
            raise RuntimeError('orden vacia')
        r = self.interpretar(texto)
        total_ms = (time.perf_counter() - t_total) * 1000.0
        broker_result = 'no_enviado'
        if self.enqueue_broker:
            if not r.permitido:
                broker_result = 'rechazado_antes_de_cola'
            elif r.accion in ('recoger', 'depositar'):
                if self.broker.wait_for_server(timeout_sec=10.0):
                    goal = OrdenBrazo.Goal()
                    goal.client_id = self.client_id
                    goal.accion = r.accion
                    goal.objeto = r.objeto
                    goal.color = r.color
                    goal.zona_destino = self.zona_destino
                    goal.priority = max(0, min(255, int(r.prioridad)))
                    sf = self.broker.send_goal_async(goal)
                    rclpy.spin_until_future_complete(self, sf)
                    handle = sf.result()
                    if handle and handle.accepted:
                        rf = handle.get_result_async()
                        rclpy.spin_until_future_complete(self, rf)
                        rr = rf.result().result
                        broker_result = f'{rr.success}:{rr.message}'
                    else:
                        broker_result = 'broker_rechazo_goal'
                else:
                    broker_result = 'broker_no_disponible'
            else:
                broker_result = f'accion_{r.accion}_no_encolada'

        registro = {
            'modo': modo,
            'audio_file': self.audio_file,
            'texto': texto,
            'transcripcion_ms': trans_ms,
            'accion': r.accion,
            'objeto': r.objeto,
            'color': r.color,
            'prioridad': int(r.prioridad),
            'permitido': bool(r.permitido),
            'metodo_interprete': r.metodo,
            'decision_ms': r.tiempo_proceso_ms,
            'total_hasta_decision_ms': total_ms,
            'broker_result': broker_result,
        }
        print(json.dumps(registro, indent=2, ensure_ascii=False))
        return registro


def main(args=None):
    rclpy.init(args=args)
    node = TranscriptorVoz()
    try:
        node.correr()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'{type(e).__name__}: {e}')
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
