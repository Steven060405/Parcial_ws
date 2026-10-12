#!/usr/bin/env python3
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from parcial_interfaces.srv import InterpretarOrden
from arm_broker_interfaces.action import OrdenBrazo


class OrdenDesdeTexto(Node):
    def __init__(self):
        super().__init__('orden_desde_texto')
        self.interprete = self.create_client(InterpretarOrden, '/interpretar_orden')
        self.broker = ActionClient(self, OrdenBrazo, 'orden_brazo')

    def ejecutar(self, frase, cliente='operador', zona='zona_a'):
        if not self.interprete.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/interpretar_orden no disponible')
        req = InterpretarOrden.Request(); req.frase = frase
        fut = self.interprete.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        dec = fut.result()
        print(f'DECISION accion={dec.accion} objeto={dec.objeto} color={dec.color} prioridad={dec.prioridad} permitido={dec.permitido} metodo={dec.metodo}')
        if not dec.permitido:
            print('RECHAZADA antes de encolar: permitido=false')
            return 2
        if dec.accion == 'detener':
            print('Orden detener interpretada; no se encola como pick-and-place.')
            return 0
        if not self.broker.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('action /orden_brazo no disponible')
        goal = OrdenBrazo.Goal()
        goal.client_id = cliente
        goal.accion = dec.accion
        goal.objeto = dec.objeto
        goal.color = dec.color
        goal.zona_destino = zona
        # Requisito: prioridad tomada de la decision del modelo/servicio.
        goal.priority = max(0, min(255, int(dec.prioridad)))
        send = self.broker.send_goal_async(goal, feedback_callback=lambda m: print(f'FB {m.feedback.state} {m.feedback.stage} pos={m.feedback.queue_position}'))
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            print('Broker rechazo la orden')
            return 3
        result_f = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_f)
        result = result_f.result().result
        print(f'BROKER success={result.success} mensaje={result.message}')
        return 0 if result.success else 4


def main():
    frase = ' '.join(sys.argv[1:]).strip() if len(sys.argv) > 1 else input('Orden: ').strip()
    rclpy.init(); node = OrdenDesdeTexto()
    try:
        code = node.ejecutar(frase)
    finally:
        node.destroy_node(); rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
