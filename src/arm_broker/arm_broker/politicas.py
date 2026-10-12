"""Politicas de cola para el broker del JetCobot."""
import threading
import time


class Pedido:
    def __init__(self, goal_handle, client_id, priority, joint_positions=None,
                 tipo='joint', accion='', objeto='', color='', zona_destino=''):
        self.goal_handle = goal_handle
        self.goal_id = bytes(goal_handle.goal_id.uuid).hex()[:12]
        self.client_id = str(client_id)
        self.priority = int(priority)
        self.tipo = str(tipo)
        self.joint_positions = list(joint_positions) if joint_positions is not None else []
        self.accion = str(accion)
        self.objeto = str(objeto)
        self.color = str(color)
        self.zona_destino = str(zona_destino)
        self.t_llegada = time.time()
        self.t_inicio_ejec = None
        self.fin = threading.Event()
        self.resultado = None

    @property
    def espera_s(self):
        fin = self.t_inicio_ejec if self.t_inicio_ejec else time.time()
        return fin - self.t_llegada


class Politica:
    nombre = 'base'
    def siguiente(self, pendientes):
        raise NotImplementedError
    def atendido(self, pedido):
        pass


class FIFO(Politica):
    nombre = 'fifo'
    def siguiente(self, pendientes):
        if not pendientes:
            return None
        return min(range(len(pendientes)), key=lambda i: pendientes[i].t_llegada)


class SegundaPolitica(Politica):
    nombre = 'prioridad'
    def __init__(self, tau_envejecimiento_s=8.0):
        self.tau = max(0.001, float(tau_envejecimiento_s))
    def prioridad_efectiva(self, pedido):
        return float(pedido.priority) + pedido.espera_s / self.tau
    def siguiente(self, pendientes):
        if not pendientes:
            return None
        return max(
            range(len(pendientes)),
            key=lambda i: (self.prioridad_efectiva(pendientes[i]), -pendientes[i].t_llegada),
        )


POLITICAS = {'fifo': FIFO, 'prioridad': SegundaPolitica}
