"""arm_broker — el único nodo que publica en /joint_states.

Versión conservada a partir del broker RB-2 del repositorio del usuario.
La ruta semántica de P2 aún marca el pipeline físico como pendiente, igual que
el estado actual del repositorio; no se presenta como agarre físico terminado.
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_broker_interfaces.action import MoveArm, OrdenBrazo
from arm_broker_interfaces.msg import QueueState

from arm_broker import fk
from arm_broker.politicas import POLITICAS, Pedido


class ArmBroker(Node):
    def __init__(self):
        super().__init__('arm_broker')
        self.declare_parameter('politica', 'fifo')
        self.declare_parameter('tau_envejecimiento_s', 8.0)
        self.declare_parameter('cola_max', 20)
        self.declare_parameter('paso_max_rad', 1.2)
        self.declare_parameter('duracion_movimiento_s', 3.0)
        self.declare_parameter('pasos_interpolacion', 10)

        nombre = self.get_parameter('politica').value
        if nombre not in POLITICAS:
            raise RuntimeError(f'politica desconocida: {nombre}. Hay {list(POLITICAS)}')
        clase = POLITICAS[nombre]
        self.politica = clase(self.get_parameter('tau_envejecimiento_s').value) if nombre == 'prioridad' else clase()
        self.cola_max = int(self.get_parameter('cola_max').value)
        self.paso_max = float(self.get_parameter('paso_max_rad').value)
        self.duracion = float(self.get_parameter('duracion_movimiento_s').value)
        self.pasos = max(1, int(self.get_parameter('pasos_interpolacion').value))

        self.grupo_entrada = ReentrantCallbackGroup()
        self.grupo_worker = MutuallyExclusiveCallbackGroup()
        self.lock = threading.Lock()
        self.pendientes = []
        self.por_goal_id = {}
        self.ejecutando = None
        self.q_actual = [0.0] * 6
        self.n_aceptados = 0
        self.n_rechazados = 0
        self.n_completados = 0
        self._parar = threading.Event()

        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_cola = self.create_publisher(QueueState, '/arm/queue_state', 10)

        self.servidor = ActionServer(
            self, MoveArm, 'move_arm',
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self.grupo_entrada,
        )
        self.servidor_orden = ActionServer(
            self, OrdenBrazo, 'orden_brazo',
            goal_callback=self.goal_orden_callback,
            handle_accepted_callback=self.handle_accepted_orden_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_orden_callback,
            callback_group=self.grupo_entrada,
        )
        self.create_timer(0.2, self.publicar_estado_cola, callback_group=self.grupo_worker)
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()
        self.get_logger().info(
            f'arm_broker listo · politica={self.politica.nombre} · '
            f'cola_max={self.cola_max} · unico publicador de /joint_states'
        )

    def goal_callback(self, goal_request):
        cliente = goal_request.client_id.strip() or 'sin_id'
        prioridad = int(goal_request.priority)
        q_destino = [float(v) for v in goal_request.joint_positions]

        def rechazar(motivo):
            with self.lock:
                self.n_rechazados += 1
            self.get_logger().warning(
                f'REJECT cliente={cliente} prioridad={prioridad} motivo="{motivo}"'
            )
            return GoalResponse.REJECT

        if len(q_destino) != 6:
            return rechazar(f'se esperaban 6 articulaciones, llegaron {len(q_destino)}')
        if not all(math.isfinite(v) for v in q_destino):
            return rechazar('el objetivo contiene NaN o infinito')
        limites_ok, motivo = fk.dentro_de_limites(q_destino)
        if not limites_ok:
            return rechazar(f'limites articulares: {motivo}')
        workspace_ok, motivo = fk.dentro_del_workspace(q_destino)
        if not workspace_ok:
            return rechazar(f'workspace: {motivo}')
        with self.lock:
            q_actual = list(self.q_actual)
            longitud_cola = len(self.pendientes)
        paso = fk.paso_articular(q_actual, q_destino)
        if paso > self.paso_max:
            return rechazar(f'paso articular {paso:.3f} rad > maximo {self.paso_max:.3f} rad')
        if longitud_cola >= self.cola_max:
            return rechazar(f'cola llena ({longitud_cola}/{self.cola_max})')
        with self.lock:
            self.n_aceptados += 1
        self.get_logger().info(
            f'ACCEPT cliente={cliente} prioridad={prioridad} paso={paso:.3f} rad '
            f'cola={longitud_cola}/{self.cola_max}'
        )
        return GoalResponse.ACCEPT

    def goal_orden_callback(self, goal_request):
        cliente = goal_request.client_id.strip() or 'sin_id'
        prioridad = int(goal_request.priority)
        accion = goal_request.accion.strip().lower()
        objeto = goal_request.objeto.strip().lower()
        color = goal_request.color.strip().lower()
        zona = goal_request.zona_destino.strip()

        def rechazar(motivo):
            with self.lock:
                self.n_rechazados += 1
            self.get_logger().warning(
                f'REJECT_ORDEN cliente={cliente} prioridad={prioridad} motivo="{motivo}"'
            )
            return GoalResponse.REJECT

        if accion not in ('recoger', 'depositar'):
            return rechazar(f'accion no valida: {accion}')
        if not objeto:
            return rechazar('objeto vacio')
        if not color:
            return rechazar('color vacio')
        if not zona:
            return rechazar('zona_destino vacia')
        with self.lock:
            longitud_cola = len(self.pendientes)
        if longitud_cola >= self.cola_max:
            return rechazar(f'cola llena ({longitud_cola}/{self.cola_max})')
        with self.lock:
            self.n_aceptados += 1
        self.get_logger().info(
            f'ACCEPT_ORDEN cliente={cliente} prioridad={prioridad} '
            f'objeto={objeto} color={color} zona={zona}'
        )
        return GoalResponse.ACCEPT

    def handle_accepted_orden_callback(self, goal_handle):
        request = goal_handle.request
        pedido = Pedido(
            goal_handle=goal_handle,
            client_id=request.client_id.strip() or 'sin_id',
            priority=int(request.priority),
            tipo='orden', accion=request.accion, objeto=request.objeto,
            color=request.color, zona_destino=request.zona_destino,
        )
        with self.lock:
            self.pendientes.append(pedido)
            self.por_goal_id[pedido.goal_id] = pedido
            posicion = len(self.pendientes)
        self.get_logger().info(
            f'QUEUED_ORDEN goal={pedido.goal_id} cliente={pedido.client_id} '
            f'prioridad={pedido.priority} objeto={pedido.color}/{pedido.objeto} '
            f'zona={pedido.zona_destino} posicion={posicion}'
        )

    def execute_orden_callback(self, goal_handle):
        """Estado actual del repo: la cola semantica funciona, pipeline fisico pendiente."""
        goal_id = bytes(goal_handle.goal_id.uuid).hex()[:12]
        with self.lock:
            pedido = self.por_goal_id.get(goal_id)
        resultado = OrdenBrazo.Result()
        if pedido is None:
            resultado.success = False
            resultado.message = 'pedido semantico no encontrado'
            resultado.wait_time_s = 0.0
            resultado.exec_time_s = 0.0
            resultado.requested_x_mm = resultado.requested_y_mm = resultado.requested_z_mm = -1.0
            resultado.reached_x_mm = resultado.reached_y_mm = resultado.reached_z_mm = -1.0
            resultado.error_mm = -1.0
            goal_handle.abort()
            return resultado

        t_inicio = time.time()
        espera_s = max(0.0, pedido.t_inicio_ejec - pedido.t_llegada)
        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                resultado.success = False
                resultado.message = 'orden cancelada antes de ejecutar'
            else:
                fb = OrdenBrazo.Feedback()
                fb.state = 'EXECUTING'
                fb.stage = 'PIPELINE_PENDIENTE'
                fb.queue_position = 0
                fb.elapsed_s = 0.0
                goal_handle.publish_feedback(fb)
                self.get_logger().info(
                    f'EXECUTE_ORDEN goal={pedido.goal_id} cliente={pedido.client_id} '
                    f'objeto={pedido.color}/{pedido.objeto} zona={pedido.zona_destino}'
                )
                goal_handle.abort()
                resultado.success = False
                resultado.message = 'cola OK; pipeline fisico pendiente'

            resultado.wait_time_s = espera_s
            resultado.exec_time_s = max(0.0, time.time() - t_inicio)
            resultado.requested_x_mm = resultado.requested_y_mm = resultado.requested_z_mm = -1.0
            resultado.reached_x_mm = resultado.reached_y_mm = resultado.reached_z_mm = -1.0
            resultado.error_mm = -1.0
            pedido.resultado = resultado
            return resultado
        finally:
            pedido.fin.set()
            with self.lock:
                self.por_goal_id.pop(pedido.goal_id, None)

    def handle_accepted_callback(self, goal_handle):
        request = goal_handle.request
        pedido = Pedido(
            goal_handle=goal_handle,
            client_id=request.client_id.strip() or 'sin_id',
            priority=int(request.priority),
            joint_positions=[float(v) for v in request.joint_positions],
        )
        with self.lock:
            self.pendientes.append(pedido)
            self.por_goal_id[pedido.goal_id] = pedido
            posicion = len(self.pendientes)
        self.get_logger().info(
            f'QUEUED goal={pedido.goal_id} cliente={pedido.client_id} '
            f'prioridad={pedido.priority} posicion={posicion}'
        )

    def _worker(self):
        while not self._parar.is_set():
            pedido = None
            with self.lock:
                if self.ejecutando is None and self.pendientes:
                    indice = self.politica.siguiente(self.pendientes)
                    if indice is not None:
                        pedido = self.pendientes.pop(indice)
                        pedido.t_inicio_ejec = time.time()
                        self.ejecutando = pedido
            if pedido is None:
                time.sleep(0.02)
                continue
            self.get_logger().info(
                f'DISPATCH goal={pedido.goal_id} cliente={pedido.client_id} '
                f'prioridad={pedido.priority} espera={pedido.espera_s:.3f}s'
            )
            try:
                pedido.goal_handle.execute()
                while not pedido.fin.wait(timeout=0.1):
                    if self._parar.is_set():
                        break
            except Exception as error:
                self.get_logger().error(
                    f'Error ejecutando goal={pedido.goal_id}: {type(error).__name__}: {error}'
                )
                pedido.fin.set()
            finally:
                with self.lock:
                    if self.ejecutando is pedido:
                        self.ejecutando = None
                try:
                    self.politica.atendido(pedido)
                except Exception as error:
                    self.get_logger().warning(f'Error notificando a politica: {error}')

    def execute_callback(self, goal_handle):
        goal_id = bytes(goal_handle.goal_id.uuid).hex()[:12]
        with self.lock:
            pedido = self.por_goal_id.get(goal_id)
        resultado = MoveArm.Result()
        if pedido is None:
            resultado.success = False
            resultado.message = 'pedido interno no encontrado'
            resultado.wait_time_s = 0.0
            resultado.exec_time_s = 0.0
            goal_handle.abort()
            return resultado

        t_inicio_exec = time.time()
        if pedido.t_inicio_ejec is None:
            pedido.t_inicio_ejec = t_inicio_exec
        espera_s = max(0.0, pedido.t_inicio_ejec - pedido.t_llegada)
        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                resultado.success = False
                resultado.message = 'cancelado antes de ejecutar'
                resultado.wait_time_s = espera_s
                resultado.exec_time_s = 0.0
                pedido.resultado = resultado
                return resultado

            with self.lock:
                q_inicio = list(self.q_actual)
            q_destino = list(pedido.joint_positions)
            dt = self.duracion / float(self.pasos)
            self.get_logger().info(
                f'EXECUTE goal={pedido.goal_id} cliente={pedido.client_id} '
                f'pasos={self.pasos} duracion={self.duracion:.2f}s'
            )
            for i in range(1, self.pasos + 1):
                if goal_handle.is_cancel_requested:
                    exec_s = max(0.0, time.time() - t_inicio_exec)
                    goal_handle.canceled()
                    resultado.success = False
                    resultado.message = f'cancelado en paso {i}/{self.pasos}'
                    resultado.wait_time_s = espera_s
                    resultado.exec_time_s = exec_s
                    pedido.resultado = resultado
                    return resultado
                alpha = i / float(self.pasos)
                q_intermedia = [q0 + alpha * (q1 - q0) for q0, q1 in zip(q_inicio, q_destino)]
                self.mover(q_intermedia)
                fb = MoveArm.Feedback()
                fb.state = 'EXECUTING'
                fb.queue_position = 0
                fb.elapsed_s = max(0.0, time.time() - t_inicio_exec)
                goal_handle.publish_feedback(fb)
                time.sleep(dt)

            exec_s = max(0.0, time.time() - t_inicio_exec)
            goal_handle.succeed()
            resultado.success = True
            resultado.message = 'movimiento completado'
            resultado.wait_time_s = espera_s
            resultado.exec_time_s = exec_s
            pedido.resultado = resultado
            with self.lock:
                self.n_completados += 1
            self.get_logger().info(
                f'DONE goal={pedido.goal_id} cliente={pedido.client_id} '
                f'espera={espera_s:.3f}s ejec={exec_s:.3f}s'
            )
            return resultado
        except Exception as error:
            exec_s = max(0.0, time.time() - t_inicio_exec)
            self.get_logger().error(
                f'ABORT goal={pedido.goal_id} cliente={pedido.client_id} '
                f'error={type(error).__name__}: {error}'
            )
            try:
                goal_handle.abort()
            except Exception:
                pass
            resultado.success = False
            resultado.message = f'error interno: {type(error).__name__}'
            resultado.wait_time_s = espera_s
            resultado.exec_time_s = exec_s
            pedido.resultado = resultado
            return resultado
        finally:
            pedido.fin.set()
            with self.lock:
                self.por_goal_id.pop(pedido.goal_id, None)

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def mover(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = fk.JOINT_NAMES
        msg.position = [float(v) for v in q]
        self.pub_joint.publish(msg)
        with self.lock:
            self.q_actual = list(q)

    def publicar_estado_cola(self):
        msg = QueueState()
        msg.stamp = self.get_clock().now().to_msg()
        with self.lock:
            ej = self.ejecutando
            msg.executing_client = ej.client_id if ej else ''
            msg.executing_goal_id = ej.goal_id if ej else ''
            msg.executing_elapsed_s = (time.time() - ej.t_inicio_ejec) if ej and ej.t_inicio_ejec else 0.0
            msg.queue_length = len(self.pendientes)
            msg.queued_goal_ids = [p.goal_id for p in self.pendientes]
            msg.queued_clients = [p.client_id for p in self.pendientes]
            msg.queued_priorities = [min(255, max(0, p.priority)) for p in self.pendientes]
            msg.queued_wait_s = [p.espera_s for p in self.pendientes]
            msg.total_accepted = self.n_aceptados
            msg.total_rejected = self.n_rechazados
            msg.total_completed = self.n_completados
            cola = list(self.pendientes)
        self.pub_cola.publish(msg)
        for posicion, p in enumerate(cola, start=1):
            try:
                if p.tipo == 'orden':
                    fb = OrdenBrazo.Feedback()
                    fb.state = 'QUEUED'
                    fb.stage = 'COLA'
                    fb.queue_position = posicion
                    fb.elapsed_s = p.espera_s
                else:
                    fb = MoveArm.Feedback()
                    fb.state = 'QUEUED'
                    fb.queue_position = posicion
                    fb.elapsed_s = p.espera_s
                p.goal_handle.publish_feedback(fb)
            except Exception:
                pass

    def destroy_node(self):
        self._parar.set()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = ArmBroker()
    executor = MultiThreadedExecutor()
    executor.add_node(nodo)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
