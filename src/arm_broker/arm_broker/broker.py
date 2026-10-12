#!/usr/bin/env python3

"""
arm_broker - Broker único del JetCobot para la Pregunta 2.

Responsabilidades:
- /move_arm
- /orden_brazo
- cola FIFO / prioridad
- único worker de ejecución
- único controlador del brazo físico
- único controlador del gripper
- publicación de /joint_states
- integración con /localizar_objeto

Unidades:
- ROS / MoveArm: radianes
- pymycobot: grados
"""

import math
import threading
import time

import rclpy

from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)

from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState

from arm_broker_interfaces.action import (
    MoveArm,
    OrdenBrazo,
)

from arm_broker_interfaces.msg import QueueState
from arm_broker_interfaces.srv import LocalizarObjeto

from arm_broker import fk
from arm_broker.politicas import POLITICAS, Pedido


try:
    from pymycobot.mycobot import MyCobot
except ImportError:
    MyCobot = None


class ArmBroker(Node):

    def __init__(self):
        super().__init__('arm_broker')

        # ============================================================
        # PARAMETROS DEL BROKER
        # ============================================================

        self.declare_parameter(
            'politica',
            'prioridad'
        )

        self.declare_parameter(
            'tau_envejecimiento_s',
            8.0
        )

        self.declare_parameter(
            'cola_max',
            20
        )

        self.declare_parameter(
            'paso_max_rad',
            1.2
        )

        self.declare_parameter(
            'duracion_movimiento_s',
            3.0
        )

        self.declare_parameter(
            'pasos_interpolacion',
            10
        )

        # ============================================================
        # HARDWARE REAL
        # ============================================================

        self.declare_parameter(
            'hardware_real',
            True
        )

        self.declare_parameter(
            'serial_port',
            '/dev/ttyUSB0'
        )

        self.declare_parameter(
            'serial_baud',
            1000000
        )

        self.declare_parameter(
            'velocidad_robot',
            30
        )

        self.declare_parameter(
            'velocidad_gripper',
            30
        )

        self.declare_parameter(
            'gripper_abierto',
            100
        )

        self.declare_parameter(
            'gripper_cerrado',
            15
        )

        # ============================================================
        # LEER PARAMETROS
        # ============================================================

        nombre_politica = str(
            self.get_parameter(
                'politica'
            ).value
        )

        if nombre_politica not in POLITICAS:
            raise RuntimeError(
                f'Politica desconocida: '
                f'{nombre_politica}'
            )

        clase_politica = POLITICAS[
            nombre_politica
        ]

        if nombre_politica == 'prioridad':
            self.politica = clase_politica(
                self.get_parameter(
                    'tau_envejecimiento_s'
                ).value
            )
        else:
            self.politica = clase_politica()

        self.cola_max = int(
            self.get_parameter(
                'cola_max'
            ).value
        )

        self.paso_max = float(
            self.get_parameter(
                'paso_max_rad'
            ).value
        )

        self.duracion = float(
            self.get_parameter(
                'duracion_movimiento_s'
            ).value
        )

        self.pasos = max(
            1,
            int(
                self.get_parameter(
                    'pasos_interpolacion'
                ).value
            )
        )

        self.hardware_real = bool(
            self.get_parameter(
                'hardware_real'
            ).value
        )

        self.serial_port = str(
            self.get_parameter(
                'serial_port'
            ).value
        )

        self.serial_baud = int(
            self.get_parameter(
                'serial_baud'
            ).value
        )

        self.velocidad_robot = int(
            self.get_parameter(
                'velocidad_robot'
            ).value
        )

        self.velocidad_gripper = int(
            self.get_parameter(
                'velocidad_gripper'
            ).value
        )

        self.gripper_abierto = int(
            self.get_parameter(
                'gripper_abierto'
            ).value
        )

        self.gripper_cerrado = int(
            self.get_parameter(
                'gripper_cerrado'
            ).value
        )

        # ============================================================
        # POSES MEDIDAS EN EL JETCOBOT
        # TODAS ESTAN EN GRADOS
        # ============================================================

        # Pose desde donde la camara ve los cuatro cubos.
        self.pose_home = [
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	-45.0,
	]
        self.pose_inspeccion = [
            0.0,
            0.0,
            4.0,
            -70.0,
            17.0,
            -42.0,
        ]

        # Pose de retiro segura.
        self.pose_segura = [
            0.0,
            -27.0,
            -47.0,
            10.0,
            0.0,
            -45.0,
        ]

        # Zona donde se deja el objeto.
        self.pose_zona_a = [
            74.0,
            -46.0,
            -55.0,
            22.0,
            0.0,
            -45.0,
        ]

        # ============================================================
        # POSES DE LAS CUATRO POSICIONES FISICAS
        #
        # IMPORTANTE:
        # estas poses NO pertenecen a un color.
        # pertenecen a una esquina física.
        #
        # La camara descubre en que esquina esta el color pedido.
        # ============================================================

        self.poses_esquinas = {

            # Actualmente aqui estaba el verde.
            'arriba_izquierda': [
                32.0,
                -60.0,
                -28.0,
                0.0,
                2.0,
                -26.0,
            ],

            # Actualmente aqui estaba el rojo.
            'arriba_derecha': [
                0.0,
                -57.0,
                -35.0,
                7.0,
                0.0,
                -45.0,
            ],

            # Actualmente aqui estaba el amarillo.
            'abajo_izquierda': [
                38.0,
                -28.0,
                -83.0,
                8.0,
                18.0,
                -13.0,
            ],

            # Actualmente aqui estaba el azul.
            'abajo_derecha': [
                -1.0,
                -29.0,
                -86.0,
                10.0,
                5.0,
                -53.0,
            ],
        }

        # Imagen 640 x 480.
        # Dividimos en cuatro cuadrantes.
        self.corte_pixel_x = 320
        self.corte_pixel_y = 240

        # ============================================================
        # CALLBACK GROUPS
        # ============================================================

        self.grupo_entrada = (
            ReentrantCallbackGroup()
        )

        self.grupo_worker = (
            MutuallyExclusiveCallbackGroup()
        )

        # ============================================================
        # ESTADO INTERNO
        # ============================================================

        self.lock = threading.Lock()

        # Solo un hilo toca el puerto serie.
        self.hw_lock = threading.Lock()

        self.pendientes = []
        self.por_goal_id = {}

        self.ejecutando = None

        self.n_aceptados = 0
        self.n_rechazados = 0
        self.n_completados = 0

        self._parar = threading.Event()

        # ============================================================
        # CONEXION AL JETCOBOT
        # ============================================================

        self.mc = None

        if self.hardware_real:

            if MyCobot is None:
                raise RuntimeError(
                    'pymycobot no esta instalado'
                )

            self.get_logger().info(
                f'Conectando JetCobot '
                f'{self.serial_port} '
                f'@ {self.serial_baud}'
            )

            self.mc = MyCobot(
                self.serial_port,
                self.serial_baud
            )

            time.sleep(1.0)

            angulos_iniciales = (
                self._leer_angulos_hardware(
                    reintentos=5
                )
            )

            if angulos_iniciales is None:
                raise RuntimeError(
                    'No se pudieron leer '
                    'los 6 angulos del JetCobot'
                )

            self.q_actual = [
                math.radians(v)
                for v in angulos_iniciales
            ]

            self.get_logger().info(
                'POSE_FISICA_INICIAL grados=['
                + ', '.join(
                    f'{v:.2f}'
                    for v
                    in angulos_iniciales
                )
                + ']'
            )

        else:

            self.q_actual = [
                0.0
            ] * 6

            self.get_logger().warning(
                'hardware_real=False'
            )

        # ============================================================
        # PUBLISHERS
        # ============================================================

        self.pub_joint = (
            self.create_publisher(
                JointState,
                '/joint_states',
                10
            )
        )

        self.pub_cola = (
            self.create_publisher(
                QueueState,
                '/arm/queue_state',
                10
            )
        )

        self.publicar_joint_state(
            self.q_actual
        )

        # ============================================================
        # CLIENTE DE PERCEPCION
        # ============================================================

        self.cli_localizar = (
            self.create_client(
                LocalizarObjeto,
                '/localizar_objeto',
                callback_group=(
                    self.grupo_entrada
                )
            )
        )

        # ============================================================
        # ACTION /move_arm
        # ============================================================

        self.servidor = ActionServer(
            self,
            MoveArm,
            'move_arm',

            goal_callback=(
                self.goal_callback
            ),

            handle_accepted_callback=(
                self.handle_accepted_callback
            ),

            cancel_callback=(
                self.cancel_callback
            ),

            execute_callback=(
                self.execute_callback
            ),

            callback_group=(
                self.grupo_entrada
            ),
        )

        # ============================================================
        # ACTION /orden_brazo
        # ============================================================

        self.servidor_orden = ActionServer(
            self,
            OrdenBrazo,
            'orden_brazo',

            goal_callback=(
                self.goal_orden_callback
            ),

            handle_accepted_callback=(
                self.handle_accepted_orden_callback
            ),

            cancel_callback=(
                self.cancel_callback
            ),

            execute_callback=(
                self.execute_orden_callback
            ),

            callback_group=(
                self.grupo_entrada
            ),
        )

        # ============================================================
        # TIMER DE COLA
        # ============================================================

        self.create_timer(
            0.2,
            self.publicar_estado_cola,
            callback_group=(
                self.grupo_worker
            )
        )

        # ============================================================
        # WORKER UNICO
        # ============================================================

        self.worker = threading.Thread(
            target=self._worker,
            daemon=True
        )

        self.worker.start()

        self.get_logger().info(
            f'arm_broker listo · '
            f'politica={self.politica.nombre} · '
            f'cola_max={self.cola_max} · '
            f'hardware_real={self.hardware_real} · '
            f'velocidad_robot='
            f'{self.velocidad_robot} · '
            f'velocidad_gripper='
            f'{self.velocidad_gripper}'
        )

    # ================================================================
    # HARDWARE
    # ================================================================

    def _leer_angulos_hardware(
        self,
        reintentos=3
    ):
        """Lee J1...J6 reales en grados."""

        if not self.hardware_real:
            return None

        for intento in range(
            max(1, reintentos)
        ):

            try:

                with self.hw_lock:

                    angulos = (
                        self.mc.get_angles()
                    )

                if (
                    angulos is not None
                    and len(angulos) == 6
                ):

                    return [
                        float(v)
                        for v in angulos
                    ]

            except Exception as error:

                self.get_logger().warning(
                    f'Lectura hardware '
                    f'intento={intento + 1}: '
                    f'{type(error).__name__}: '
                    f'{error}'
                )

            time.sleep(0.2)

        return None

    def publicar_joint_state(
        self,
        q_rad
    ):

        msg = JointState()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            'base_link'
        )

        msg.name = fk.JOINT_NAMES

        msg.position = [
            float(v)
            for v in q_rad
        ]

        self.pub_joint.publish(
            msg
        )

    def mover(
        self,
        q_rad
    ):
        """Movimiento físico.

        Entrada:
            q_rad: seis ángulos en radianes.

        pymycobot:
            recibe grados.
        """

        q_rad = [
            float(v)
            for v in q_rad
        ]

        q_deg = [
            math.degrees(v)
            for v in q_rad
        ]

        if self.hardware_real:

            self.get_logger().info(
                'JETCOBOT_CMD grados=['
                + ', '.join(
                    f'{v:.1f}'
                    for v in q_deg
                )
                + ']'
            )

            with self.hw_lock:

                self.mc.send_angles(
                    q_deg,
                    self.velocidad_robot
                )

        self.publicar_joint_state(
            q_rad
        )

        with self.lock:

            self.q_actual = list(
                q_rad
            )

    # ================================================================
    # GRIPPER
    # ================================================================

    def abrir_gripper(self):

        if not self.hardware_real:
            return

        self.get_logger().info(
            f'GRIPPER ABRIR '
            f'valor={self.gripper_abierto} '
            f'velocidad='
            f'{self.velocidad_gripper}'
        )

        with self.hw_lock:

            self.mc.set_gripper_value(
                self.gripper_abierto,
                self.velocidad_gripper
            )

        time.sleep(1.0)

    def cerrar_gripper(self):

        if not self.hardware_real:
            return

        self.get_logger().info(
            f'GRIPPER CERRAR '
            f'valor={self.gripper_cerrado} '
            f'velocidad='
            f'{self.velocidad_gripper}'
        )

        with self.hw_lock:

            self.mc.set_gripper_value(
                self.gripper_cerrado,
                self.velocidad_gripper
            )

        time.sleep(1.0)

    # ================================================================
    # P2 - FEEDBACK
    # ================================================================

    def publicar_feedback_orden(
        self,
        goal_handle,
        stage,
        inicio
    ):

        fb = OrdenBrazo.Feedback()

        fb.state = 'EXECUTING'
        fb.stage = str(stage)
        fb.queue_position = 0

        fb.elapsed_s = max(
            0.0,
            time.time() - inicio
        )

        goal_handle.publish_feedback(
            fb
        )

        self.get_logger().info(
            f'P2_STAGE {stage}'
        )

    # ================================================================
    # P2 - CLASIFICACION DE ESQUINA
    # ================================================================

    def clasificar_esquina(
        self,
        pixel_x,
        pixel_y
    ):

        x = int(pixel_x)
        y = int(pixel_y)

        if (
            x < self.corte_pixel_x
            and
            y < self.corte_pixel_y
        ):
            return 'arriba_izquierda'

        if (
            x >= self.corte_pixel_x
            and
            y < self.corte_pixel_y
        ):
            return 'arriba_derecha'

        if (
            x < self.corte_pixel_x
            and
            y >= self.corte_pixel_y
        ):
            return 'abajo_izquierda'

        return 'abajo_derecha'

    # ================================================================
    # P2 - PERCEPCION
    # ================================================================

    def localizar_color(
        self,
        color,
        objeto,
        timeout_s=6.0
    ):

        if not (
            self.cli_localizar
            .wait_for_service(
                timeout_sec=2.0
            )
        ):

            return (
                None,
                'servicio /localizar_objeto '
                'no disponible'
            )

        request = (
            LocalizarObjeto.Request()
        )

        request.color = str(color)
        request.objeto = str(objeto)

        future = (
            self.cli_localizar
            .call_async(request)
        )

        t0 = time.time()

        while (
            rclpy.ok()
            and not future.done()
        ):

            if (
                time.time() - t0
                > timeout_s
            ):

                return (
                    None,
                    'timeout esperando '
                    '/localizar_objeto'
                )

            time.sleep(0.05)

        try:

            response = future.result()

        except Exception as error:

            return (
                None,
                f'error percepcion: '
                f'{type(error).__name__}: '
                f'{error}'
            )

        if response is None:

            return (
                None,
                'respuesta vacia '
                'de percepcion'
            )

        if not response.success:

            return (
                None,
                response.message
            )

        return response, ''

    # ================================================================
    # P2 - MOVER A UNA POSE EN GRADOS
    # ================================================================

    def mover_pose_grados(
        self,
        nombre,
        grados,
        goal_handle=None,
        inicio=None
    ):

        if (
            grados is None
            or len(grados) != 6
        ):

            raise RuntimeError(
                f'pose {nombre} invalida'
            )

        q_destino = [
            math.radians(
                float(v)
            )
            for v in grados
        ]

        limites_ok, motivo = (
            fk.dentro_de_limites(
                q_destino
            )
        )

        if not limites_ok:

            raise RuntimeError(
                f'{nombre}: '
                f'limites articulares: '
                f'{motivo}'
            )

        workspace_ok, motivo = (
            fk.dentro_del_workspace(
                q_destino
            )
        )

        if not workspace_ok:

            raise RuntimeError(
                f'{nombre}: '
                f'workspace: {motivo}'
            )

        with self.lock:

            q_inicio = list(
                self.q_actual
            )

        if (
            goal_handle is not None
            and inicio is not None
        ):

            self.publicar_feedback_orden(
                goal_handle,
                f'MOVIENDO_{nombre.upper()}',
                inicio
            )

        self.get_logger().info(
            f'P2_MOVE '
            f'{nombre} '
            f'grados={grados}'
        )

        dt = (
            self.duracion
            /
            float(self.pasos)
        )

        for i in range(
            1,
            self.pasos + 1
        ):

            if (
                goal_handle is not None
                and
                goal_handle
                .is_cancel_requested
            ):

                raise RuntimeError(
                    'orden cancelada'
                )

            alpha = (
                i
                /
                float(self.pasos)
            )

            q_intermedia = [
                q0
                + alpha * (q1 - q0)

                for q0, q1
                in zip(
                    q_inicio,
                    q_destino
                )
            ]

            self.mover(
                q_intermedia
            )

            time.sleep(dt)

        # Leer posición real del brazo.
        if self.hardware_real:

            time.sleep(0.3)

            angulos_reales = (
                self._leer_angulos_hardware()
            )

            if (
                angulos_reales
                is not None
            ):

                q_real = [
                    math.radians(v)
                    for v
                    in angulos_reales
                ]

                with self.lock:

                    self.q_actual = list(
                        q_real
                    )

                self.publicar_joint_state(
                    q_real
                )

                self.get_logger().info(
                    f'P2_POSE_REAL '
                    f'{nombre} grados=['
                    + ', '.join(
                        f'{v:.2f}'
                        for v
                        in angulos_reales
                    )
                    + ']'
                )

    # ================================================================
    # ADMISION /move_arm
    # ================================================================

    def goal_callback(
        self,
        goal_request
    ):

        cliente = (
            goal_request
            .client_id
            .strip()
            or 'sin_id'
        )

        prioridad = int(
            goal_request.priority
        )

        q_destino = [
            float(v)
            for v
            in goal_request
            .joint_positions
        ]

        def rechazar(motivo):

            with self.lock:

                self.n_rechazados += 1

            self.get_logger().warning(
                f'REJECT '
                f'cliente={cliente} '
                f'prioridad={prioridad} '
                f'motivo="{motivo}"'
            )

            return GoalResponse.REJECT

        if len(q_destino) != 6:

            return rechazar(
                'se esperaban '
                '6 articulaciones'
            )

        if not all(
            math.isfinite(v)
            for v in q_destino
        ):

            return rechazar(
                'NaN o infinito'
            )

        limites_ok, motivo = (
            fk.dentro_de_limites(
                q_destino
            )
        )

        if not limites_ok:

            return rechazar(
                f'limites: {motivo}'
            )

        workspace_ok, motivo = (
            fk.dentro_del_workspace(
                q_destino
            )
        )

        if not workspace_ok:

            return rechazar(
                f'workspace: {motivo}'
            )

        with self.lock:

            q_actual = list(
                self.q_actual
            )

            longitud = len(
                self.pendientes
            )

        paso = fk.paso_articular(
            q_actual,
            q_destino
        )

        if paso > self.paso_max:

            return rechazar(
                f'paso articular '
                f'{paso:.3f} rad > '
                f'{self.paso_max:.3f}'
            )

        if longitud >= self.cola_max:

            return rechazar(
                'cola llena'
            )

        with self.lock:

            self.n_aceptados += 1

        self.get_logger().info(
            f'ACCEPT '
            f'cliente={cliente} '
            f'prioridad={prioridad} '
            f'paso={paso:.3f}'
        )

        return GoalResponse.ACCEPT

    # ================================================================
    # ADMISION /orden_brazo
    # ================================================================

    def goal_orden_callback(
        self,
        goal_request
    ):

        cliente = (
            goal_request
            .client_id
            .strip()
            or 'sin_id'
        )

        accion = (
            goal_request
            .accion
            .strip()
            .lower()
        )

        objeto = (
            goal_request
            .objeto
            .strip()
            .lower()
        )

        color = (
            goal_request
            .color
            .strip()
            .lower()
        )

        zona = (
            goal_request
            .zona_destino
            .strip()
            .lower()
        )

        prioridad = int(
            goal_request.priority
        )

        def rechazar(motivo):

            with self.lock:

                self.n_rechazados += 1

            self.get_logger().warning(
                f'REJECT_ORDEN '
                f'cliente={cliente} '
                f'motivo="{motivo}"'
            )

            return GoalResponse.REJECT

        if accion not in (
            'recoger',
            'depositar'
        ):

            return rechazar(
                f'accion no valida: '
                f'{accion}'
            )

        if not objeto:
            return rechazar(
                'objeto vacio'
            )

        if color not in (
            'rojo',
            'verde',
            'azul',
            'amarillo'
        ):

            return rechazar(
                f'color no valido: '
                f'{color}'
            )

        if not zona:
            return rechazar(
                'zona vacia'
            )

        with self.lock:

            longitud = len(
                self.pendientes
            )

        if longitud >= self.cola_max:

            return rechazar(
                'cola llena'
            )

        with self.lock:

            self.n_aceptados += 1

        self.get_logger().info(
            f'ACCEPT_ORDEN '
            f'cliente={cliente} '
            f'accion={accion} '
            f'objeto={objeto} '
            f'color={color} '
            f'zona={zona} '
            f'prioridad={prioridad}'
        )

        return GoalResponse.ACCEPT

    # ================================================================
    # ENCOLAR /move_arm
    # ================================================================

    def handle_accepted_callback(
        self,
        goal_handle
    ):

        request = goal_handle.request

        pedido = Pedido(
            goal_handle=goal_handle,

            client_id=(
                request
                .client_id
                .strip()
                or 'sin_id'
            ),

            priority=int(
                request.priority
            ),

            joint_positions=[
                float(v)
                for v
                in request
                .joint_positions
            ],
        )

        with self.lock:

            self.pendientes.append(
                pedido
            )

            self.por_goal_id[
                pedido.goal_id
            ] = pedido

            posicion = len(
                self.pendientes
            )

        self.get_logger().info(
            f'QUEUED '
            f'goal={pedido.goal_id} '
            f'cliente='
            f'{pedido.client_id} '
            f'posicion={posicion}'
        )

    # ================================================================
    # ENCOLAR /orden_brazo
    # ================================================================

    def handle_accepted_orden_callback(
        self,
        goal_handle
    ):

        request = goal_handle.request

        pedido = Pedido(
            goal_handle=goal_handle,

            client_id=(
                request
                .client_id
                .strip()
                or 'sin_id'
            ),

            priority=int(
                request.priority
            ),

            tipo='orden',

            accion=request.accion,
            objeto=request.objeto,
            color=request.color,

            zona_destino=(
                request.zona_destino
            ),
        )

        with self.lock:

            self.pendientes.append(
                pedido
            )

            self.por_goal_id[
                pedido.goal_id
            ] = pedido

            posicion = len(
                self.pendientes
            )

        self.get_logger().info(
            f'QUEUED_ORDEN '
            f'goal={pedido.goal_id} '
            f'cliente='
            f'{pedido.client_id} '
            f'color={pedido.color} '
            f'prioridad='
            f'{pedido.priority} '
            f'posicion={posicion}'
        )

    # ================================================================
    # WORKER UNICO
    # ================================================================

    def _worker(self):

        while not (
            self._parar.is_set()
        ):

            pedido = None

            with self.lock:

                if (
                    self.ejecutando
                    is None
                    and self.pendientes
                ):

                    indice = (
                        self.politica
                        .siguiente(
                            self.pendientes
                        )
                    )

                    if indice is not None:

                        pedido = (
                            self.pendientes
                            .pop(indice)
                        )

                        pedido.t_inicio_ejec = (
                            time.time()
                        )

                        self.ejecutando = (
                            pedido
                        )

            if pedido is None:

                time.sleep(0.02)
                continue

            self.get_logger().info(
                f'DISPATCH '
                f'goal={pedido.goal_id} '
                f'cliente='
                f'{pedido.client_id} '
                f'prioridad='
                f'{pedido.priority}'
            )

            try:

                pedido.goal_handle.execute()

                while not (
                    pedido.fin.wait(
                        timeout=0.1
                    )
                ):

                    if (
                        self._parar
                        .is_set()
                    ):
                        break

            except Exception as error:

                self.get_logger().error(
                    f'WORKER_ERROR '
                    f'{type(error).__name__}: '
                    f'{error}'
                )

                pedido.fin.set()

            finally:

                with self.lock:

                    if (
                        self.ejecutando
                        is pedido
                    ):

                        self.ejecutando = None

                try:

                    self.politica.atendido(
                        pedido
                    )

                except Exception:
                    pass

    # ================================================================
    # EJECUCION /move_arm
    # ================================================================

    def execute_callback(
        self,
        goal_handle
    ):

        goal_id = (
            bytes(
                goal_handle
                .goal_id
                .uuid
            )
            .hex()[:12]
        )

        with self.lock:

            pedido = (
                self.por_goal_id
                .get(goal_id)
            )

        resultado = MoveArm.Result()

        if pedido is None:

            resultado.success = False

            resultado.message = (
                'pedido no encontrado'
            )

            resultado.wait_time_s = 0.0
            resultado.exec_time_s = 0.0

            goal_handle.abort()

            return resultado

        inicio = time.time()

        espera_s = max(
            0.0,
            (
                pedido.t_inicio_ejec
                or inicio
            )
            - pedido.t_llegada
        )

        try:

            if (
                goal_handle
                .is_cancel_requested
            ):

                goal_handle.canceled()

                resultado.success = False
                resultado.message = (
                    'cancelado'
                )

                resultado.wait_time_s = (
                    espera_s
                )

                resultado.exec_time_s = 0.0

                return resultado

            with self.lock:

                q_inicio = list(
                    self.q_actual
                )

            q_destino = list(
                pedido.joint_positions
            )

            dt = (
                self.duracion
                /
                float(self.pasos)
            )

            for i in range(
                1,
                self.pasos + 1
            ):

                if (
                    goal_handle
                    .is_cancel_requested
                ):

                    goal_handle.canceled()

                    resultado.success = (
                        False
                    )

                    resultado.message = (
                        'cancelado durante '
                        'movimiento'
                    )

                    resultado.wait_time_s = (
                        espera_s
                    )

                    resultado.exec_time_s = (
                        time.time()
                        - inicio
                    )

                    return resultado

                alpha = (
                    i
                    /
                    float(self.pasos)
                )

                q_intermedia = [
                    q0
                    + alpha * (q1 - q0)

                    for q0, q1
                    in zip(
                        q_inicio,
                        q_destino
                    )
                ]

                self.mover(
                    q_intermedia
                )

                fb = MoveArm.Feedback()

                fb.state = 'EXECUTING'
                fb.queue_position = 0

                fb.elapsed_s = (
                    time.time()
                    - inicio
                )

                goal_handle.publish_feedback(
                    fb
                )

                time.sleep(dt)

            # Sincronizar pose física real.
            angulos = (
                self._leer_angulos_hardware()
                if self.hardware_real
                else None
            )

            if angulos is not None:

                q_real = [
                    math.radians(v)
                    for v in angulos
                ]

                with self.lock:

                    self.q_actual = q_real

                self.publicar_joint_state(
                    q_real
                )

            exec_s = (
                time.time()
                - inicio
            )

            goal_handle.succeed()

            resultado.success = True

            resultado.message = (
                'movimiento fisico '
                'completado'
            )

            resultado.wait_time_s = (
                espera_s
            )

            resultado.exec_time_s = (
                exec_s
            )

            with self.lock:

                self.n_completados += 1

            pedido.resultado = (
                resultado
            )

            return resultado

        except Exception as error:

            try:
                goal_handle.abort()
            except Exception:
                pass

            resultado.success = False

            resultado.message = (
                f'error: '
                f'{type(error).__name__}: '
                f'{error}'
            )

            resultado.wait_time_s = (
                espera_s
            )

            resultado.exec_time_s = (
                time.time()
                - inicio
            )

            return resultado

        finally:

            pedido.fin.set()

            with self.lock:

                self.por_goal_id.pop(
                    pedido.goal_id,
                    None
                )

    # ================================================================
    # EJECUCION P2 /orden_brazo
    # ================================================================

    def execute_orden_callback(
        self,
        goal_handle
    ):

        goal_id = (
            bytes(
                goal_handle
                .goal_id
                .uuid
            )
            .hex()[:12]
        )

        with self.lock:

            pedido = (
                self.por_goal_id
                .get(goal_id)
            )

        resultado = OrdenBrazo.Result()

        # Estos campos no se usan porque
        # nuestra estrategia trabaja con J1...J6.
        resultado.requested_x_mm = -1.0
        resultado.requested_y_mm = -1.0
        resultado.requested_z_mm = -1.0

        resultado.reached_x_mm = -1.0
        resultado.reached_y_mm = -1.0
        resultado.reached_z_mm = -1.0

        resultado.error_mm = -1.0

        if pedido is None:

            resultado.success = False

            resultado.message = (
                'pedido semantico '
                'no encontrado'
            )

            resultado.wait_time_s = 0.0
            resultado.exec_time_s = 0.0

            goal_handle.abort()

            return resultado

        inicio = time.time()

        espera_s = max(
            0.0,
            (
                pedido.t_inicio_ejec
                or inicio
            )
            - pedido.t_llegada
        )

        accion = (
            pedido.accion
            .strip()
            .lower()
        )

        objeto = (
            pedido.objeto
            .strip()
            .lower()
        )

        color = (
            pedido.color
            .strip()
            .lower()
        )

        objeto_sujeto = False

        try:

            # --------------------------------------------------------
            # 0. VALIDACIONES
            # --------------------------------------------------------

            if (
                goal_handle
                .is_cancel_requested
            ):

                goal_handle.canceled()

                resultado.success = False
                resultado.message = (
                    'orden cancelada'
                )

                return resultado

            if accion != 'recoger':

                raise RuntimeError(
                    'por ahora el pipeline '
                    'fisico soporta recoger'
                )

            # --------------------------------------------------------
            # 1. ABRIR GRIPPER
            # --------------------------------------------------------

            self.publicar_feedback_orden(
                goal_handle,
                'ABRIENDO_GRIPPER',
                inicio
            )

            self.abrir_gripper()

            # --------------------------------------------------------
            # 2. IR A INSPECCION
            # --------------------------------------------------------

            self.mover_pose_grados(
                'INSPECCION',
                self.pose_inspeccion,
                goal_handle,
                inicio
            )

            # --------------------------------------------------------
            # 3. CAMARA BUSCA EL COLOR
            # --------------------------------------------------------

            self.publicar_feedback_orden(
                goal_handle,
                f'BUSCANDO_{color.upper()}',
                inicio
            )

            percepcion, error = (
                self.localizar_color(
                    color,
                    objeto
                )
            )

            if percepcion is None:

                raise RuntimeError(
                    f'percepcion: {error}'
                )

            px = int(
                percepcion.pixel_x
            )

            py = int(
                percepcion.pixel_y
            )

            area = float(
                percepcion.area_px
            )

            self.get_logger().info(
                f'P2_DETECT '
                f'color={color} '
                f'pixel=({px},{py}) '
                f'area={area:.1f}'
            )

            # --------------------------------------------------------
            # 4. DETERMINAR EN QUE ESQUINA ESTA
            # --------------------------------------------------------

            esquina = (
                self.clasificar_esquina(
                    px,
                    py
                )
            )

            self.publicar_feedback_orden(
                goal_handle,
                f'ESQUINA_{esquina.upper()}',
                inicio
            )

            self.get_logger().info(
                f'P2_ESQUINA '
                f'color={color} '
                f'esquina={esquina}'
            )

            # --------------------------------------------------------
            # 5. OBTENER J1...J6 DE ESA ESQUINA
            # --------------------------------------------------------

            pose_objeto = (
                self.poses_esquinas.get(
                    esquina
                )
            )

            if pose_objeto is None:

                raise RuntimeError(
                    f'pose no disponible '
                    f'para {esquina}'
                )

            # --------------------------------------------------------
            # 6. IR AL CUBO
            # --------------------------------------------------------

            self.mover_pose_grados(
                esquina,
                pose_objeto,
                goal_handle,
                inicio
            )

            # --------------------------------------------------------
            # 7. CERRAR GRIPPER
            # --------------------------------------------------------

            self.publicar_feedback_orden(
                goal_handle,
                'CERRANDO_GRIPPER',
                inicio
            )

            self.cerrar_gripper()

            objeto_sujeto = True

            # --------------------------------------------------------
            # 8. RETIRARSE A POSE SEGURA
            # --------------------------------------------------------

            self.mover_pose_grados(
                'segura',
                self.pose_segura,
                goal_handle,
                inicio
            )

            # --------------------------------------------------------
            # 9. IR A ZONA A
            # --------------------------------------------------------

            self.mover_pose_grados(
                'zona_a',
                self.pose_zona_a,
                goal_handle,
                inicio
            )

            # --------------------------------------------------------
            # 10. ABRIR GRIPPER / DEPOSITAR
            # --------------------------------------------------------

            self.publicar_feedback_orden(
                goal_handle,
                'SOLTANDO_OBJETO',
                inicio
            )

            self.abrir_gripper()

            objeto_sujeto = False

            # --------------------------------------------------------
            # 11. RETIRARSE DE LA ZONA
            #
            # HOME todavía no ha sido medido.
            # Por ahora termina en SEGURA.
            # --------------------------------------------------------

            self.mover_pose_grados(
                'HOME',
                self.pose_home,
                goal_handle,
                inicio
            )

            # --------------------------------------------------------
            # RESULTADO
            # --------------------------------------------------------

            exec_s = (
                time.time()
                - inicio
            )

            goal_handle.succeed()

            resultado.success = True

            resultado.message = (
                f'{color} detectado en '
                f'{esquina}; '
                f'recogido y depositado '
                f'en zona_a'
            )

            resultado.wait_time_s = (
                espera_s
            )

            resultado.exec_time_s = (
                exec_s
            )

            pedido.resultado = (
                resultado
            )

            with self.lock:

                self.n_completados += 1

            self.get_logger().info(
                f'P2_DONE '
                f'color={color} '
                f'esquina={esquina} '
                f't={exec_s:.2f}s'
            )

            return resultado

        except Exception as error:

            mensaje = str(error)

            self.get_logger().error(
                f'P2_ABORT '
                f'color={color} '
                f'error={mensaje}'
            )

            if objeto_sujeto:

                self.get_logger().warning(
                    'P2_ABORT con objeto '
                    'sujeto: NO se abre '
                    'automaticamente el '
                    'gripper para evitar '
                    'dejar caer el cubo.'
                )

            try:

                goal_handle.abort()

            except Exception:
                pass

            resultado.success = False

            resultado.message = (
                f'P2 abortado: {mensaje}'
            )

            resultado.wait_time_s = (
                espera_s
            )

            resultado.exec_time_s = (
                time.time()
                - inicio
            )

            pedido.resultado = (
                resultado
            )

            return resultado

        finally:

            pedido.fin.set()

            with self.lock:

                self.por_goal_id.pop(
                    pedido.goal_id,
                    None
                )

    # ================================================================
    # CANCEL
    # ================================================================

    def cancel_callback(
        self,
        goal_handle
    ):

        return CancelResponse.ACCEPT

    # ================================================================
    # ESTADO DE COLA
    # ================================================================

    def publicar_estado_cola(
        self
    ):

        msg = QueueState()

        msg.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        with self.lock:

            ejecutando = (
                self.ejecutando
            )

            msg.executing_client = (
                ejecutando.client_id
                if ejecutando
                else ''
            )

            msg.executing_goal_id = (
                ejecutando.goal_id
                if ejecutando
                else ''
            )

            msg.executing_elapsed_s = (
                (
                    time.time()
                    - ejecutando
                    .t_inicio_ejec
                )
                if (
                    ejecutando
                    and
                    ejecutando
                    .t_inicio_ejec
                )
                else 0.0
            )

            msg.queue_length = len(
                self.pendientes
            )

            msg.queued_goal_ids = [
                p.goal_id
                for p
                in self.pendientes
            ]

            msg.queued_clients = [
                p.client_id
                for p
                in self.pendientes
            ]

            msg.queued_priorities = [
                min(
                    255,
                    max(
                        0,
                        p.priority
                    )
                )
                for p
                in self.pendientes
            ]

            msg.queued_wait_s = [
                p.espera_s
                for p
                in self.pendientes
            ]

            msg.total_accepted = (
                self.n_aceptados
            )

            msg.total_rejected = (
                self.n_rechazados
            )

            msg.total_completed = (
                self.n_completados
            )

            cola = list(
                self.pendientes
            )

        self.pub_cola.publish(
            msg
        )

        for posicion, pedido in enumerate(
            cola,
            start=1
        ):

            try:

                if pedido.tipo == 'orden':

                    fb = (
                        OrdenBrazo
                        .Feedback()
                    )

                    fb.state = 'QUEUED'
                    fb.stage = 'COLA'

                    fb.queue_position = (
                        posicion
                    )

                    fb.elapsed_s = (
                        pedido.espera_s
                    )

                else:

                    fb = (
                        MoveArm
                        .Feedback()
                    )

                    fb.state = 'QUEUED'

                    fb.queue_position = (
                        posicion
                    )

                    fb.elapsed_s = (
                        pedido.espera_s
                    )

                pedido.goal_handle.publish_feedback(
                    fb
                )

            except Exception:
                pass

    # ================================================================
    # DESTRUIR
    # ================================================================

    def destroy_node(self):

        self._parar.set()

        return super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    nodo = ArmBroker()

    executor = (
        MultiThreadedExecutor(
            num_threads=4
        )
    )

    executor.add_node(
        nodo
    )

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
