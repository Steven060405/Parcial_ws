#!/usr/bin/env python3

"""
arm_broker - Broker unico del JetCobot para la Pregunta 2.

Responsabilidades:
- /move_arm
- /orden_brazo
- cola FIFO / prioridad
- unico worker de ejecucion
- unico controlador del brazo fisico
- unico controlador del gripper
- publica /joint_states con el estado REAL del JetCobot
- publica /arm/queue_state
- integra /localizar_objeto

Unidades:
- ROS / MoveArm: radianes
- pymycobot: grados
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
from arm_broker_interfaces.srv import LocalizarObjeto, PlanificarPose

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
        # PARAMETROS
        # ============================================================

        self.declare_parameter('politica', 'prioridad')
        self.declare_parameter('tau_envejecimiento_s', 8.0)
        self.declare_parameter('cola_max', 20)
        self.declare_parameter('paso_max_rad', 1.2)
        self.declare_parameter('duracion_movimiento_s', 3.0)
        self.declare_parameter('pasos_interpolacion', 10)

        self.declare_parameter('hardware_real', True)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 1000000)
        self.declare_parameter('velocidad_robot', 30)
        self.declare_parameter('velocidad_gripper', 30)
        self.declare_parameter('gripper_abierto', 100)
        self.declare_parameter('gripper_cerrado', 15)

        nombre_politica = str(self.get_parameter('politica').value)

        if nombre_politica not in POLITICAS:
            raise RuntimeError(f'Politica desconocida: {nombre_politica}')

        clase_politica = POLITICAS[nombre_politica]

        if nombre_politica == 'prioridad':
            self.politica = clase_politica(
                self.get_parameter('tau_envejecimiento_s').value
            )
        else:
            self.politica = clase_politica()

        self.cola_max = int(self.get_parameter('cola_max').value)
        self.paso_max = float(self.get_parameter('paso_max_rad').value)
        self.duracion = float(self.get_parameter('duracion_movimiento_s').value)
        self.pasos = max(
            1,
            int(self.get_parameter('pasos_interpolacion').value)
        )

        self.hardware_real = bool(self.get_parameter('hardware_real').value)
        self.serial_port = str(self.get_parameter('serial_port').value)
        self.serial_baud = int(self.get_parameter('serial_baud').value)
        self.velocidad_robot = int(self.get_parameter('velocidad_robot').value)
        self.velocidad_gripper = int(self.get_parameter('velocidad_gripper').value)
        self.gripper_abierto = int(self.get_parameter('gripper_abierto').value)
        self.gripper_cerrado = int(self.get_parameter('gripper_cerrado').value)

        # ============================================================
        # POSES MEDIDAS - GRADOS
        # ============================================================

        self.pose_home = [
            0.0, 0.0, 0.0, 0.0, 0.0, -45.0
        ]

        self.pose_inspeccion = [
            0.0, 0.0, 4.0, -70.0, 17.0, -42.0
        ]

        self.pose_segura = [
            0.0, -27.0, -47.0, 10.0, 0.0, -45.0
        ]

        self.pose_zona_a = [
            74.0, -46.0, -55.0, 22.0, 0.0, -45.0
        ]

        self.poses_esquinas = {
            'arriba_izquierda': [
                32.0, -60.0, -28.0, 0.0, 2.0, -26.0
            ],
            'arriba_derecha': [
                0.0, -57.0, -35.0, 7.0, 0.0, -45.0
            ],
            'abajo_izquierda': [
                38.0, -28.0, -83.0, 8.0, 18.0, -13.0
            ],
            'abajo_derecha': [
                -1.0, -29.0, -86.0, 10.0, 5.0, -53.0
            ],
        }

        # ============================================================
        # OBJETIVOS CARTESIANOS RB-2 [mm]
        #
        # Se calculan con la cinematica directa del RB-2 a partir de
        # las poses fisicas calibradas. La camara identifica la esquina
        # y esa esquina selecciona el objetivo conocido.
        #
        # Estos mismos XYZ se usan despues para calcular el error:
        # error = || XYZ_solicitado - FK(q_real) ||
        # ============================================================

        self.objetivos_xyz_rb2 = {}

        for nombre, grados in self.poses_esquinas.items():
            q_rad = [
                math.radians(float(v))
                for v in grados
            ]
            self.objetivos_xyz_rb2[nombre] = fk.fk(q_rad)

        q_zona_a = [
            math.radians(float(v))
            for v in self.pose_zona_a
        ]

        self.objetivos_xyz_rb2['zona_a'] = fk.fk(q_zona_a)

        self.corte_pixel_x = 320
        self.corte_pixel_y = 240

        # ============================================================
        # CALLBACK GROUPS
        # ============================================================

        self.grupo_entrada = ReentrantCallbackGroup()
        self.grupo_worker = MutuallyExclusiveCallbackGroup()

        # ============================================================
        # ESTADO
        # ============================================================

        self.lock = threading.Lock()
        self.hw_lock = threading.Lock()

        self.pendientes = []
        self.por_goal_id = {}
        self.ejecutando = None

        self.n_aceptados = 0
        self.n_rechazados = 0
        self.n_completados = 0

        self._parar = threading.Event()

        # ============================================================
        # HARDWARE
        # ============================================================

        self.mc = None

        if self.hardware_real:

            if MyCobot is None:
                raise RuntimeError('pymycobot no esta instalado')

            self.get_logger().info(
                f'Conectando JetCobot {self.serial_port} @ {self.serial_baud}'
            )

            self.mc = MyCobot(
                self.serial_port,
                self.serial_baud
            )

            time.sleep(1.0)

            angulos_iniciales = self._leer_angulos_hardware(reintentos=5)

            if angulos_iniciales is None:
                raise RuntimeError(
                    'No se pudieron leer los 6 angulos del JetCobot'
                )

            self.q_actual = [
                math.radians(v)
                for v in angulos_iniciales
            ]

            self.get_logger().info(
                'POSE_FISICA_INICIAL grados=['
                + ', '.join(f'{v:.2f}' for v in angulos_iniciales)
                + ']'
            )

        else:
            self.q_actual = [0.0] * 6
            self.get_logger().warning('hardware_real=False')

        # ============================================================
        # PUBLISHERS
        # ============================================================

        self.pub_joint = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.pub_cola = self.create_publisher(
            QueueState,
            '/arm/queue_state',
            10
        )

        # Publicacion inicial.
        self.publicar_joint_state(self.q_actual)

        # NUEVO:
        # Publica continuamente el estado REAL del JetCobot a 10 Hz.
        # Esto permite que MoveIt2 conozca el estado articular actual.
        self.joint_state_timer = self.create_timer(
            0.1,
            self.publicar_joint_states_reales,
            callback_group=self.grupo_entrada
        )

        # ============================================================
        # CLIENTE DE PERCEPCION
        # ============================================================

        self.cli_localizar = self.create_client(
            LocalizarObjeto,
            '/localizar_objeto',
            callback_group=self.grupo_entrada
        )

        # ============================================================
        # CLIENTE DE MOVEIT2
        #
        # El planificador SOLO calcula JointTrajectory.
        # El unico que ejecuta fisicamente sigue siendo este broker.
        # ============================================================

        self.cli_planificar = self.create_client(
            PlanificarPose,
            '/planificar_pose',
            callback_group=self.grupo_entrada
        )

        # ============================================================
        # ACTION /move_arm
        # ============================================================

        self.servidor = ActionServer(
            self,
            MoveArm,
            'move_arm',
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self.grupo_entrada,
        )

        # ============================================================
        # ACTION /orden_brazo
        # ============================================================

        self.servidor_orden = ActionServer(
            self,
            OrdenBrazo,
            'orden_brazo',
            goal_callback=self.goal_orden_callback,
            handle_accepted_callback=self.handle_accepted_orden_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_orden_callback,
            callback_group=self.grupo_entrada,
        )

        # ============================================================
        # TIMER DE COLA 5 Hz
        # ============================================================

        self.create_timer(
            0.2,
            self.publicar_estado_cola,
            callback_group=self.grupo_worker
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
            f'velocidad_robot={self.velocidad_robot} · '
            f'velocidad_gripper={self.velocidad_gripper} · '
            f'joint_states=10Hz'
        )

    # ================================================================
    # HARDWARE / JOINT STATES
    # ================================================================

    def _leer_angulos_hardware(self, reintentos=3):
        """Lee J1...J6 reales en grados."""

        if not self.hardware_real or self.mc is None:
            return None

        for intento in range(max(1, reintentos)):

            try:
                with self.hw_lock:
                    angulos = self.mc.get_angles()

                if angulos is not None and len(angulos) == 6:
                    return [float(v) for v in angulos]

            except Exception as error:
                self.get_logger().warning(
                    f'Lectura hardware intento={intento + 1}: '
                    f'{type(error).__name__}: {error}'
                )

            time.sleep(0.2)

        return None

    def publicar_joint_state(self, q_rad):
        """Publica un JointState en radianes."""

        if q_rad is None or len(q_rad) != 6:
            return

        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        # MoveIt2 usa los nombres definidos por el URDF.
        msg.name = list(fk.JOINT_NAMES)

        msg.position = [
            float(v)
            for v in q_rad
        ]

        msg.velocity = []
        msg.effort = []

        self.pub_joint.publish(msg)

    def publicar_joint_states_reales(self):
        """
        Lee el JetCobot real y publica /joint_states.

        IMPORTANTE:
        - usa el mismo puerto serie que el broker;
        - hw_lock evita acceso concurrente al serial;
        - actualiza tambien self.q_actual;
        - MoveIt2 recibe un estado actual a 10 Hz.
        """

        if not self.hardware_real or self.mc is None:
            self.publicar_joint_state(self.q_actual)
            return

        try:
            with self.hw_lock:
                angulos_deg = self.mc.get_angles()

            if angulos_deg is None or len(angulos_deg) != 6:
                return

            q_real = [
                math.radians(float(v))
                for v in angulos_deg
            ]

            with self.lock:
                self.q_actual = list(q_real)

            self.publicar_joint_state(q_real)

        except Exception as error:
            # No llenamos la terminal con errores repetitivos.
            self.get_logger().debug(
                f'joint_states: {type(error).__name__}: {error}'
            )

    def mover(self, q_rad):
        """Mueve fisicamente el JetCobot. Entrada en radianes."""

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
                + ', '.join(f'{v:.1f}' for v in q_deg)
                + ']'
            )

            with self.hw_lock:
                self.mc.send_angles(
                    q_deg,
                    self.velocidad_robot
                )

        with self.lock:
            self.q_actual = list(q_rad)

        # Publicacion inmediata, ademas del timer de 10 Hz.
        self.publicar_joint_state(q_rad)

    # ================================================================
    # GRIPPER
    # ================================================================

    def abrir_gripper(self):

        if not self.hardware_real:
            return

        self.get_logger().info(
            f'GRIPPER ABRIR valor={self.gripper_abierto} '
            f'velocidad={self.velocidad_gripper}'
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
            f'GRIPPER CERRAR valor={self.gripper_cerrado} '
            f'velocidad={self.velocidad_gripper}'
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

        goal_handle.publish_feedback(fb)

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

        if x < self.corte_pixel_x and y < self.corte_pixel_y:
            return 'arriba_izquierda'

        if x >= self.corte_pixel_x and y < self.corte_pixel_y:
            return 'arriba_derecha'

        if x < self.corte_pixel_x and y >= self.corte_pixel_y:
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

        if not self.cli_localizar.wait_for_service(timeout_sec=2.0):
            return None, 'servicio /localizar_objeto no disponible'

        request = LocalizarObjeto.Request()
        request.color = str(color)
        request.objeto = str(objeto)

        future = self.cli_localizar.call_async(request)

        t0 = time.time()

        while rclpy.ok() and not future.done():

            if time.time() - t0 > timeout_s:
                return None, 'timeout esperando /localizar_objeto'

            time.sleep(0.05)

        try:
            response = future.result()

        except Exception as error:
            return (
                None,
                f'error percepcion: {type(error).__name__}: {error}'
            )

        if response is None:
            return None, 'respuesta vacia de percepcion'

        if not response.success:
            return None, response.message

        return response, ''

    # ================================================================
    # P2 - MOVEIT2: PLANIFICAR OBJETIVO CONOCIDO
    # ================================================================

    def planificar_objetivo_moveit(
        self,
        nombre_objetivo,
        timeout_s=12.0
    ):
        """
        Solicita a /planificar_pose una trayectoria MoveIt2.

        Quaternion = 0 activa en nuestro planificador el modo
        "IK calibrada": la posicion conocida selecciona una semilla
        articular segura y MoveIt2 resuelve IK + planifica.

        Este metodo NO mueve el brazo.
        """

        nombre_objetivo = str(nombre_objetivo)

        if nombre_objetivo not in self.objetivos_xyz_rb2:
            return (
                None,
                f'objetivo MoveIt desconocido: {nombre_objetivo}'
            )

        if not self.cli_planificar.wait_for_service(
            timeout_sec=3.0
        ):
            return (
                None,
                'servicio /planificar_pose no disponible'
            )

        x_mm, y_mm, z_mm = (
            self.objetivos_xyz_rb2[nombre_objetivo]
        )

        request = PlanificarPose.Request()

        request.x_mm = float(x_mm)
        request.y_mm = float(y_mm)
        request.z_mm = float(z_mm)

        # Modo IK calibrada en planificador_item2.
        request.qx = 0.0
        request.qy = 0.0
        request.qz = 0.0
        request.qw = 0.0

        self.get_logger().info(
            f'MOVEIT_PLAN_REQUEST '
            f'objetivo={nombre_objetivo} '
            f'XYZ_RB2=({x_mm:.2f},'
            f'{y_mm:.2f},'
            f'{z_mm:.2f})mm'
        )

        future = self.cli_planificar.call_async(
            request
        )

        t0 = time.time()

        while rclpy.ok() and not future.done():

            if time.time() - t0 > timeout_s:
                return (
                    None,
                    f'timeout MoveIt2 para {nombre_objetivo}'
                )

            time.sleep(0.02)

        try:
            response = future.result()

        except Exception as error:
            return (
                None,
                f'error llamando MoveIt2: '
                f'{type(error).__name__}: {error}'
            )

        if response is None:
            return (
                None,
                'MoveIt2 devolvio respuesta vacia'
            )

        if not response.success:
            return (
                None,
                response.message
            )

        if not response.trajectory.points:
            return (
                None,
                'MoveIt2 devolvio trayectoria vacia'
            )

        self.get_logger().info(
            f'MOVEIT_PLAN_OK '
            f'objetivo={nombre_objetivo} '
            f'puntos={len(response.trajectory.points)} '
            f'mensaje="{response.message}"'
        )

        return response, ''


    # ================================================================
    # P2 - MOVEIT2: EJECUTAR JOINTTRAJECTORY
    # ================================================================

    @staticmethod
    def _duration_s(duration_msg):
        return (
            float(duration_msg.sec)
            +
            float(duration_msg.nanosec) * 1e-9
        )


    def ejecutar_trayectoria_moveit(
        self,
        nombre,
        trajectory,
        goal_handle=None,
        inicio=None
    ):
        """
        Ejecuta fisicamente una JointTrajectory ya planificada.

        IMPORTANTE:
        MoveIt2 NO envia comandos al hardware.
        Cada punto se ejecuta aqui, dentro del broker.
        """

        if trajectory is None:
            raise RuntimeError(
                f'{nombre}: trayectoria MoveIt2 nula'
            )

        if not trajectory.points:
            raise RuntimeError(
                f'{nombre}: trayectoria MoveIt2 vacia'
            )

        joint_names = list(
            trajectory.joint_names
        )

        if not joint_names:
            raise RuntimeError(
                f'{nombre}: MoveIt2 no envio joint_names'
            )

        faltantes = [
            j for j in fk.JOINT_NAMES
            if j not in joint_names
        ]

        if faltantes:
            raise RuntimeError(
                f'{nombre}: faltan joints MoveIt2: '
                f'{faltantes}'
            )

        indices = [
            joint_names.index(j)
            for j in fk.JOINT_NAMES
        ]

        if (
            goal_handle is not None
            and inicio is not None
        ):
            self.publicar_feedback_orden(
                goal_handle,
                f'EJECUTANDO_MOVEIT_{nombre.upper()}',
                inicio
            )

        self.get_logger().info(
            f'MOVEIT_EXEC_BEGIN '
            f'objetivo={nombre} '
            f'puntos={len(trajectory.points)}'
        )

        tiempo_anterior = 0.0

        for numero, point in enumerate(
            trajectory.points,
            start=1
        ):

            if (
                goal_handle is not None
                and goal_handle.is_cancel_requested
            ):
                raise RuntimeError(
                    'orden cancelada durante trayectoria MoveIt2'
                )

            posiciones = list(
                point.positions
            )

            if len(posiciones) < len(joint_names):
                raise RuntimeError(
                    f'{nombre}: punto {numero} '
                    f'con posiciones incompletas'
                )

            q_rad = [
                float(posiciones[i])
                for i in indices
            ]

            limites_ok, motivo = (
                fk.dentro_de_limites(q_rad)
            )

            if not limites_ok:
                raise RuntimeError(
                    f'{nombre}: punto MoveIt2 fuera '
                    f'de limites RB-2: {motivo}'
                )

            # UNICO punto donde se manda el comando fisico.
            self.mover(q_rad)

            tiempo_actual = self._duration_s(
                point.time_from_start
            )

            espera = max(
                0.0,
                tiempo_actual - tiempo_anterior
            )

            tiempo_anterior = tiempo_actual

            # Evita ciclos cero, pero conserva el tiempo de MoveIt.
            if espera > 0.0:
                time.sleep(espera)

        # Dar tiempo al controlador del JetCobot para asentarse.
        time.sleep(0.5)

        # Leer estado REAL final.
        angulos_reales = (
            self._leer_angulos_hardware(
                reintentos=8
            )
            if self.hardware_real
            else None
        )

        if angulos_reales is not None:

            q_real = [
                math.radians(v)
                for v in angulos_reales
            ]

            with self.lock:
                self.q_actual = list(q_real)

            self.publicar_joint_state(q_real)

            self.get_logger().info(
                f'MOVEIT_EXEC_END '
                f'objetivo={nombre} '
                f'q_real_deg=['
                + ', '.join(
                    f'{v:.2f}'
                    for v in angulos_reales
                )
                + ']'
            )

            return q_real

        # En simulacion, o si no hay lectura real disponible.
        with self.lock:
            q_final = list(self.q_actual)

        self.get_logger().warning(
            f'MOVEIT_EXEC_END {nombre}: '
            f'sin lectura hardware final'
        )

        return q_final


    # ================================================================
    # P2 - MOVER POSE CALIBRADA
    # ================================================================

    def mover_pose_grados(
        self,
        nombre,
        grados,
        goal_handle=None,
        inicio=None
    ):

        if grados is None or len(grados) != 6:
            raise RuntimeError(
                f'pose {nombre} invalida'
            )

        q_destino = [
            math.radians(float(v))
            for v in grados
        ]

        limites_ok, motivo = fk.dentro_de_limites(q_destino)

        if not limites_ok:
            raise RuntimeError(
                f'{nombre}: limites articulares: {motivo}'
            )

        workspace_ok, motivo = fk.dentro_del_workspace(q_destino)

        if not workspace_ok:
            raise RuntimeError(
                f'{nombre}: workspace: {motivo}'
            )

        with self.lock:
            q_inicio = list(self.q_actual)

        if goal_handle is not None and inicio is not None:
            self.publicar_feedback_orden(
                goal_handle,
                f'MOVIENDO_{nombre.upper()}',
                inicio
            )

        self.get_logger().info(
            f'P2_MOVE {nombre} grados={grados}'
        )

        dt = self.duracion / float(self.pasos)

        for i in range(1, self.pasos + 1):

            if (
                goal_handle is not None
                and goal_handle.is_cancel_requested
            ):
                raise RuntimeError(
                    'orden cancelada'
                )

            alpha = i / float(self.pasos)

            q_intermedia = [
                q0 + alpha * (q1 - q0)
                for q0, q1 in zip(
                    q_inicio,
                    q_destino
                )
            ]

            self.mover(q_intermedia)
            time.sleep(dt)

        # Sincronizar con angulos reales al terminar.
        if self.hardware_real:

            time.sleep(0.3)

            angulos_reales = self._leer_angulos_hardware()

            if angulos_reales is not None:

                q_real = [
                    math.radians(v)
                    for v in angulos_reales
                ]

                with self.lock:
                    self.q_actual = list(q_real)

                self.publicar_joint_state(q_real)

                self.get_logger().info(
                    f'P2_POSE_REAL {nombre} grados=['
                    + ', '.join(
                        f'{v:.2f}'
                        for v in angulos_reales
                    )
                    + ']'
                )

    # ================================================================
    # ADMISION /move_arm
    # ================================================================

    def goal_callback(self, goal_request):

        cliente = goal_request.client_id.strip() or 'sin_id'
        prioridad = int(goal_request.priority)

        q_destino = [
            float(v)
            for v in goal_request.joint_positions
        ]

        def rechazar(motivo):

            with self.lock:
                self.n_rechazados += 1

            self.get_logger().warning(
                f'REJECT cliente={cliente} '
                f'prioridad={prioridad} '
                f'motivo="{motivo}"'
            )

            return GoalResponse.REJECT

        if len(q_destino) != 6:
            return rechazar(
                'se esperaban 6 articulaciones'
            )

        if not all(
            math.isfinite(v)
            for v in q_destino
        ):
            return rechazar(
                'NaN o infinito'
            )

        limites_ok, motivo = fk.dentro_de_limites(q_destino)

        if not limites_ok:
            return rechazar(
                f'limites: {motivo}'
            )

        workspace_ok, motivo = fk.dentro_del_workspace(q_destino)

        if not workspace_ok:
            return rechazar(
                f'workspace: {motivo}'
            )

        with self.lock:
            q_actual = list(self.q_actual)
            longitud = len(self.pendientes)

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
            f'ACCEPT cliente={cliente} '
            f'prioridad={prioridad} '
            f'paso={paso:.3f}'
        )

        return GoalResponse.ACCEPT

    # ================================================================
    # ADMISION /orden_brazo
    # ================================================================

    def goal_orden_callback(self, goal_request):

        cliente = goal_request.client_id.strip() or 'sin_id'
        accion = goal_request.accion.strip().lower()
        objeto = goal_request.objeto.strip().lower()
        color = goal_request.color.strip().lower()
        zona = goal_request.zona_destino.strip().lower()
        prioridad = int(goal_request.priority)

        def rechazar(motivo):

            with self.lock:
                self.n_rechazados += 1

            self.get_logger().warning(
                f'REJECT_ORDEN cliente={cliente} '
                f'motivo="{motivo}"'
            )

            return GoalResponse.REJECT

        if accion not in ('recoger', 'depositar'):
            return rechazar(
                f'accion no valida: {accion}'
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
                f'color no valido: {color}'
            )

        if not zona:
            return rechazar(
                'zona vacia'
            )

        with self.lock:
            longitud = len(self.pendientes)

        if longitud >= self.cola_max:
            return rechazar(
                'cola llena'
            )

        with self.lock:
            self.n_aceptados += 1

        self.get_logger().info(
            f'ACCEPT_ORDEN cliente={cliente} '
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
            client_id=request.client_id.strip() or 'sin_id',
            priority=int(request.priority),
            joint_positions=[
                float(v)
                for v in request.joint_positions
            ],
        )

        with self.lock:
            self.pendientes.append(pedido)
            self.por_goal_id[pedido.goal_id] = pedido
            posicion = len(self.pendientes)

        self.get_logger().info(
            f'QUEUED goal={pedido.goal_id} '
            f'cliente={pedido.client_id} '
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
            client_id=request.client_id.strip() or 'sin_id',
            priority=int(request.priority),
            tipo='orden',
            accion=request.accion,
            objeto=request.objeto,
            color=request.color,
            zona_destino=request.zona_destino,
        )

        with self.lock:
            self.pendientes.append(pedido)
            self.por_goal_id[pedido.goal_id] = pedido
            posicion = len(self.pendientes)

        self.get_logger().info(
            f'QUEUED_ORDEN '
            f'goal={pedido.goal_id} '
            f'cliente={pedido.client_id} '
            f'color={pedido.color} '
            f'prioridad={pedido.priority} '
            f'posicion={posicion}'
        )

    # ================================================================
    # WORKER UNICO
    # ================================================================

    def _worker(self):

        while not self._parar.is_set():

            pedido = None

            with self.lock:

                if self.ejecutando is None and self.pendientes:

                    indice = self.politica.siguiente(
                        self.pendientes
                    )

                    if indice is not None:

                        pedido = self.pendientes.pop(indice)

                        pedido.t_inicio_ejec = time.time()

                        self.ejecutando = pedido

            if pedido is None:
                time.sleep(0.02)
                continue

            self.get_logger().info(
                f'DISPATCH goal={pedido.goal_id} '
                f'cliente={pedido.client_id} '
                f'prioridad={pedido.priority}'
            )

            try:

                pedido.goal_handle.execute()

                while not pedido.fin.wait(timeout=0.1):

                    if self._parar.is_set():
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

                    if self.ejecutando is pedido:
                        self.ejecutando = None

                try:
                    self.politica.atendido(pedido)
                except Exception:
                    pass

    # ================================================================
    # EJECUCION /move_arm
    # ================================================================

    def execute_callback(
        self,
        goal_handle
    ):

        goal_id = bytes(
            goal_handle.goal_id.uuid
        ).hex()[:12]

        with self.lock:
            pedido = self.por_goal_id.get(goal_id)

        resultado = MoveArm.Result()

        if pedido is None:

            resultado.success = False
            resultado.message = 'pedido no encontrado'
            resultado.wait_time_s = 0.0
            resultado.exec_time_s = 0.0

            goal_handle.abort()

            return resultado

        inicio = time.time()

        espera_s = max(
            0.0,
            (pedido.t_inicio_ejec or inicio)
            - pedido.t_llegada
        )

        try:

            if goal_handle.is_cancel_requested:

                goal_handle.canceled()

                resultado.success = False
                resultado.message = 'cancelado'
                resultado.wait_time_s = espera_s
                resultado.exec_time_s = 0.0

                return resultado

            with self.lock:
                q_inicio = list(self.q_actual)

            q_destino = list(
                pedido.joint_positions
            )

            dt = self.duracion / float(self.pasos)

            for i in range(1, self.pasos + 1):

                if goal_handle.is_cancel_requested:

                    goal_handle.canceled()

                    resultado.success = False
                    resultado.message = (
                        'cancelado durante movimiento'
                    )

                    resultado.wait_time_s = espera_s
                    resultado.exec_time_s = time.time() - inicio

                    return resultado

                alpha = i / float(self.pasos)

                q_intermedia = [
                    q0 + alpha * (q1 - q0)
                    for q0, q1 in zip(
                        q_inicio,
                        q_destino
                    )
                ]

                self.mover(q_intermedia)

                fb = MoveArm.Feedback()

                fb.state = 'EXECUTING'
                fb.queue_position = 0
                fb.elapsed_s = (
                    time.time() - inicio
                )

                goal_handle.publish_feedback(fb)

                time.sleep(dt)

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
                    self.q_actual = list(q_real)

                self.publicar_joint_state(q_real)

            exec_s = time.time() - inicio

            goal_handle.succeed()

            resultado.success = True
            resultado.message = (
                'movimiento fisico completado'
            )

            resultado.wait_time_s = espera_s
            resultado.exec_time_s = exec_s

            with self.lock:
                self.n_completados += 1

            pedido.resultado = resultado

            return resultado

        except Exception as error:

            try:
                goal_handle.abort()
            except Exception:
                pass

            resultado.success = False
            resultado.message = (
                f'error: {type(error).__name__}: {error}'
            )

            resultado.wait_time_s = espera_s
            resultado.exec_time_s = (
                time.time() - inicio
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

        goal_id = bytes(
            goal_handle.goal_id.uuid
        ).hex()[:12]

        with self.lock:
            pedido = self.por_goal_id.get(goal_id)

        resultado = OrdenBrazo.Result()

        # Todavia no calculamos XYZ/error cartesiano en este paso.
        # Se integrara despues de validar MoveIt2.
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
                'pedido semantico no encontrado'
            )

            resultado.wait_time_s = 0.0
            resultado.exec_time_s = 0.0

            goal_handle.abort()

            return resultado

        inicio = time.time()

        espera_s = max(
            0.0,
            (pedido.t_inicio_ejec or inicio)
            - pedido.t_llegada
        )

        accion = pedido.accion.strip().lower()
        objeto = pedido.objeto.strip().lower()
        color = pedido.color.strip().lower()

        objeto_sujeto = False

        try:

            if goal_handle.is_cancel_requested:

                goal_handle.canceled()

                resultado.success = False
                resultado.message = (
                    'orden cancelada'
                )

                return resultado

            if accion != 'recoger':
                raise RuntimeError(
                    'por ahora el pipeline fisico soporta recoger'
                )

            # 1. ABRIR GRIPPER
            self.publicar_feedback_orden(
                goal_handle,
                'ABRIENDO_GRIPPER',
                inicio
            )

            self.abrir_gripper()

            # 2. INSPECCION
            self.mover_pose_grados(
                'INSPECCION',
                self.pose_inspeccion,
                goal_handle,
                inicio
            )

            # 3. PERCEPCION
            self.publicar_feedback_orden(
                goal_handle,
                f'BUSCANDO_{color.upper()}',
                inicio
            )

            percepcion, error = self.localizar_color(
                color,
                objeto
            )

            if percepcion is None:
                raise RuntimeError(
                    f'percepcion: {error}'
                )

            px = int(percepcion.pixel_x)
            py = int(percepcion.pixel_y)
            area = float(percepcion.area_px)

            self.get_logger().info(
                f'P2_DETECT '
                f'color={color} '
                f'pixel=({px},{py}) '
                f'area={area:.1f}'
            )

            # 4. ESQUINA
            esquina = self.clasificar_esquina(
                px,
                py
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

            # 5. POSE DEL CUBO
            pose_objeto = self.poses_esquinas.get(
                esquina
            )

            if pose_objeto is None:
                raise RuntimeError(
                    f'pose no disponible para {esquina}'
                )

            # 6. IK + PLANIFICACION MOVEIT2 HACIA EL CUBO
            self.publicar_feedback_orden(
                goal_handle,
                f'PLANIFICANDO_MOVEIT_{esquina.upper()}',
                inicio
            )

            plan_objeto, error_moveit = (
                self.planificar_objetivo_moveit(
                    esquina
                )
            )

            if plan_objeto is None:
                raise RuntimeError(
                    f'MoveIt2 objeto: {error_moveit}'
                )

            self.ejecutar_trayectoria_moveit(
                esquina,
                plan_objeto.trajectory,
                goal_handle,
                inicio
            )

            # 7. CERRAR GRIPPER
            self.publicar_feedback_orden(
                goal_handle,
                'CERRANDO_GRIPPER',
                inicio
            )

            self.cerrar_gripper()

            objeto_sujeto = True

            # 8. SEGURA
            self.mover_pose_grados(
                'segura',
                self.pose_segura,
                goal_handle,
                inicio
            )

            # 9. IK + PLANIFICACION MOVEIT2 HACIA ZONA A
            self.publicar_feedback_orden(
                goal_handle,
                'PLANIFICANDO_MOVEIT_ZONA_A',
                inicio
            )

            plan_zona, error_moveit = (
                self.planificar_objetivo_moveit(
                    'zona_a'
                )
            )

            if plan_zona is None:
                raise RuntimeError(
                    f'MoveIt2 zona_a: {error_moveit}'
                )

            self.ejecutar_trayectoria_moveit(
                'zona_a',
                plan_zona.trajectory,
                goal_handle,
                inicio
            )

            # 10. SOLTAR
            self.publicar_feedback_orden(
                goal_handle,
                'SOLTANDO_OBJETO',
                inicio
            )

            self.abrir_gripper()

            objeto_sujeto = False

            # --------------------------------------------------------
            # ERROR CARTESIANO RB-2
            #
            # Solicitado: FK de la pose calibrada de zona_a.
            # Alcanzado: FK de los angulos REALES leidos del robot
            # mientras el efector sigue en la zona de deposito.
            # --------------------------------------------------------

            angulos_medidos = (
                self._leer_angulos_hardware(
                    reintentos=10
                )
                if self.hardware_real
                else None
            )

            if angulos_medidos is None:

                if self.hardware_real:
                    raise RuntimeError(
                        'no se pudo medir q_real '
                        'para calcular error FK'
                    )

                with self.lock:
                    q_medida = list(self.q_actual)

            else:

                q_medida = [
                    math.radians(v)
                    for v in angulos_medidos
                ]

                with self.lock:
                    self.q_actual = list(q_medida)

                self.publicar_joint_state(
                    q_medida
                )

            solicitado = (
                self.objetivos_xyz_rb2[
                    'zona_a'
                ]
            )

            alcanzado = fk.fk(
                q_medida
            )

            error_mm = math.sqrt(
                (alcanzado[0] - solicitado[0]) ** 2
                +
                (alcanzado[1] - solicitado[1]) ** 2
                +
                (alcanzado[2] - solicitado[2]) ** 2
            )

            resultado.requested_x_mm = float(
                solicitado[0]
            )
            resultado.requested_y_mm = float(
                solicitado[1]
            )
            resultado.requested_z_mm = float(
                solicitado[2]
            )

            resultado.reached_x_mm = float(
                alcanzado[0]
            )
            resultado.reached_y_mm = float(
                alcanzado[1]
            )
            resultado.reached_z_mm = float(
                alcanzado[2]
            )

            resultado.error_mm = float(
                error_mm
            )

            self.get_logger().info(
                f'P2_FK_ERROR '
                f'requested=({solicitado[0]:.2f},'
                f'{solicitado[1]:.2f},'
                f'{solicitado[2]:.2f})mm '
                f'reached=({alcanzado[0]:.2f},'
                f'{alcanzado[1]:.2f},'
                f'{alcanzado[2]:.2f})mm '
                f'error={error_mm:.2f}mm'
            )

            
            # 12. HOME
            self.mover_pose_grados(
                'HOME',
                self.pose_home,
                goal_handle,
                inicio
            )

            exec_s = time.time() - inicio

            goal_handle.succeed()

            resultado.success = True

            resultado.message = (
                f'{color} detectado en '
                f'{esquina}; '
                f'IK+MoveIt2 OK; '
                f'depositado en zona_a; '
                f'error_FK={resultado.error_mm:.2f} mm'
            )

            resultado.wait_time_s = espera_s
            resultado.exec_time_s = exec_s

            pedido.resultado = resultado

            with self.lock:
                self.n_completados += 1

            self.get_logger().info(
                f'P2_DONE '
                f'color={color} '
                f'esquina={esquina} '
                f'error_FK={resultado.error_mm:.2f}mm '
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
                    'P2_ABORT con objeto sujeto: '
                    'NO se abre automaticamente el '
                    'gripper para evitar dejar caer el cubo.'
                )

            try:
                goal_handle.abort()
            except Exception:
                pass

            resultado.success = False
            resultado.message = (
                f'P2 abortado: {mensaje}'
            )

            resultado.wait_time_s = espera_s
            resultado.exec_time_s = (
                time.time() - inicio
            )

            pedido.resultado = resultado

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

    def publicar_estado_cola(self):

        msg = QueueState()

        msg.stamp = self.get_clock().now().to_msg()

        with self.lock:

            ejecutando = self.ejecutando

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
                    - ejecutando.t_inicio_ejec
                )
                if (
                    ejecutando
                    and ejecutando.t_inicio_ejec
                )
                else 0.0
            )

            msg.queue_length = len(
                self.pendientes
            )

            msg.queued_goal_ids = [
                p.goal_id
                for p in self.pendientes
            ]

            msg.queued_clients = [
                p.client_id
                for p in self.pendientes
            ]

            msg.queued_priorities = [
                min(
                    255,
                    max(0, p.priority)
                )
                for p in self.pendientes
            ]

            msg.queued_wait_s = [
                p.espera_s
                for p in self.pendientes
            ]

            msg.total_accepted = self.n_aceptados
            msg.total_rejected = self.n_rechazados
            msg.total_completed = self.n_completados

            cola = list(self.pendientes)

        self.pub_cola.publish(msg)

        for posicion, pedido in enumerate(
            cola,
            start=1
        ):

            try:

                if pedido.tipo == 'orden':

                    fb = OrdenBrazo.Feedback()

                    fb.state = 'QUEUED'
                    fb.stage = 'COLA'
                    fb.queue_position = posicion
                    fb.elapsed_s = pedido.espera_s

                else:

                    fb = MoveArm.Feedback()

                    fb.state = 'QUEUED'
                    fb.queue_position = posicion
                    fb.elapsed_s = pedido.espera_s

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

    executor = MultiThreadedExecutor(
        num_threads=4
    )

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
