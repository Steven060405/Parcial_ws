import csv
import sys
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from arm_broker_interfaces.action import MoveArm


class Cliente(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.declare_parameter('client_id', 'alumno')
        self.declare_parameter('priority', 1)
        self.declare_parameter('traza', '')
        self.declare_parameter('repeticiones', 1)
        self.declare_parameter('pausa_s', 0.5)
        self.client_id = self.get_parameter('client_id').value
        self.priority = int(self.get_parameter('priority').value)
        self.repeticiones = int(self.get_parameter('repeticiones').value)
        self.pausa = float(self.get_parameter('pausa_s').value)
        self.cli = ActionClient(self, MoveArm, 'move_arm')
        self.poses = self.cargar(self.get_parameter('traza').value)

    def cargar(self, ruta):
        if not ruta:
            return [[0.3,0,0,0,0,0],[-0.3,0,0,0,0,0]]
        with open(ruta, newline='') as f:
            filas = [r for r in csv.reader(f) if r and not r[0].lstrip().startswith('#')]
        return [[float(v) for v in fila[:6]] for fila in filas]

    def correr(self):
        self.get_logger().info(f'[{self.client_id}] esperando al broker...')
        if not self.cli.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('El broker no aparece.')
            return 1
        for _ in range(self.repeticiones):
            for i, q in enumerate(self.poses):
                goal = MoveArm.Goal()
                goal.joint_positions = q
                goal.client_id = self.client_id
                goal.priority = self.priority
                t0 = time.time()
                envio = self.cli.send_goal_async(goal, feedback_callback=self.feedback)
                rclpy.spin_until_future_complete(self, envio)
                handle = envio.result()
                if not handle.accepted:
                    self.get_logger().warning(f'[{self.client_id}] pose {i}: RECHAZADA')
                    continue
                res_fut = handle.get_result_async()
                rclpy.spin_until_future_complete(self, res_fut)
                r = res_fut.result().result
                self.get_logger().info(
                    f'[{self.client_id}] pose {i}: success={r.success} '
                    f'espera={r.wait_time_s:.2f}s ejec={r.exec_time_s:.2f}s '
                    f'total={time.time()-t0:.2f}s — {r.message}'
                )
                time.sleep(self.pausa)
        return 0

    def feedback(self, msg):
        f = msg.feedback
        self.get_logger().info(f'[{self.client_id}] {f.state} pos={f.queue_position} t={f.elapsed_s:.1f}s')


def main(args=None):
    rclpy.init(args=args)
    nodo = Cliente()
    codigo = 0
    try:
        codigo = nodo.correr()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(codigo)


if __name__ == '__main__':
    main()
