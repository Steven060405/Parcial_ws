#!/usr/bin/env python3
import csv
import math
import os
import sys
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from parcial_interfaces.srv import InterpretarOrden
from arm_broker_interfaces.action import OrdenBrazo


PRUEBAS = [
    ("recoge el cubo verde", "verde"),
    ("recoge el cubo rojo", "rojo"),
    ("recoge el cubo azul", "azul"),
    ("recoge el cubo amarillo", "amarillo"),
    ("por favor recoge el cubo verde", "verde"),
    ("por favor recoge el cubo rojo", "rojo"),
    ("por favor recoge el cubo azul", "azul"),
    ("por favor recoge el cubo amarillo", "amarillo"),
    ("recoge el cubo verde por favor", "verde"),
    ("recoge el cubo rojo por favor", "rojo"),
]


class BenchmarkItem2(Node):
    def __init__(self):
        super().__init__("benchmark_item2_10")
        self.interprete = self.create_client(
            InterpretarOrden,
            "/interpretar_orden"
        )
        self.broker = ActionClient(
            self,
            OrdenBrazo,
            "/orden_brazo"
        )

    def interpretar(self, frase):
        if not self.interprete.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/interpretar_orden no disponible")

        req = InterpretarOrden.Request()
        req.frase = frase

        fut = self.interprete.call_async(req)
        rclpy.spin_until_future_complete(
            self,
            fut,
            timeout_sec=20.0
        )

        if not fut.done() or fut.result() is None:
            raise RuntimeError(
                "timeout o respuesta vacia de /interpretar_orden"
            )

        return fut.result()

    def ejecutar_broker(self, dec, intento):
        if not self.broker.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/orden_brazo no disponible")

        goal = OrdenBrazo.Goal()
        goal.client_id = f"benchmark_{intento:02d}"
        goal.accion = dec.accion
        goal.objeto = dec.objeto
        goal.color = dec.color
        goal.zona_destino = "zona_a"

        # REQUISITO DEL PARCIAL:
        # prioridad obtenida del modelo/servicio, no hardcodeada.
        goal.priority = max(0, min(255, int(dec.prioridad)))

        def feedback_cb(msg):
            fb = msg.feedback
            print(
                f"  FB state={fb.state} "
                f"stage={fb.stage} "
                f"pos={fb.queue_position} "
                f"t={fb.elapsed_s:.2f}s"
            )

        send_f = self.broker.send_goal_async(
            goal,
            feedback_callback=feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_f)

        handle = send_f.result()

        if handle is None or not handle.accepted:
            raise RuntimeError("broker rechazo el goal")

        result_f = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_f)

        wrapped = result_f.result()
        if wrapped is None:
            raise RuntimeError("resultado vacio de /orden_brazo")

        return wrapped.result


def pedir_confirmacion_fisica():
    while True:
        r = input(
            "\n¿El objeto CORRECTO quedo en la zona A? [s/n]: "
        ).strip().lower()

        if r in ("s", "si", "sí", "y", "yes"):
            return True

        if r in ("n", "no"):
            return False

        print("Responde s o n.")


def fmt(v):
    try:
        return f"{float(v):.3f}"
    except Exception:
        return ""


def main():
    rclpy.init()
    node = BenchmarkItem2()

    root = os.getcwd()
    salida = os.path.join(root, "resultados_item2")
    os.makedirs(salida, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(
        salida,
        f"intentos_item2_{stamp}.csv"
    )
    resumen_path = os.path.join(
        salida,
        f"resumen_item2_{stamp}.txt"
    )

    columnas = [
        "intento",
        "frase",
        "color_esperado",
        "accion_modelo",
        "objeto_modelo",
        "color_modelo",
        "prioridad_modelo",
        "permitido",
        "metodo",
        "broker_success",
        "exito_fisico",
        "message",
        "wait_time_s",
        "exec_time_s",
        "requested_x_mm",
        "requested_y_mm",
        "requested_z_mm",
        "reached_x_mm",
        "reached_y_mm",
        "reached_z_mm",
        "error_mm",
    ]

    filas = []

    print("=" * 70)
    print("PREGUNTA 2 - 10 INTENTOS DE AGARRE")
    print("=" * 70)
    print()
    print("IMPORTANTE:")
    print("- Cada intento pasa por P1 -> prioridad -> broker -> camara -> MoveIt2.")
    print("- Despues de cada intento confirma visualmente el exito fisico.")
    print("- Antes del siguiente, vuelve a colocar el cubo en su posicion original.")
    print("- NO cierres broker, percepcion, MoveIt2 ni interprete.")
    print()

    try:
        for i, (frase, color_esperado) in enumerate(PRUEBAS, start=1):
            print("\n" + "=" * 70)
            print(f"INTENTO {i}/10")
            print(f"Orden: {frase}")
            print(f"Color esperado: {color_esperado}")
            print("=" * 70)

            input(
                f"Coloca el cubo {color_esperado} en su posicion original "
                "y presiona ENTER para iniciar..."
            )

            fila = {
                "intento": i,
                "frase": frase,
                "color_esperado": color_esperado,
                "accion_modelo": "",
                "objeto_modelo": "",
                "color_modelo": "",
                "prioridad_modelo": "",
                "permitido": False,
                "metodo": "",
                "broker_success": False,
                "exito_fisico": False,
                "message": "",
                "wait_time_s": "",
                "exec_time_s": "",
                "requested_x_mm": "",
                "requested_y_mm": "",
                "requested_z_mm": "",
                "reached_x_mm": "",
                "reached_y_mm": "",
                "reached_z_mm": "",
                "error_mm": "",
            }

            try:
                dec = node.interpretar(frase)

                fila["accion_modelo"] = dec.accion
                fila["objeto_modelo"] = dec.objeto
                fila["color_modelo"] = dec.color
                fila["prioridad_modelo"] = int(dec.prioridad)
                fila["permitido"] = bool(dec.permitido)
                fila["metodo"] = dec.metodo

                print(
                    "\nDECISION "
                    f"accion={dec.accion} "
                    f"objeto={dec.objeto} "
                    f"color={dec.color} "
                    f"prioridad={dec.prioridad} "
                    f"permitido={dec.permitido} "
                    f"metodo={dec.metodo}"
                )

                if not dec.permitido:
                    fila["message"] = (
                        "RECHAZADA antes de encolar: permitido=false"
                    )
                    print(fila["message"])

                elif dec.accion == "detener":
                    fila["message"] = (
                        "Orden detener; no es intento pick-and-place"
                    )
                    print(fila["message"])

                else:
                    result = node.ejecutar_broker(dec, i)

                    fila["broker_success"] = bool(result.success)
                    fila["message"] = result.message
                    fila["wait_time_s"] = fmt(result.wait_time_s)
                    fila["exec_time_s"] = fmt(result.exec_time_s)
                    fila["requested_x_mm"] = fmt(result.requested_x_mm)
                    fila["requested_y_mm"] = fmt(result.requested_y_mm)
                    fila["requested_z_mm"] = fmt(result.requested_z_mm)
                    fila["reached_x_mm"] = fmt(result.reached_x_mm)
                    fila["reached_y_mm"] = fmt(result.reached_y_mm)
                    fila["reached_z_mm"] = fmt(result.reached_z_mm)
                    fila["error_mm"] = fmt(result.error_mm)

                    print("\nRESULTADO BROKER")
                    print(f"  success       = {result.success}")
                    print(f"  message       = {result.message}")
                    print(
                        "  requested XYZ = "
                        f"({result.requested_x_mm:.2f}, "
                        f"{result.requested_y_mm:.2f}, "
                        f"{result.requested_z_mm:.2f}) mm"
                    )
                    print(
                        "  reached XYZ   = "
                        f"({result.reached_x_mm:.2f}, "
                        f"{result.reached_y_mm:.2f}, "
                        f"{result.reached_z_mm:.2f}) mm"
                    )
                    print(
                        f"  error FK      = {result.error_mm:.2f} mm"
                    )
                    print(
                        f"  wait          = {result.wait_time_s:.2f} s"
                    )
                    print(
                        f"  exec          = {result.exec_time_s:.2f} s"
                    )

                    # La rubrica define exito fisico como:
                    # objeto correcto en zona correcta.
                    fila["exito_fisico"] = pedir_confirmacion_fisica()

            except Exception as exc:
                fila["message"] = (
                    f"EXCEPCION: {type(exc).__name__}: {exc}"
                )
                print("\nERROR:", fila["message"])

            filas.append(fila)

            # Guardado incremental: si algo se interrumpe, no se pierden
            # los intentos ya terminados.
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=columnas
                )
                writer.writeheader()
                writer.writerows(filas)

            print(f"\nGuardado parcial: {csv_path}")

        exitos = sum(
            1 for f in filas
            if f["exito_fisico"]
        )
        tasa = 100.0 * exitos / len(filas)

        errores_exitosos = []
        for f in filas:
            if f["exito_fisico"] and f["error_mm"] != "":
                try:
                    errores_exitosos.append(
                        float(f["error_mm"])
                    )
                except ValueError:
                    pass

        error_promedio = (
            sum(errores_exitosos) / len(errores_exitosos)
            if errores_exitosos
            else float("nan")
        )

        with open(resumen_path, "w", encoding="utf-8") as f:
            f.write("PREGUNTA 2 - RESUMEN 10 INTENTOS\n")
            f.write("=" * 45 + "\n")
            f.write(f"Intentos totales: {len(filas)}\n")
            f.write(f"Exitos fisicos: {exitos}\n")
            f.write(f"Tasa de exito: {tasa:.1f}%\n")
            if math.isfinite(error_promedio):
                f.write(
                    "Error FK promedio en intentos exitosos: "
                    f"{error_promedio:.3f} mm\n"
                )
            else:
                f.write(
                    "Error FK promedio en intentos exitosos: N/D\n"
                )
            f.write(f"CSV: {csv_path}\n")

        print("\n" + "=" * 70)
        print("RESUMEN FINAL")
        print("=" * 70)
        print(f"Intentos:      {len(filas)}")
        print(f"Exitos fisicos:{exitos}")
        print(f"Tasa de exito: {tasa:.1f}%")
        if math.isfinite(error_promedio):
            print(
                f"Error FK promedio: {error_promedio:.3f} mm"
            )
        print(f"CSV:           {csv_path}")
        print(f"Resumen:       {resumen_path}")
        print("=" * 70)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
