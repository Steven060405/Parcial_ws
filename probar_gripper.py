#!/usr/bin/env python3

import sys
import time
from pymycobot.mycobot import MyCobot

PUERTO = "/dev/ttyUSB0"
BAUDRATE = 1000000

GRIPPER_ABIERTO = 100
GRIPPER_CERRADO = 15
VELOCIDAD = 30


def main():
    if len(sys.argv) != 2:
        print("Uso:")
        print("  python3 probar_gripper.py abrir")
        print("  python3 probar_gripper.py cerrar")
        sys.exit(1)

    accion = sys.argv[1].strip().lower()

    if accion not in ("abrir", "cerrar"):
        print(f"Accion desconocida: {accion}")
        print("Opciones: abrir | cerrar")
        sys.exit(1)

    print(f"Conectando con JetCobot en {PUERTO} @ {BAUDRATE}...")
    mc = MyCobot(PUERTO, BAUDRATE)

    time.sleep(1.0)

    if accion == "abrir":
        valor = GRIPPER_ABIERTO
    else:
        valor = GRIPPER_CERRADO

    print(
        f"Gripper -> {accion.upper()} "
        f"valor={valor} velocidad={VELOCIDAD}"
    )

    mc.set_gripper_value(valor, VELOCIDAD)

    time.sleep(2.0)

    print("Comando enviado.")


if __name__ == "__main__":
    main()
