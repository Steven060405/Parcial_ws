#!/usr/bin/env bash

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

echo "========================================"
echo " PREPARANDO PARCIAL_ROBOTICA"
echo "========================================"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "[ERROR] No encuentro ROS 2 Humble en:"
    echo "        /opt/ros/humble/setup.bash"
    exit 1
fi

source /opt/ros/humble/setup.bash

MODO="${1:-all}"

case "$MODO" in
    p1)
        echo
        echo "[INFO] Compilando Pregunta 1..."
        colcon build \
            --symlink-install \
            --packages-up-to parcial_interprete
        ;;

    p2)
        echo
        echo "[INFO] Compilando Pregunta 2..."
        colcon build \
            --symlink-install \
            --packages-up-to arm_broker percepcion_item2
        ;;

    all)
        echo
        echo "[INFO] Compilando workspace completo..."
        colcon build --symlink-install
        ;;

    *)
        echo
        echo "Uso:"
        echo "  ./preparar.sh p1"
        echo "  ./preparar.sh p2"
        echo "  ./preparar.sh all"
        exit 2
        ;;
esac

echo
echo "========================================"
echo " COMPILACION TERMINADA"
echo "========================================"
echo
echo "Ahora ejecuta:"
echo "  source install/setup.bash"
