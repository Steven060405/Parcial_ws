#!/usr/bin/env bash

# ============================================================
# PREGUNTA 2 - INICIALIZADOR COMPLETO JETCOBOT
# ============================================================

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

# ------------------------------------------------------------
# CONFIGURACION
# ------------------------------------------------------------

ROS_ID="${ROS_DOMAIN_ID:-92}"

LAYA_IP="${1:-127.0.0.1}"
LAYA_PORT="${LAYA_PORT:-8000}"
LAYA_MODEL="${LAYA_MODEL:-typed-decisions}"
LAYA_TIMEOUT="${LAYA_TIMEOUT:-5.0}"

CAMERA_INDEX="${CAMERA_INDEX:-0}"

SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
SERIAL_BAUD="${SERIAL_BAUD:-1000000}"

VELOCIDAD_ROBOT="${VELOCIDAD_ROBOT:-30}"
VELOCIDAD_GRIPPER="${VELOCIDAD_GRIPPER:-30}"

GRIPPER_ABIERTO="${GRIPPER_ABIERTO:-100}"
GRIPPER_CERRADO="${GRIPPER_CERRADO:-15}"

POLITICA="${POLITICA:-prioridad}"

# ------------------------------------------------------------
# ROS 2
# ------------------------------------------------------------

source /opt/ros/humble/setup.bash

if [[ ! -f "$ROOT/install/setup.bash" ]]; then
    echo "ERROR: No existe install/setup.bash"
    echo "Compila primero:"
    echo "  colcon build --symlink-install"
    exit 1
fi

source "$ROOT/install/setup.bash"

export ROS_DOMAIN_ID="$ROS_ID"
export ROS_LOCALHOST_ONLY=0
unset ROS_DISCOVERY_SERVER || true

# ------------------------------------------------------------
# COMPROBAR HARDWARE
# ------------------------------------------------------------

echo
echo "============================================"
echo "       PREGUNTA 2 - JETCOBOT"
echo "============================================"
echo
echo "ROS_DOMAIN_ID     = $ROS_DOMAIN_ID"
echo "LAYA              = http://${LAYA_IP}:${LAYA_PORT}"
echo "Camara            = /dev/video${CAMERA_INDEX}"
echo "JetCobot          = $SERIAL_PORT"
echo "Baudrate          = $SERIAL_BAUD"
echo "Velocidad brazo   = $VELOCIDAD_ROBOT"
echo "Gripper abierto   = $GRIPPER_ABIERTO"
echo "Gripper cerrado   = $GRIPPER_CERRADO"
echo "Velocidad gripper = $VELOCIDAD_GRIPPER"
echo "Politica          = $POLITICA"
echo

if [[ ! -e "$SERIAL_PORT" ]]; then
    echo "ERROR: No existe $SERIAL_PORT"
    exit 1
fi

if [[ ! -e "/dev/video${CAMERA_INDEX}" ]]; then
    echo "ERROR: No existe /dev/video${CAMERA_INDEX}"
    exit 1
fi

# ------------------------------------------------------------
# LOGS
# ------------------------------------------------------------

mkdir -p "$ROOT/registros"

STAMP="$(date +%Y%m%d_%H%M%S)"

LOG_INTERPRETE="$ROOT/registros/p2_interprete_${STAMP}.log"
LOG_PERCEPCION="$ROOT/registros/p2_percepcion_${STAMP}.log"
LOG_BROKER="$ROOT/registros/p2_broker_${STAMP}.log"

PIDS=()

cleanup() {
    echo
    echo "Cerrando Pregunta 2..."

    for pid in "${PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done

    wait 2>/dev/null || true

    echo "Procesos cerrados."
}

trap cleanup EXIT INT TERM

# ------------------------------------------------------------
# 1. PERCEPCION
# ------------------------------------------------------------

echo "[1/3] Levantando percepcion_camara..."

ros2 run percepcion_item2 percepcion_camara \
    --ros-args \
    -p camera_index:="$CAMERA_INDEX" \
    -p area_min_px:=500.0 \
    >"$LOG_PERCEPCION" 2>&1 &

PIDS+=("$!")

sleep 2

# ------------------------------------------------------------
# 2. BROKER
# ------------------------------------------------------------

echo "[2/3] Levantando arm_broker..."

ros2 run arm_broker broker \
    --ros-args \
    -p politica:="$POLITICA" \
    -p hardware_real:=true \
    -p serial_port:="$SERIAL_PORT" \
    -p serial_baud:="$SERIAL_BAUD" \
    -p velocidad_robot:="$VELOCIDAD_ROBOT" \
    -p velocidad_gripper:="$VELOCIDAD_GRIPPER" \
    -p gripper_abierto:="$GRIPPER_ABIERTO" \
    -p gripper_cerrado:="$GRIPPER_CERRADO" \
    >"$LOG_BROKER" 2>&1 &

PIDS+=("$!")

sleep 2

# ------------------------------------------------------------
# 3. INTERPRETE P1
# ------------------------------------------------------------

echo "[3/3] Levantando interprete_ordenes..."

ros2 run parcial_interprete interprete_ordenes \
    --ros-args \
    -p laya_url:="http://${LAYA_IP}:${LAYA_PORT}" \
    -p laya_model:="$LAYA_MODEL" \
    -p timeout_s:="$LAYA_TIMEOUT" \
    >"$LOG_INTERPRETE" 2>&1 &

PIDS+=("$!")

# ------------------------------------------------------------
# ESPERAR INICIALIZACION
# ------------------------------------------------------------

echo
echo "Esperando inicializacion..."
sleep 5

echo
echo "============================================"
echo "           VERIFICACION P2"
echo "============================================"

echo
echo "NODOS:"
ros2 node list || true

echo
echo "SERVICIOS:"
ros2 service list | grep -E "interpretar|localizar" || true

echo
echo "ACTIONS:"
ros2 action list -t || true

echo
echo "============================================"

# ------------------------------------------------------------
# COMPROBAR COMPONENTES CRITICOS
# ------------------------------------------------------------

FALLO=0

if ! ros2 node list | grep -qx "/arm_broker"; then
    echo "ERROR: /arm_broker no levanto"
    echo "Log: $LOG_BROKER"
    FALLO=1
fi

if ! ros2 node list | grep -qx "/percepcion_camara"; then
    echo "ERROR: /percepcion_camara no levanto"
    echo "Log: $LOG_PERCEPCION"
    FALLO=1
fi

if ! ros2 node list | grep -qx "/interprete_ordenes"; then
    echo "ERROR: /interprete_ordenes no levanto"
    echo "Log: $LOG_INTERPRETE"
    FALLO=1
fi

if ! ros2 service list | grep -qx "/localizar_objeto"; then
    echo "ERROR: /localizar_objeto no disponible"
    FALLO=1
fi

if ! ros2 service list | grep -qx "/interpretar_orden"; then
    echo "ERROR: /interpretar_orden no disponible"
    FALLO=1
fi

if ! ros2 action list | grep -qx "/orden_brazo"; then
    echo "ERROR: /orden_brazo no disponible"
    FALLO=1
fi

echo

if [[ "$FALLO" -ne 0 ]]; then
    echo "P2 NO levanto completamente."
    echo
    echo "Revisa:"
    echo "  tail -f $LOG_BROKER"
    echo "  tail -f $LOG_PERCEPCION"
    echo "  tail -f $LOG_INTERPRETE"
    exit 1
fi

echo "============================================"
echo "        P2 LISTA PARA EJECUTAR"
echo "============================================"
echo
echo "En OTRA terminal:"
echo
echo "  cd ~/Parcial_ws"
echo "  ./probar_item2.sh \"recoge el cubo rojo\""
echo
echo "Otros ejemplos:"
echo
echo "  ./probar_item2.sh \"recoge el cubo verde\""
echo "  ./probar_item2.sh \"recoge el cubo azul\""
echo "  ./probar_item2.sh \"recoge el cubo amarillo\""
echo
echo "Logs:"
echo "  Broker:     $LOG_BROKER"
echo "  Percepcion: $LOG_PERCEPCION"
echo "  Interprete: $LOG_INTERPRETE"
echo
echo "Ctrl+C para apagar P2."
echo

wait
