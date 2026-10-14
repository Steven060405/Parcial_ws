#!/usr/bin/env bash

# ============================================================
# PREGUNTA 2
# INICIALIZADOR JETCOBOT + MOVEIT2
#
# Uso normal en Jetson:
#   ./iniciar_item2.sh
#
# Verificacion sin hardware:
#   ./iniciar_item2.sh --check
# ============================================================

set -eo pipefail

ROOT="$HOME/Parcial_ws"
VENDOR_WS="$HOME/jetcobot_colcon_ws"

cd "$ROOT"

# ------------------------------------------------------------
# MODO
# ------------------------------------------------------------

CHECK_ONLY=0

if [[ "${1:-}" == "--check" ]]; then
    CHECK_ONLY=1
    shift
fi

# ------------------------------------------------------------
# CONFIGURACION COMUN
# ------------------------------------------------------------

ROS_ID=92

LAYA_PORT=8000
LAYA_MODEL="typed-decisions"
LAYA_TIMEOUT=5.0

CAMERA_INDEX=0

SERIAL_PORT="/dev/ttyUSB0"
SERIAL_BAUD=1000000

VELOCIDAD_GRIPPER=30
GRIPPER_ABIERTO=100
GRIPPER_CERRADO=15

POLITICA="prioridad"

# ------------------------------------------------------------
# ROS 2
# ------------------------------------------------------------

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ERROR: ROS 2 Humble no encontrado."
    exit 1
fi

source /opt/ros/humble/setup.bash

# El workspace del JetCobot puede no existir en VirtualBox.
VENDOR_DISPONIBLE=0

if [[ -f "$VENDOR_WS/install/setup.bash" ]]; then
    source "$VENDOR_WS/install/setup.bash"
    VENDOR_DISPONIBLE=1
fi

# Nuestro workspace SI debe estar compilado.
if [[ ! -f "$ROOT/install/setup.bash" ]]; then
    echo "ERROR: Parcial_ws no esta compilado."
    echo
    echo "Ejecuta:"
    echo "  cd ~/Parcial_ws"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  colcon build --symlink-install"
    exit 1
fi

source "$ROOT/install/setup.bash"

export ROS_DOMAIN_ID="$ROS_ID"
export ROS_LOCALHOST_ONLY=0

unset ROS_DISCOVERY_SERVER || true

# ============================================================
# FUNCIONES
# ============================================================

existe_linea() {

    local BUSCADA="$1"
    local TEXTO="$2"
    local LINEA

    while IFS= read -r LINEA; do

        if [[ "$LINEA" == "$BUSCADA" ]]; then
            return 0
        fi

    done <<< "$TEXTO"

    return 1
}


comprobar_paquete() {

    ros2 pkg prefix "$1" >/dev/null 2>&1
}


esperar_nodo() {

    local NOMBRE="$1"

    for _ in $(seq 1 40); do

        SALIDA="$(ros2 node list 2>/dev/null || true)"

        if existe_linea "$NOMBRE" "$SALIDA"; then
            return 0
        fi

        sleep 0.5

    done

    return 1
}


esperar_servicio() {

    local NOMBRE="$1"

    for _ in $(seq 1 40); do

        SALIDA="$(ros2 service list 2>/dev/null || true)"

        if existe_linea "$NOMBRE" "$SALIDA"; then
            return 0
        fi

        sleep 0.5

    done

    return 1
}


esperar_action() {

    local NOMBRE="$1"

    for _ in $(seq 1 40); do

        SALIDA="$(ros2 action list 2>/dev/null || true)"

        if existe_linea "$NOMBRE" "$SALIDA"; then
            return 0
        fi

        sleep 0.5

    done

    return 1
}

# ============================================================
# CHECK SIN JETCOBOT
# ============================================================

if [[ "$CHECK_ONLY" -eq 1 ]]; then

    echo
    echo "============================================================"
    echo "              CHECK P2 SIN JETCOBOT"
    echo "============================================================"
    echo

    FALLO=0

    # --------------------------------------------------------
    # Paquetes propios
    # --------------------------------------------------------

    PAQUETES_PROPIOS=(
        arm_broker
        percepcion_item2
        planificador_item2
        parcial_interprete
        arm_broker_interfaces
        parcial_interfaces
    )

    for PAQUETE in "${PAQUETES_PROPIOS[@]}"; do

        if comprobar_paquete "$PAQUETE"; then
            echo "[OK] paquete $PAQUETE"
        else
            echo "[ERROR] falta paquete $PAQUETE"
            FALLO=1
        fi

    done

    echo

    # --------------------------------------------------------
    # MoveIt2 general
    # --------------------------------------------------------

    if comprobar_paquete moveit_ros_planning_interface; then
        echo "[OK] MoveIt2 planning interface"
    else
        echo "[ERROR] falta moveit_ros_planning_interface"
        FALLO=1
    fi

    if comprobar_paquete moveit_configs_utils; then
        echo "[OK] moveit_configs_utils"
    else
        echo "[ERROR] falta moveit_configs_utils"
        FALLO=1
    fi

    echo

    # --------------------------------------------------------
    # Software especifico del Jetson
    # --------------------------------------------------------

    if [[ "$VENDOR_DISPONIBLE" -eq 1 ]]; then
        echo "[OK] ~/jetcobot_colcon_ws"
    else
        echo "[PENDIENTE JETSON] ~/jetcobot_colcon_ws"
    fi

    if comprobar_paquete jetcobot_moveit; then

        echo "[OK] jetcobot_moveit"

        MOVEIT_PREFIX="$(ros2 pkg prefix jetcobot_moveit)"
        MOVEIT_SHARE="$MOVEIT_PREFIX/share/jetcobot_moveit"

        if [[ -f "$MOVEIT_SHARE/launch/rsp.launch.py" ]]; then
            echo "[OK] rsp.launch.py"
        else
            echo "[AVISO JETSON] rsp.launch.py no encontrado"
        fi

        if [[ -f "$MOVEIT_SHARE/launch/move_group.launch.py" ]]; then
            echo "[OK] move_group.launch.py"
        else
            echo "[AVISO JETSON] move_group.launch.py no encontrado"
        fi

        if [[ -f "$MOVEIT_SHARE/launch/demo.launch.py" ]]; then
            echo "[OK] demo.launch.py disponible como alternativa"
        fi

    else

        echo "[PENDIENTE JETSON] jetcobot_moveit"

    fi

    echo

    # --------------------------------------------------------
    # Hardware
    # --------------------------------------------------------

    if [[ -e /dev/ttyUSB0 ]]; then
        echo "[OK] /dev/ttyUSB0"
    else
        echo "[PENDIENTE HARDWARE] /dev/ttyUSB0"
    fi

    if [[ -e /dev/video0 ]]; then
        echo "[OK] /dev/video0"
    else
        echo "[PENDIENTE HARDWARE] /dev/video0"
    fi

    echo

    # --------------------------------------------------------
    # Bash
    # --------------------------------------------------------

    if bash -n "$ROOT/iniciar_item2.sh"; then
        echo "[OK] sintaxis iniciar_item2.sh"
    else
        echo "[ERROR] iniciar_item2.sh"
        FALLO=1
    fi

    if bash -n "$ROOT/probar_item2.sh"; then
        echo "[OK] sintaxis probar_item2.sh"
    else
        echo "[ERROR] probar_item2.sh"
        FALLO=1
    fi

    echo

    # --------------------------------------------------------
    # Python
    # --------------------------------------------------------

    if python3 -m py_compile \
        "$ROOT/src/arm_broker/arm_broker/broker.py"
    then
        echo "[OK] broker.py"
    else
        echo "[ERROR] broker.py"
        FALLO=1
    fi

    if python3 -m py_compile \
        "$ROOT/herramientas/orden_desde_texto.py"
    then
        echo "[OK] orden_desde_texto.py"
    else
        echo "[ERROR] orden_desde_texto.py"
        FALLO=1
    fi

    if python3 -m py_compile \
        "$ROOT/herramientas/benchmark_item2_10.py"
    then
        echo "[OK] benchmark_item2_10.py"
    else
        echo "[ERROR] benchmark_item2_10.py"
        FALLO=1
    fi

    if python3 -m py_compile \
        "$ROOT/src/planificador_item2/launch/planificador.launch.py"
    then
        echo "[OK] planificador.launch.py"
    else
        echo "[ERROR] planificador.launch.py"
        FALLO=1
    fi

    echo

    # --------------------------------------------------------
    # Resultado
    # --------------------------------------------------------

    if [[ "$FALLO" -eq 0 ]]; then

        echo "============================================================"
        echo "                    CHECK P2 OK"
        echo "============================================================"
        echo
        echo "Software validado en esta maquina."
        echo
        echo "Los elementos marcados como:"
        echo "  [PENDIENTE JETSON]"
        echo "  [PENDIENTE HARDWARE]"
        echo
        echo "se verificaran cuando vuelvas al Jetson."
        echo

        exit 0

    fi

    echo "============================================================"
    echo "               CHECK P2 CON ERRORES"
    echo "============================================================"

    exit 1

fi

# ============================================================
# DESDE AQUI ES MODO REAL CON JETCOBOT
# ============================================================

# ------------------------------------------------------------
# Comprobar software especifico del Jetson
# ------------------------------------------------------------

if [[ "$VENDOR_DISPONIBLE" -ne 1 ]]; then

    echo "ERROR: no existe:"
    echo "  $VENDOR_WS/install/setup.bash"
    echo
    echo "Este modo debe ejecutarse en el Jetson configurado."
    echo
    echo "Sin JetCobot usa:"
    echo "  ./iniciar_item2.sh --check"

    exit 1
fi


if ! comprobar_paquete jetcobot_moveit; then

    echo "ERROR: paquete jetcobot_moveit no encontrado."
    exit 1

fi


if ! command -v setsid >/dev/null 2>&1; then

    echo "ERROR: comando setsid no disponible."
    exit 1

fi


if ! command -v fuser >/dev/null 2>&1; then

    echo "ERROR: comando fuser no disponible."
    exit 1

fi

# ------------------------------------------------------------
# IP LAYA Y VELOCIDAD
# ------------------------------------------------------------

LAYA_IP="${1:-}"
VELOCIDAD_ROBOT="${2:-}"


if [[ -z "$LAYA_IP" ]]; then

    read -rp "IP de LAYA: " LAYA_IP

fi


if [[ -z "$VELOCIDAD_ROBOT" ]]; then

    read -rp "Velocidad del brazo [30]: " VELOCIDAD_ROBOT

    VELOCIDAD_ROBOT="${VELOCIDAD_ROBOT:-30}"

fi


if [[ -z "${LAYA_IP// }" ]]; then

    echo "ERROR: IP de LAYA vacia."
    exit 1

fi


if ! [[ "$VELOCIDAD_ROBOT" =~ ^[0-9]+$ ]]; then

    echo "ERROR: velocidad invalida."
    exit 1

fi


if (( VELOCIDAD_ROBOT < 1 || VELOCIDAD_ROBOT > 100 )); then

    echo "ERROR: velocidad debe estar entre 1 y 100."
    exit 1

fi

# ------------------------------------------------------------
# Comprobar paquetes
# ------------------------------------------------------------

PAQUETES_REALES=(
    jetcobot_moveit
    arm_broker
    percepcion_item2
    planificador_item2
    parcial_interprete
)

for PAQUETE in "${PAQUETES_REALES[@]}"; do

    if ! comprobar_paquete "$PAQUETE"; then

        echo "ERROR: paquete no encontrado:"
        echo "  $PAQUETE"

        exit 1

    fi

done

# ============================================================
# EVITAR PROCESOS DUPLICADOS
# ============================================================

NODOS_ANTES="$(ros2 node list 2>/dev/null || true)"

NODOS_CRITICOS=(
    /arm_broker
    /percepcion_camara
    /planificador_moveit
    /move_group
    /interprete_ordenes
)

for NODO in "${NODOS_CRITICOS[@]}"; do

    if existe_linea "$NODO" "$NODOS_ANTES"; then

        echo "ERROR: ya existe el nodo:"
        echo "  $NODO"
        echo
        echo "Hay otra ejecucion de P2 activa."
        echo "Cierra primero la ejecucion anterior."

        exit 1

    fi

done

# ============================================================
# HARDWARE
# ============================================================

echo
echo "============================================================"
echo "        PREGUNTA 2 - JETCOBOT + MOVEIT2"
echo "============================================================"
echo
echo "ROS_DOMAIN_ID      = $ROS_DOMAIN_ID"
echo "LAYA               = http://${LAYA_IP}:${LAYA_PORT}"
echo "Camara             = /dev/video${CAMERA_INDEX}"
echo "JetCobot           = $SERIAL_PORT"
echo "Baudrate           = $SERIAL_BAUD"
echo "Velocidad brazo    = $VELOCIDAD_ROBOT"
echo "Velocidad gripper  = $VELOCIDAD_GRIPPER"
echo "Politica broker    = $POLITICA"
echo


if [[ ! -e "$SERIAL_PORT" ]]; then

    echo "ERROR: no existe $SERIAL_PORT"
    exit 1

fi


if [[ ! -e "/dev/video${CAMERA_INDEX}" ]]; then

    echo "ERROR: no existe /dev/video${CAMERA_INDEX}"
    exit 1

fi


if fuser -s "$SERIAL_PORT" 2>/dev/null; then

    echo "ERROR: $SERIAL_PORT ya esta siendo usado."
    fuser -v "$SERIAL_PORT" || true

    exit 1

fi


if fuser -s "/dev/video${CAMERA_INDEX}" 2>/dev/null; then

    echo "ERROR: /dev/video${CAMERA_INDEX} ya esta siendo usada."
    fuser -v "/dev/video${CAMERA_INDEX}" || true

    exit 1

fi

# ============================================================
# LOGS
# ============================================================

mkdir -p "$ROOT/registros"

STAMP="$(date +%Y%m%d_%H%M%S)"

LOG_RSP="$ROOT/registros/p2_rsp_${STAMP}.log"

LOG_MOVEIT="$ROOT/registros/p2_moveit_${STAMP}.log"

LOG_PLANIFICADOR="$ROOT/registros/p2_planificador_${STAMP}.log"

LOG_PERCEPCION="$ROOT/registros/p2_percepcion_${STAMP}.log"

LOG_BROKER="$ROOT/registros/p2_broker_${STAMP}.log"

LOG_INTERPRETE="$ROOT/registros/p2_interprete_${STAMP}.log"

PIDS=()

# ============================================================
# INICIAR PROCESOS EN GRUPOS
# ============================================================

iniciar_grupo() {

    local LOG="$1"

    shift

    setsid "$@" >"$LOG" 2>&1 &

    local PID=$!

    PIDS+=("$PID")
}

# ============================================================
# CLEANUP
# ============================================================

cleanup() {

    trap - EXIT INT TERM

    echo
    echo "============================================================"
    echo "Cerrando Pregunta 2..."
    echo "============================================================"

    for PID in "${PIDS[@]:-}"; do

        kill -TERM -- "-$PID" 2>/dev/null || true

    done

    sleep 2

    for PID in "${PIDS[@]:-}"; do

        kill -KILL -- "-$PID" 2>/dev/null || true

    done

    wait 2>/dev/null || true

    echo "Procesos cerrados."
}

trap cleanup EXIT INT TERM

# ============================================================
# MOVEIT2
# ============================================================

MOVEIT_PREFIX="$(ros2 pkg prefix jetcobot_moveit)"
MOVEIT_SHARE="$MOVEIT_PREFIX/share/jetcobot_moveit"


if [[ -f "$MOVEIT_SHARE/launch/rsp.launch.py" ]] && \
   [[ -f "$MOVEIT_SHARE/launch/move_group.launch.py" ]]
then

    # --------------------------------------------------------
    # 1. ROBOT STATE PUBLISHER
    # --------------------------------------------------------

    echo "[1/6] Levantando robot_state_publisher..."

    iniciar_grupo \
        "$LOG_RSP" \
        ros2 launch jetcobot_moveit rsp.launch.py


    if ! esperar_nodo "/robot_state_publisher"; then

        echo "ERROR: /robot_state_publisher no levanto."
        echo "Log:"
        echo "  $LOG_RSP"

        exit 1

    fi

    # --------------------------------------------------------
    # 2. MOVE GROUP
    # --------------------------------------------------------

    echo "[2/6] Levantando MoveIt2 move_group..."

    iniciar_grupo \
        "$LOG_MOVEIT" \
        ros2 launch jetcobot_moveit move_group.launch.py


else

    # --------------------------------------------------------
    # FALLBACK: DEMO.LAUNCH.PY
    # --------------------------------------------------------

    if [[ ! -f "$MOVEIT_SHARE/launch/demo.launch.py" ]]; then

        echo "ERROR: no se encontraron launch de MoveIt2."
        exit 1

    fi

    echo "[1-2/6] Usando jetcobot_moveit demo.launch.py..."

    iniciar_grupo \
        "$LOG_MOVEIT" \
        ros2 launch jetcobot_moveit demo.launch.py

fi


if ! esperar_nodo "/move_group"; then

    echo "ERROR: /move_group no levanto."
    echo "Log:"
    echo "  $LOG_MOVEIT"

    exit 1

fi

# ============================================================
# 3. PLANIFICADOR MOVEIT2
# ============================================================

echo "[3/6] Levantando planificador_item2..."

iniciar_grupo \
    "$LOG_PLANIFICADOR" \
    ros2 launch planificador_item2 planificador.launch.py


if ! esperar_servicio "/planificar_pose"; then

    echo "ERROR: /planificar_pose no disponible."
    echo "Log:"
    echo "  $LOG_PLANIFICADOR"

    exit 1

fi

# ============================================================
# 4. PERCEPCION
# ============================================================

echo "[4/6] Levantando percepcion_camara..."

iniciar_grupo \
    "$LOG_PERCEPCION" \
    ros2 run percepcion_item2 percepcion_camara \
    --ros-args \
    -p camera_index:="$CAMERA_INDEX" \
    -p area_min_px:=500.0


if ! esperar_servicio "/localizar_objeto"; then

    echo "ERROR: /localizar_objeto no disponible."
    echo "Log:"
    echo "  $LOG_PERCEPCION"

    exit 1

fi

# ============================================================
# 5. BROKER
# ============================================================

echo "[5/6] Levantando arm_broker..."

iniciar_grupo \
    "$LOG_BROKER" \
    ros2 run arm_broker broker \
    --ros-args \
    -p politica:="$POLITICA" \
    -p hardware_real:=true \
    -p serial_port:="$SERIAL_PORT" \
    -p serial_baud:="$SERIAL_BAUD" \
    -p velocidad_robot:="$VELOCIDAD_ROBOT" \
    -p velocidad_gripper:="$VELOCIDAD_GRIPPER" \
    -p gripper_abierto:="$GRIPPER_ABIERTO" \
    -p gripper_cerrado:="$GRIPPER_CERRADO"


if ! esperar_action "/orden_brazo"; then

    echo "ERROR: /orden_brazo no disponible."
    echo "Log:"
    echo "  $LOG_BROKER"

    exit 1

fi

# ============================================================
# 6. INTERPRETE P1
# ============================================================

echo "[6/6] Levantando interprete_ordenes..."

iniciar_grupo \
    "$LOG_INTERPRETE" \
    ros2 run parcial_interprete interprete_ordenes \
    --ros-args \
    -p laya_url:="http://${LAYA_IP}:${LAYA_PORT}" \
    -p laya_model:="$LAYA_MODEL" \
    -p timeout_s:="$LAYA_TIMEOUT"


if ! esperar_servicio "/interpretar_orden"; then

    echo "ERROR: /interpretar_orden no disponible."
    echo "Log:"
    echo "  $LOG_INTERPRETE"

    exit 1

fi

# ============================================================
# VERIFICACION FINAL
# ============================================================

echo
echo "============================================================"
echo "                  VERIFICACION P2"
echo "============================================================"
echo

NODOS="$(ros2 node list 2>/dev/null || true)"

SERVICIOS="$(ros2 service list 2>/dev/null || true)"

ACTIONS="$(ros2 action list 2>/dev/null || true)"

TOPICS="$(ros2 topic list 2>/dev/null || true)"

FALLO=0

# ------------------------------------------------------------
# Nodos
# ------------------------------------------------------------

NODOS_ESPERADOS=(
    /arm_broker
    /move_group
    /percepcion_camara
    /planificador_moveit
)

for NODO in "${NODOS_ESPERADOS[@]}"; do

    if existe_linea "$NODO" "$NODOS"; then

        echo "[OK] nodo $NODO"

    else

        echo "[ERROR] falta nodo $NODO"
        FALLO=1

    fi

done

# ------------------------------------------------------------
# Servicios
# ------------------------------------------------------------

SERVICIOS_ESPERADOS=(
    /interpretar_orden
    /localizar_objeto
    /planificar_pose
)

for SERVICIO in "${SERVICIOS_ESPERADOS[@]}"; do

    if existe_linea "$SERVICIO" "$SERVICIOS"; then

        echo "[OK] servicio $SERVICIO"

    else

        echo "[ERROR] falta servicio $SERVICIO"
        FALLO=1

    fi

done

# ------------------------------------------------------------
# Action
# ------------------------------------------------------------

if existe_linea "/orden_brazo" "$ACTIONS"; then

    echo "[OK] action /orden_brazo"

else

    echo "[ERROR] falta action /orden_brazo"
    FALLO=1

fi

# ------------------------------------------------------------
# Topics
# ------------------------------------------------------------

if existe_linea "/joint_states" "$TOPICS"; then

    echo "[OK] topic /joint_states"

else

    echo "[ERROR] falta topic /joint_states"
    FALLO=1

fi


if existe_linea "/arm/queue_state" "$TOPICS"; then

    echo "[OK] topic /arm/queue_state"

else

    echo "[ERROR] falta topic /arm/queue_state"
    FALLO=1

fi

# ------------------------------------------------------------
# Comprobar unico servidor del broker
# ------------------------------------------------------------

INFO_ACTION="$(ros2 action info /orden_brazo 2>/dev/null || true)"

SERVERS_ACTION="$(
    awk -F': ' '/Action servers:/ {print $2}' <<< "$INFO_ACTION"
)"

if [[ -n "$SERVERS_ACTION" ]]; then

    if [[ "$SERVERS_ACTION" == "1" ]]; then

        echo "[OK] /orden_brazo tiene un unico servidor"

    else

        echo "[ERROR] /orden_brazo tiene $SERVERS_ACTION servidores"
        FALLO=1

    fi

fi

# ------------------------------------------------------------
# Comprobar unico publicador de joint_states
# ------------------------------------------------------------

INFO_JOINT="$(
    ros2 topic info /joint_states 2>/dev/null || true
)"

PUBLISHERS_JOINT="$(
    awk -F': ' '/Publisher count:/ {print $2}' <<< "$INFO_JOINT"
)"

if [[ -n "$PUBLISHERS_JOINT" ]]; then

    if [[ "$PUBLISHERS_JOINT" == "1" ]]; then

        echo "[OK] /joint_states tiene un unico publicador"

    else

        echo "[ERROR] /joint_states tiene $PUBLISHERS_JOINT publicadores"
        echo "Esto puede causar movimientos/estados inconsistentes."

        FALLO=1

    fi

fi

# ------------------------------------------------------------
# Resultado
# ------------------------------------------------------------

echo

if [[ "$FALLO" -ne 0 ]]; then

    echo "============================================================"
    echo "            P2 NO LEVANTO CORRECTAMENTE"
    echo "============================================================"

    exit 1

fi


echo "============================================================"
echo "             P2 LISTA PARA EJECUTAR"
echo "============================================================"
echo
echo "En otra terminal:"
echo
echo "  cd ~/Parcial_ws"
echo "  ./probar_item2.sh \"recoge el cubo rojo\""
echo
echo "Para observar la cola:"
echo
echo "  ros2 topic echo /arm/queue_state"
echo
echo "Logs:"
echo "  MoveIt2:      $LOG_MOVEIT"
echo "  Planificador: $LOG_PLANIFICADOR"
echo "  Percepcion:   $LOG_PERCEPCION"
echo "  Broker:       $LOG_BROKER"
echo "  Interprete:   $LOG_INTERPRETE"
echo
echo "Ctrl+C aqui apaga toda la Pregunta 2."
echo

while true; do
    sleep 5
done

