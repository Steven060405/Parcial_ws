#!/usr/bin/env bash

# ============================================================
# PREGUNTA 2 - ENVIAR ORDEN EN ESPANOL
# ============================================================

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-92}"
export ROS_LOCALHOST_ONLY=0
unset ROS_DISCOVERY_SERVER || true

# ------------------------------------------------------------
# FRASE
# ------------------------------------------------------------

if [[ $# -eq 0 ]]; then
    echo
    read -rp "Orden: " FRASE
else
    FRASE="$*"
fi

if [[ -z "${FRASE// }" ]]; then
    echo "ERROR: orden vacia"
    exit 1
fi

echo
echo "============================================"
echo "        EJECUTANDO PREGUNTA 2"
echo "============================================"
echo
echo "Orden:"
echo "  $FRASE"
echo

# ------------------------------------------------------------
# COMPROBAR P2
# ------------------------------------------------------------

if ! ros2 service list | grep -qx "/interpretar_orden"; then
    echo "ERROR: /interpretar_orden no disponible"
    echo "Ejecuta primero:"
    echo "  ./iniciar_item2.sh IP_LAYA"
    exit 1
fi

if ! ros2 service list | grep -qx "/localizar_objeto"; then
    echo "ERROR: /localizar_objeto no disponible"
    exit 1
fi

if ! ros2 action list | grep -qx "/orden_brazo"; then
    echo "ERROR: /orden_brazo no disponible"
    exit 1
fi

# ------------------------------------------------------------
# P1 -> P2
# ------------------------------------------------------------

python3 "$ROOT/herramientas/orden_desde_texto.py" "$FRASE"

CODIGO=$?

echo

if [[ "$CODIGO" -eq 0 ]]; then
    echo "============================================"
    echo "          P2 FINALIZADA OK"
    echo "============================================"
else
    echo "============================================"
    echo "        P2 TERMINO CON ERROR"
    echo "        codigo=$CODIGO"
    echo "============================================"
fi

exit "$CODIGO"
