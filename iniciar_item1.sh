#!/usr/bin/env bash

set -eo pipefail

# ============================================================
# ITEM 1 - INTERPRETE DE ORDENES + LAYA
# Uso:
#   ./iniciar_item1.sh IP_WINDOWS
#
# Ejemplo:
#   ./iniciar_item1.sh 192.168.1.50
#
# Opcionalmente:
#   ./iniciar_item1.sh 192.168.1.50 62
# donde 62 es el ROS_DOMAIN_ID.
# ============================================================

PROYECTO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

LAYA_HOST="${1:-}"
DOMAIN_ID="${2:-62}"
LAYA_PORT="${LAYA_PORT:-8000}"
TIMEOUT="${LAYA_TIMEOUT:-5.0}"
MODELO="typed-decisions"

# ------------------------------------------------------------
# Validar IP / host de LAYA
# ------------------------------------------------------------

if [ -z "$LAYA_HOST" ]; then
    echo
    echo "ERROR: debes indicar la IP de la PC Windows donde corre LAYA."
    echo
    echo "Uso:"
    echo "  ./iniciar_item1.sh IP_WINDOWS"
    echo
    echo "Ejemplo:"
    echo "  ./iniciar_item1.sh 192.168.1.50"
    echo
    exit 1
fi

# Permite pasar:
#   192.168.1.50
# o:
#   http://192.168.1.50:8000

if [[ "$LAYA_HOST" == http://* || "$LAYA_HOST" == https://* ]]; then
    LAYA_URL="$LAYA_HOST"
else
    LAYA_URL="http://${LAYA_HOST}:${LAYA_PORT}"
fi

# ------------------------------------------------------------
# ROS 2
# ------------------------------------------------------------

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: no existe /opt/ros/humble/setup.bash"
    exit 1
fi

source /opt/ros/humble/setup.bash

if [ ! -f "$PROYECTO/install/setup.bash" ]; then
    echo
    echo "ERROR: el workspace todavía no está compilado."
    echo "Ejecuta:"
    echo
    echo "  cd $PROYECTO"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  colcon build --symlink-install"
    echo
    exit 1
fi

source "$PROYECTO/install/setup.bash"

# ------------------------------------------------------------
# CONFIGURACION ROS
# ------------------------------------------------------------

export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0

# Para esta prueba no utilizamos servidor de descubrimiento.
unset ROS_DISCOVERY_SERVER

# ------------------------------------------------------------
# MOSTRAR CONFIGURACION
# ------------------------------------------------------------

echo
echo "============================================================"
echo " ITEM 1 - INTERPRETE DE ORDENES"
echo "============================================================"
echo "Proyecto       : $PROYECTO"
echo "ROS_DOMAIN_ID  : $ROS_DOMAIN_ID"
echo "LAYA URL       : $LAYA_URL"
echo "Modelo LAYA    : $MODELO"
echo "Timeout        : $TIMEOUT s"
echo "Servicio ROS   : /interpretar_orden"
echo "============================================================"
echo
echo "Iniciando interprete_ordenes..."
echo

# ------------------------------------------------------------
# EJECUTAR NODO
# ------------------------------------------------------------

exec ros2 run parcial_interprete interprete_ordenes \
    --ros-args \
    -p laya_url:="$LAYA_URL" \
    -p laya_model:="$MODELO" \
    -p timeout_s:="$TIMEOUT"
