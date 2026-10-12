#!/usr/bin/env bash
set -eo pipefail

BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="$BASE/config/poses_item2.env"

POSE="${1:-}"
PRIORIDAD="${2:-10}"
DOMAIN_ID="${ROS_DOMAIN_ID:-62}"

if [ -z "$POSE" ]; then
    echo "Uso:"
    echo "  ./probar_pose_item2.sh rojo"
    echo "  ./probar_pose_item2.sh verde"
    echo "  ./probar_pose_item2.sh azul"
    echo "  ./probar_pose_item2.sh amarillo"
    echo "  ./probar_pose_item2.sh home"
    echo "  ./probar_pose_item2.sh zona_a"
    exit 1
fi

if [ ! -f "$CONFIG" ]; then
    echo "ERROR: no existe $CONFIG"
    exit 1
fi

# ROS 2
source /opt/ros/humble/setup.bash
source "$BASE/install/setup.bash"

export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0

# Poses
# shellcheck disable=SC1090
source "$CONFIG"

POSE_UPPER="$(echo "$POSE" | tr '[:lower:]' '[:upper:]')"

case "$POSE_UPPER" in
    ROJO|VERDE|AZUL|AMARILLO|HOME|ZONA_A|INSPECCION|SEGURA)
        ;;
    *)
        echo "Pose no reconocida: $POSE"
        echo "Opciones: rojo verde azul amarillo home zona_a inspeccion segura inspeccion segura"
        exit 1
        ;;
esac

J1_VAR="${POSE_UPPER}_J1"
J2_VAR="${POSE_UPPER}_J2"
J3_VAR="${POSE_UPPER}_J3"
J4_VAR="${POSE_UPPER}_J4"
J5_VAR="${POSE_UPPER}_J5"
J6_VAR="${POSE_UPPER}_J6"

J1="${!J1_VAR}"
J2="${!J2_VAR}"
J3="${!J3_VAR}"
J4="${!J4_VAR}"
J5="${!J5_VAR}"
J6="${!J6_VAR}"

echo
echo "=============================================="
echo " ITEM 2 - PRUEBA DE POSE: $POSE_UPPER"
echo "=============================================="
echo "Grados:"
echo "J1=$J1"
echo "J2=$J2"
echo "J3=$J3"
echo "J4=$J4"
echo "J5=$J5"
echo "J6=$J6"
echo

# Conversión grados -> radianes
RADIANES="$(python3 - "$J1" "$J2" "$J3" "$J4" "$J5" "$J6" <<'PY'
import math
import sys

grados = [float(x) for x in sys.argv[1:]]
radianes = [math.radians(x) for x in grados]

print(",".join(f"{x:.8f}" for x in radianes))
PY
)"

IFS=',' read -r R1 R2 R3 R4 R5 R6 <<< "$RADIANES"

echo "Radianes enviados al broker:"
echo "[$R1, $R2, $R3, $R4, $R5, $R6]"
echo

echo "Comprobando /move_arm..."

if ! ros2 action list 2>/dev/null | grep -qx "/move_arm"; then
    echo
    echo "ERROR: /move_arm no está disponible."
    echo "Primero inicia arm_broker."
    exit 1
fi

echo "Enviando pose mediante arm_broker..."
echo

ros2 action send_goal \
    /move_arm \
    arm_broker_interfaces/action/MoveArm \
    "{joint_positions: [$R1, $R2, $R3, $R4, $R5, $R6], client_id: 'prueba_${POSE}', priority: $PRIORIDAD}" \
    --feedback
