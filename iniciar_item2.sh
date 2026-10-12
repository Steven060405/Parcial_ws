#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash
if [[ ! -f install/setup.bash ]]; then
  colcon build --symlink-install --packages-up-to arm_broker percepcion_item2 parcial_interprete
fi
source install/setup.bash

read -rp "ROS_DOMAIN_ID [62]: " ROS_ID; ROS_ID="${ROS_ID:-62}"
export ROS_DOMAIN_ID="$ROS_ID"; export ROS_LOCALHOST_ONLY=0
read -rp "ROS_DISCOVERY_SERVER (Enter = no usar): " DISCOVERY
if [[ -n "$DISCOVERY" ]]; then export ROS_DISCOVERY_SERVER="$DISCOVERY"; else unset ROS_DISCOVERY_SERVER || true; fi
read -rp "IP LAYA [127.0.0.1]: " LAYA_IP; LAYA_IP="${LAYA_IP:-127.0.0.1}"
read -rp "Puerto LAYA [8000]: " LAYA_PORT; LAYA_PORT="${LAYA_PORT:-8000}"
read -rp "Modelo LAYA [typed-decisions]: " LAYA_MODEL; LAYA_MODEL="${LAYA_MODEL:-typed-decisions}"
read -rp "Timeout [5.0]: " TIMEOUT; TIMEOUT="${TIMEOUT:-5.0}"
read -rp "Politica broker [prioridad]: " POL; POL="${POL:-prioridad}"
read -rp "Indice de camara [0]: " CAM; CAM="${CAM:-0}"
mkdir -p registros
STAMP="$(date +%Y%m%d_%H%M%S)"
PIDS=()
cleanup(){ for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

ros2 run parcial_interprete interprete_ordenes --ros-args \
  -p laya_url:="http://${LAYA_IP}:${LAYA_PORT}" -p laya_model:="$LAYA_MODEL" -p timeout_s:="$TIMEOUT" \
  >"registros/p2_interprete_${STAMP}.log" 2>&1 & PIDS+=("$!")
ros2 run arm_broker broker --ros-args -p politica:="$POL" \
  >"registros/p2_broker_${STAMP}.log" 2>&1 & PIDS+=("$!")
ros2 run percepcion_item2 percepcion_camara --ros-args -p camera_index:="$CAM" \
  >"registros/p2_percepcion_${STAMP}.log" 2>&1 & PIDS+=("$!")

sleep 3
echo "Nodos/servicios visibles:"
ros2 node list || true
ros2 service list | grep -E 'interpretar|localizar' || true
ros2 action list || true

echo
echo "P2 base levantada. Para enviar una orden:"
echo "python3 herramientas/orden_desde_texto.py 'urgente recoge el cubo rojo'"
echo
echo "AVISO: el broker del repo actual conserva PIPELINE_PENDIENTE para la ejecucion semantica;"
echo "la calibracion XYZ, IK/MoveIt integrado y agarre fisico deben validarse/completarse con el JetCobot."
echo "Ctrl+C para cerrar."
wait
