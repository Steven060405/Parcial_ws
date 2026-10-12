#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash

if [[ ! -f install/setup.bash ]]; then
  colcon build --symlink-install --packages-up-to transcriptor_voz parcial_interprete
fi
source install/setup.bash

read -rp "ROS_DOMAIN_ID [62]: " ROS_ID; ROS_ID="${ROS_ID:-62}"
export ROS_DOMAIN_ID="$ROS_ID"; export ROS_LOCALHOST_ONLY=0
read -rp "ROS_DISCOVERY_SERVER (Enter = no usar): " DISCOVERY
if [[ -n "$DISCOVERY" ]]; then export ROS_DISCOVERY_SERVER="$DISCOVERY"; else unset ROS_DISCOVERY_SERVER || true; fi
read -rp "IP LAYA [127.0.0.1]: " LAYA_IP; LAYA_IP="${LAYA_IP:-127.0.0.1}"
read -rp "Puerto LAYA [8000]: " LAYA_PORT; LAYA_PORT="${LAYA_PORT:-8000}"
read -rp "Modelo LAYA [typed-decisions]: " LAYA_MODEL; LAYA_MODEL="${LAYA_MODEL:-typed-decisions}"
read -rp "Timeout LAYA [5.0]: " LAYA_TIMEOUT; LAYA_TIMEOUT="${LAYA_TIMEOUT:-5.0}"
read -rp "Archivo de audio (Enter = forzar modo texto): " AUDIO
read -rp "Modelo Google AI Studio [gemini-2.5-flash]: " GOOGLE_MODEL; GOOGLE_MODEL="${GOOGLE_MODEL:-gemini-2.5-flash}"
read -rp "Timeout transcripcion [12.0]: " GOOGLE_TIMEOUT; GOOGLE_TIMEOUT="${GOOGLE_TIMEOUT:-12.0}"
read -rp "Encolar en broker despues de interpretar? [n/S]: " ENCOLAR; ENCOLAR="${ENCOLAR:-n}"

if [[ -z "${GOOGLE_API_KEY:-}" ]]; then
  echo "[AVISO] GOOGLE_API_KEY no esta definida. Se activara modo texto si se intenta transcribir."
fi

mkdir -p registros
STAMP="$(date +%Y%m%d_%H%M%S)"
ros2 run parcial_interprete interprete_ordenes --ros-args \
  -p laya_url:="http://${LAYA_IP}:${LAYA_PORT}" -p laya_model:="$LAYA_MODEL" -p timeout_s:="$LAYA_TIMEOUT" \
  >"registros/p3_interprete_${STAMP}.log" 2>&1 &
P1PID=$!
cleanup(){ kill "$P1PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
sleep 2

ENQ=false
case "${ENCOLAR,,}" in s|si|sí|y|yes) ENQ=true;; esac
ros2 run transcriptor_voz transcriptor_voz --ros-args \
  -p audio_file:="$AUDIO" \
  -p google_model:="$GOOGLE_MODEL" \
  -p timeout_s:="$GOOGLE_TIMEOUT" \
  -p enqueue_broker:="$ENQ"
