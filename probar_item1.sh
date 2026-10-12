#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash
FRASE="${*:-urgente recoge el cubo rojo}"
ros2 service call /interpretar_orden parcial_interfaces/srv/InterpretarOrden "{frase: '$FRASE'}"
