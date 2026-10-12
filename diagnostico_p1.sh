#!/usr/bin/env bash
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash
[[ -f install/setup.bash ]] && source install/setup.bash

echo "=== ENTORNO ==="
echo "PWD=$PWD"
echo "ROS_DISTRO=${ROS_DISTRO:-}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-NO DEFINIDO}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-NO DEFINIDO}"
echo "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-NO DEFINIDO}"
echo
echo "=== PAQUETES ==="
ros2 pkg list | grep -E '^parcial_(interfaces|interprete)$' || true
echo
echo "=== EJECUTABLES ==="
ros2 pkg executables parcial_interprete 2>&1 || true
echo
echo "=== INTERFAZ ==="
ros2 interface show parcial_interfaces/srv/InterpretarOrden 2>&1 || true
echo
echo "=== NODOS ==="
ros2 node list 2>&1 || true
echo
echo "=== SERVICIO ==="
ros2 service list 2>&1 | grep interpretar || true
