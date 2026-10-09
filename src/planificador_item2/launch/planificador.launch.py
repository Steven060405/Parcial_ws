import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ----------------------------------------------------------
    # Cargar kinematics.yaml directamente.
    #
    # En ROS 2 Humble de este Jetson no usamos
    # moveit_configs_utils.launch_utils.load_yaml.
    # ----------------------------------------------------------

    moveit_share = get_package_share_directory(
        "jetcobot_moveit"
    )

    kinematics_path = os.path.join(
        moveit_share,
        "config",
        "kinematics.yaml"
    )

    if not os.path.isfile(kinematics_path):
        raise RuntimeError(
            "No existe kinematics.yaml: "
            + kinematics_path
        )

    with open(
        kinematics_path,
        "r",
        encoding="utf-8"
    ) as archivo:

        kinematics_yaml = yaml.safe_load(
            archivo
        )

    if not isinstance(kinematics_yaml, dict):
        raise RuntimeError(
            "kinematics.yaml no contiene un diccionario valido"
        )

    if "arm_group" not in kinematics_yaml:
        raise RuntimeError(
            "kinematics.yaml no contiene arm_group"
        )

    return LaunchDescription([

        Node(
            package="planificador_item2",
            executable="planificador_moveit",
            name="planificador_moveit",
            output="screen",

            parameters=[

                {
                    "planning_group": "arm_group",
                    "tip_link": "6_Link",
                },

                {
                    "robot_description_kinematics":
                        kinematics_yaml
                },

            ],
        )

    ])
