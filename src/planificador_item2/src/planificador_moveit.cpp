#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <arm_broker_interfaces/srv/planificar_pose.hpp>


class PlanificadorMoveIt : public rclcpp::Node
{
public:
  PlanificadorMoveIt()
  : Node("planificador_moveit")
  {
    this->declare_parameter<std::string>(
      "planning_group",
      "arm_group"
    );

    this->declare_parameter<std::string>(
      "tip_link",
      "6_Link"
    );

    planning_group_ =
      this->get_parameter("planning_group").as_string();

    tip_link_ =
      this->get_parameter("tip_link").as_string();
  }


  void initialize()
  {
    move_group_ =
      std::make_shared<
        moveit::planning_interface::MoveGroupInterface
      >(
        shared_from_this(),
        planning_group_
      );

    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);

    servicio_ =
      this->create_service<
        arm_broker_interfaces::srv::PlanificarPose
      >(
        "/planificar_pose",
        std::bind(
          &PlanificadorMoveIt::planificar,
          this,
          std::placeholders::_1,
          std::placeholders::_2
        )
      );

    RCLCPP_INFO(
      this->get_logger(),
      "planificador_moveit listo · grupo=%s · tip=%s",
      planning_group_.c_str(),
      tip_link_.c_str()
    );

    RCLCPP_INFO(
      this->get_logger(),
      "MODO SEGURO: MoveIt2 planifica; NO ejecuta"
    );
  }


private:

  struct ObjetivoCalibrado
  {
    std::string nombre;

    // XYZ de referencia de la FK del RB-2 [mm].
    double x;
    double y;
    double z;

    // Pose fisica segura medida en el JetCobot [grados].
    std::array<double, 6> grados;
  };


  const std::vector<ObjetivoCalibrado> objetivos_ = {

    {
      "arriba_izquierda",
      259.91, 89.71, 145.78,
      {32.0, -60.0, -28.0, 0.0, 2.0, -26.0}
    },

    {
      "arriba_derecha",
      267.32, -63.40, 148.04,
      {0.0, -57.0, -35.0, 7.0, 0.0, -45.0}
    },

    {
      "abajo_izquierda",
      190.03, 87.62, 134.25,
      {38.0, -28.0, -83.0, 8.0, 18.0, -13.0}
    },

    {
      "abajo_derecha",
      198.87, -62.52, 122.85,
      {-1.0, -29.0, -86.0, 10.0, 5.0, -53.0}
    },

    {
      "zona_a",
      131.67, 229.16, 158.08,
      {74.0, -46.0, -55.0, 22.0, 0.0, -45.0}
    }
  };


  static double rad(double grados)
  {
    return grados * M_PI / 180.0;
  }


  static double deg(double radianes)
  {
    return radianes * 180.0 / M_PI;
  }


  const ObjetivoCalibrado * buscar_objetivo(
    double x,
    double y,
    double z
  )
  {
    const ObjetivoCalibrado * mejor = nullptr;

    double mejor_distancia = 1e9;

    for (const auto & objetivo : objetivos_)
    {
      const double dx = x - objetivo.x;
      const double dy = y - objetivo.y;
      const double dz = z - objetivo.z;

      const double distancia =
        std::sqrt(
          dx * dx +
          dy * dy +
          dz * dz
        );

      if (distancia < mejor_distancia)
      {
        mejor_distancia = distancia;
        mejor = &objetivo;
      }
    }

    // Las coordenadas deben corresponder
    // a uno de nuestros puntos conocidos.
    if (mejor_distancia > 20.0)
    {
      return nullptr;
    }

    return mejor;
  }


  void planificar(
    const std::shared_ptr<
      arm_broker_interfaces::srv::PlanificarPose::Request
    > request,

    std::shared_ptr<
      arm_broker_interfaces::srv::PlanificarPose::Response
    > response
  )
  {
    const double norma_q =
      std::sqrt(
        request->qx * request->qx +
        request->qy * request->qy +
        request->qz * request->qz +
        request->qw * request->qw
      );


    // ==========================================================
    // MODO CALIBRADO
    //
    // Quaternion = [0,0,0,0]
    //
    // El XYZ identifica uno de los puntos conocidos.
    // Su pose articular calibrada se usa como semilla de IK.
    // ==========================================================

    if (norma_q < 1e-9)
    {
      const auto * objetivo =
        buscar_objetivo(
          request->x_mm,
          request->y_mm,
          request->z_mm
        );

      if (objetivo == nullptr)
      {
        response->success = false;
        response->message =
          "XYZ no corresponde a un objetivo calibrado";

        response->trajectory.points.clear();
        return;
      }


      // ----------------------------------------------------------
      // Crear RobotState desde el modelo.
      //
      // NO usamos getCurrentState() dentro del callback del servicio,
      // porque puede bloquear el monitor local de /joint_states.
      // El estado REAL se usara despues como inicio de planificacion
      // mediante setStartStateToCurrentState().
      // ----------------------------------------------------------

      auto robot_model =
        move_group_->getRobotModel();

      if (!robot_model)
      {
        response->success = false;
        response->message =
          "MoveIt2 no pudo obtener RobotModel";

        response->trajectory.points.clear();
        return;
      }


      moveit::core::RobotState semilla(
        robot_model
      );

      semilla.setToDefaultValues();


      const auto * grupo =
        semilla.getJointModelGroup(
          planning_group_
        );

      if (grupo == nullptr)
      {
        response->success = false;
        response->message =
          "grupo arm_group no encontrado";

        response->trajectory.points.clear();
        return;
      }


      // ----------------------------------------------------------
      // 1. Crear SEMILLA con nuestra pose fisica calibrada.
      // ----------------------------------------------------------

      std::vector<double> q_semilla;

      for (double valor : objetivo->grados)
      {
        q_semilla.push_back(
          rad(valor)
        );
      }

      semilla.setJointGroupPositions(
        grupo,
        q_semilla
      );

      semilla.update();


      // ----------------------------------------------------------
      // 2. Obtener, mediante FK DEL MODELO MOVEIT,
      //    la pose cartesiana correspondiente a la semilla.
      //
      // Esto evita mezclar directamente el frame cartesiano
      // interno de pymycobot/RB2 con el frame del URDF.
      // ----------------------------------------------------------

      const Eigen::Isometry3d & transform =
        semilla.getGlobalLinkTransform(
          tip_link_
        );


      geometry_msgs::msg::Pose pose_objetivo;

      pose_objetivo.position.x =
        transform.translation().x();

      pose_objetivo.position.y =
        transform.translation().y();

      pose_objetivo.position.z =
        transform.translation().z();


      Eigen::Quaterniond quat(
        transform.rotation()
      );

      quat.normalize();

      pose_objetivo.orientation.x = quat.x();
      pose_objetivo.orientation.y = quat.y();
      pose_objetivo.orientation.z = quat.z();
      pose_objetivo.orientation.w = quat.w();


      // ----------------------------------------------------------
      // 3. RESOLVER IK con la configuracion calibrada como semilla.
      // ----------------------------------------------------------

      moveit::core::RobotState estado_ik(
        semilla
      );


      const bool ik_ok =
        estado_ik.setFromIK(
          grupo,
          pose_objetivo,
          tip_link_,
          1.0
        );


      if (!ik_ok)
      {
        response->success = false;
        response->message =
          "IK no encontro solucion cercana a la semilla calibrada";

        response->trajectory.points.clear();
        return;
      }


      std::vector<double> q_ik;

      estado_ik.copyJointGroupPositions(
        grupo,
        q_ik
      );


      // ----------------------------------------------------------
      // 4. Verificar que la IK NO salte a otra rama.
      // ----------------------------------------------------------

      double max_diff_deg = 0.0;

      for (std::size_t i = 0;
           i < q_ik.size() && i < q_semilla.size();
           ++i)
      {
        const double diferencia =
          std::abs(
            deg(q_ik[i] - q_semilla[i])
          );

        max_diff_deg =
          std::max(
            max_diff_deg,
            diferencia
          );
      }


      RCLCPP_INFO(
        this->get_logger(),
        "IK_CALIBRADA objetivo=%s max_diff=%.2f deg",
        objetivo->nombre.c_str(),
        max_diff_deg
      );


      // Seguridad.
      if (max_diff_deg > 20.0)
      {
        response->success = false;

        response->message =
          "IK rechazo: solucion demasiado alejada "
          "de pose calibrada";

        response->trajectory.points.clear();
        return;
      }


      // ----------------------------------------------------------
      // 5. MoveIt2 planifica desde estado REAL
      //    hacia la solucion obtenida por IK.
      // ----------------------------------------------------------

      move_group_->setStartStateToCurrentState();

      const bool target_ok =
        move_group_->setJointValueTarget(
          q_ik
        );


      if (!target_ok)
      {
        response->success = false;

        response->message =
          "MoveIt2 rechazo objetivo articular de IK";

        response->trajectory.points.clear();
        return;
      }


      moveit::planning_interface::
        MoveGroupInterface::Plan plan;


      const auto codigo =
        move_group_->plan(plan);


      if (
        codigo !=
        moveit::core::MoveItErrorCode::SUCCESS
      )
      {
        response->success = false;

        response->message =
          "MoveIt2 no encontro trayectoria "
          "hacia IK calibrada";

        response->trajectory.points.clear();
        return;
      }


      response->trajectory =
        plan.trajectory_.joint_trajectory;


      if (response->trajectory.points.empty())
      {
        response->success = false;

        response->message =
          "MoveIt2 devolvio trayectoria vacia";

        return;
      }


      response->success = true;

      response->message =
        "IK calibrada + trayectoria MoveIt2; "
        "no ejecutada";


      RCLCPP_INFO(
        this->get_logger(),
        "PLAN_OK objetivo=%s puntos=%zu max_diff=%.2fdeg",
        objetivo->nombre.c_str(),
        response->trajectory.points.size(),
        max_diff_deg
      );

      return;
    }


    // ==========================================================
    // MODO POSE NORMAL
    // Se conserva para pruebas cartesianas con quaternion real.
    // ==========================================================

    geometry_msgs::msg::Pose pose;

    pose.position.x =
      request->x_mm / 1000.0;

    pose.position.y =
      request->y_mm / 1000.0;

    pose.position.z =
      request->z_mm / 1000.0;


    pose.orientation.x =
      request->qx / norma_q;

    pose.orientation.y =
      request->qy / norma_q;

    pose.orientation.z =
      request->qz / norma_q;

    pose.orientation.w =
      request->qw / norma_q;


    move_group_->setStartStateToCurrentState();

    move_group_->setPoseTarget(
      pose,
      tip_link_
    );


    moveit::planning_interface::
      MoveGroupInterface::Plan plan;


    const auto codigo =
      move_group_->plan(plan);


    move_group_->clearPoseTargets();


    if (
      codigo !=
      moveit::core::MoveItErrorCode::SUCCESS
    )
    {
      response->success = false;

      response->message =
        "MoveIt2 no encontro trayectoria";

      response->trajectory.points.clear();
      return;
    }


    response->trajectory =
      plan.trajectory_.joint_trajectory;


    if (response->trajectory.points.empty())
    {
      response->success = false;

      response->message =
        "trayectoria vacia";

      return;
    }


    response->success = true;

    response->message =
      "trayectoria MoveIt2; no ejecutada";
  }


  std::string planning_group_;
  std::string tip_link_;

  std::shared_ptr<
    moveit::planning_interface::
      MoveGroupInterface
  > move_group_;

  rclcpp::Service<
    arm_broker_interfaces::srv::PlanificarPose
  >::SharedPtr servicio_;
};


int main(
  int argc,
  char ** argv
)
{
  rclcpp::init(argc, argv);

  auto nodo =
    std::make_shared<
      PlanificadorMoveIt
    >();

  nodo->initialize();

  rclcpp::executors::
    MultiThreadedExecutor executor;

  executor.add_node(nodo);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
