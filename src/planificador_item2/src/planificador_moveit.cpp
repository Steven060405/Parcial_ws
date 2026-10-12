#include <cmath>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <arm_broker_interfaces/srv/planificar_pose.hpp>

class PlanificadorMoveIt : public rclcpp::Node
{
public:
  PlanificadorMoveIt() : Node("planificador_moveit")
  {
    this->declare_parameter<std::string>("planning_group", "arm_group");
    planning_group_ = this->get_parameter("planning_group").as_string();
  }

  void initialize()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);
    servicio_ = this->create_service<arm_broker_interfaces::srv::PlanificarPose>(
      "/planificar_pose",
      std::bind(&PlanificadorMoveIt::planificar, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "planificador_moveit listo · grupo=%s", planning_group_.c_str());
    RCLCPP_INFO(this->get_logger(), "MODO SEGURO: solo planifica; NO ejecuta trayectorias");
  }

private:
  void planificar(
      const std::shared_ptr<arm_broker_interfaces::srv::PlanificarPose::Request> request,
      std::shared_ptr<arm_broker_interfaces::srv::PlanificarPose::Response> response)
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = request->x_mm / 1000.0;
    target_pose.position.y = request->y_mm / 1000.0;
    target_pose.position.z = request->z_mm / 1000.0;
    double qx=request->qx, qy=request->qy, qz=request->qz, qw=request->qw;
    const double norma = std::sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    if (norma < 1e-9) { qx=0.0; qy=0.0; qz=0.0; qw=1.0; }
    else { qx/=norma; qy/=norma; qz/=norma; qw/=norma; }
    target_pose.orientation.x=qx; target_pose.orientation.y=qy; target_pose.orientation.z=qz; target_pose.orientation.w=qw;
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto codigo = move_group_->plan(plan);
    move_group_->clearPoseTargets();
    if (codigo != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success=false; response->message="MoveIt2 no encontro una trayectoria"; response->trajectory.points.clear(); return;
    }
    response->trajectory = plan.trajectory_.joint_trajectory;
    if (response->trajectory.points.empty()) {
      response->success=false; response->message="MoveIt2 devolvio trayectoria vacia"; return;
    }
    response->success=true;
    response->message="trayectoria planificada; no ejecutada";
  }

  std::string planning_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Service<arm_broker_interfaces::srv::PlanificarPose>::SharedPtr servicio_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node=std::make_shared<PlanificadorMoveIt>();
  node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
