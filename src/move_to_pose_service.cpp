#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "move_group_ros2/srv/move_to_pose.hpp"

#include <visualization_msgs/msg/marker.hpp>

class PosePlanner : public rclcpp::Node
{
public:
  PosePlanner()
    : Node("move_to_pose_service")
  {
    this->declare_parameter<std::string>("planning_group", "arm");

    std::string group_name;
    this->get_parameter("planning_group", group_name);

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), group_name);

    service_ = this->create_service<move_group_ros2::srv::MoveToPose>(
      "move_to_pose",
      std::bind(&PosePlanner::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("debug_goal_marker", 1);
    RCLCPP_INFO(this->get_logger(), "MoveToPose service ready (group: %s)", group_name.c_str());
  }

private:
  rclcpp::Service<move_group_ros2::srv::MoveToPose>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  void handle_request(
    const std::shared_ptr<move_group_ros2::srv::MoveToPose::Request> req,
    std::shared_ptr<move_group_ros2::srv::MoveToPose::Response> res)
  {
    try
    {
      const std::string planning_frame = move_group_->getPlanningFrame();
      move_group_->setPoseReferenceFrame(planning_frame);

      // (Optional but helpful)
      move_group_->setStartStateToCurrentState();

      // Log the raw pose numbers
      const auto& p = req->target_pose.pose.position;
      RCLCPP_INFO(get_logger(), "Target (interpreted in %s) x=%.3f y=%.3f z=%.3f",
                  planning_frame.c_str(), p.x, p.y, p.z);

      // Publish the marker (bigger scale so it’s obvious)
      visualization_msgs::msg::Marker m;
      m.header.stamp = now();
      m.header.frame_id = planning_frame;
      m.ns = "move_to_pose_goal";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = req->target_pose.pose;
      m.scale.x = m.scale.y = m.scale.z = 0.03;
      m.color.a = 1.0; m.color.r = 0.1; m.color.g = 0.8; m.color.b = 0.2;
      marker_pub_->publish(m);

      move_group_->setPoseTarget(req->target_pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      move_group_->setPlanningTime(3.0);
      move_group_->setGoalTolerance(1e-3);
      move_group_->setGoalOrientationTolerance(0.15); // temporarily relaxed
      auto planning_result = move_group_->plan(plan);
  
      if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_WARN(this->get_logger(), "Planning failed.");
        res->success = false;
        return;
      }
  
      RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing...");
  
      moveit::core::MoveItErrorCode exec_result = move_group_->execute(plan);
  
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Motion to pose succeeded.");
        res->success = true;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Motion execution failed.");
        res->success = false;
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception during planning or execution: %s", e.what());
      res->success = false;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosePlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}