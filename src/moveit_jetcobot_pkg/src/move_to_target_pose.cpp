#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class SetTargetPosition : public rclcpp::Node
{
public:
  SetTargetPosition() : Node("set_target_position")
  {
    // Initialize other components
    RCLCPP_INFO(this->get_logger(), "Initializing TargetMoveIt2Control.");
  }

  void initialize()
  {
    // Initialize move_group_interface_ and planning_scene_interface_ in this function
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    move_group_interface_->setNumPlanningAttempts(10);  // Set maximum planning attempts to 10
    move_group_interface_->setPlanningTime(5.0);         // Set maximum planning time per attempt to 5 seconds
    
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface and PlanningSceneInterface initialized.");
  }

  bool moveToTargetPose(const geometry_msgs::msg::Pose &target_pose)
  {
    move_group_interface_->setPoseTarget(target_pose);

    bool success = (move_group_interface_->plan(my_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Planning successful, executing the plan.");
      move_group_interface_->execute(my_plan_);

      geometry_msgs::msg::PoseStamped current_pose = move_group_interface_->getPoseTarget();
      RCLCPP_INFO(this->get_logger(), "Executed! Reached to pose: position.x=%f, position.y=%f, position.z=%f, orientation.x=%f, orientation.y=%f, orientation.z=%f, orientation.w=%f" ,
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z,
                    current_pose.pose.orientation.x,
                    current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z,
                    current_pose.pose.orientation.w);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Planning failed.");
    }

    return success;
  }

  bool moveToTargetAngles(const std::vector<double> &target_joints)
  {
    move_group_interface_->setJointValueTarget(target_joints);

    bool success = (move_group_interface_->plan(my_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Planning successful, executing the plan.");
      move_group_interface_->execute(my_plan_);

      RCLCPP_INFO(this->get_logger(), "Executed! Reached to joint angles: 1_Joint=%f, 2_Joint=%f, 3_Joint=%f, 4_Joint=%f, 5_Joint=%f, 6_Joint=%f",
                    target_joints[0],
                    target_joints[1],
                    target_joints[2],
                    target_joints[3],
                    target_joints[4],
                    target_joints[5]);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Planning failed.");
    }

    return success;
  }

  void runTest()
  {
    // // set named target pose
    // move_group_interface_->setNamedTarget("up");
    // geometry_msgs::msg::PoseStamped named_pose = move_group_interface_->getPoseTarget();
    
    // // Move to named joint-space target
    // moveToTargetPose(named_pose.pose);
    
    std::vector<double> target_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Target joint angles (radians)
    moveToTargetAngles(target_joints);

    geometry_msgs::msg::Pose target_pose;
    
    target_pose.position.x = 0.13695764541625977;
    target_pose.position.y = -0.005238924641162157;
    target_pose.position.z = 0.3486359715461731;
    target_pose.orientation.x = 3.1805826438358054e-05;
    target_pose.orientation.y = -4.311262455303222e-05;
    target_pose.orientation.z = -3.6135115806246176e-05;
    target_pose.orientation.w = 1;

    // Plan and Execute target pose
    moveToTargetPose(target_pose);
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  // Plan path
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SetTargetPosition>();
  
  // Delayed initialization
  node->initialize();
  node->runTest();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
