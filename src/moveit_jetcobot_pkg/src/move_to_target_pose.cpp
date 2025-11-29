#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>

class SetTargetPosition : public rclcpp::Node
{
public:
  int max_tries = 2;

  struct quaternion
  {
      double x;
      double y;
      double z;
      double w;
  };

  SetTargetPosition() : Node("set_target_position")
  {
    // Initialize other components
    RCLCPP_INFO(this->get_logger(), "Initializing TargetMoveIt2Control.");
    // initialize a subscriber:
    target_pos_q_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "target_pose_q", 10, std::bind(&SetTargetPosition::moveToTargetPoseQ, this, std::placeholders::_1)
    );

    target_pos_xyz_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "target_pose_xyz", 10, std::bind(&SetTargetPosition::moveToTargetPoseXYZ, this, std::placeholders::_1)
    );

    target_car_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "target_cartesian", 10, std::bind(&SetTargetPosition::moveToTargetCartesian, this, std::placeholders::_1)
    );

    target_ang_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "target_angles", 10, std::bind(&SetTargetPosition::moveToTargetAngles, this, std::placeholders::_1)
    );

    status_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "/arm_execution_status", 10
    );
  }

  void initialize()
  {
    // Initialize move_group_interface_ and planning_scene_interface_ in this function
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    move_group_interface_->setNumPlanningAttempts(5);  // Set maximum planning attempts to 10
    move_group_interface_->setPlanningTime(5);         // Set maximum planning time per attempt to 10 seconds
    move_group_interface_->allowReplanning(true);
    
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface and PlanningSceneInterface initialized.");
  }

  bool moveToTargetPoseQ(const std_msgs::msg::Float64MultiArray &target_6dpose)
  {
    geometry_msgs::msg::Pose target_pose;

    target_pose.position.x = target_6dpose.data[0];
    target_pose.position.y = target_6dpose.data[1];
    target_pose.position.z = target_6dpose.data[2];
    target_pose.orientation.x = target_6dpose.data[3];
    target_pose.orientation.y = target_6dpose.data[4];
    target_pose.orientation.z = target_6dpose.data[5];
    target_pose.orientation.w = target_6dpose.data[6];

    move_group_interface_->setGoalPositionTolerance(0.005);
    move_group_interface_->setGoalOrientationTolerance(0.01);

    move_group_interface_->setPoseTarget(target_pose);
    
    bool success = false;
    
    int planning_tries = 0;
      
    while(planning_tries <= max_tries && !success)
    {
      success = (move_group_interface_->plan(my_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Planning successful, executing the plan.");
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        moveit::core::MoveItErrorCode ecode = moveit::core::MoveItErrorCode::FAILURE;
        
        int execution_tries = 0;
      
        while(execution_tries <= max_tries && ecode != moveit::core::MoveItErrorCode::SUCCESS)
        {
           ecode = move_group_interface_->execute(my_plan_);

           if(ecode == moveit::core::MoveItErrorCode::SUCCESS)
           {
              RCLCPP_INFO(this->get_logger(), "Execution sucessful!");
              geometry_msgs::msg::PoseStamped current_pose = move_group_interface_->getPoseTarget();
              RCLCPP_INFO(this->get_logger(), "Executed! Reached to pose: position.x=%f, position.y=%f, position.z=%f, orientation.x=%f, orientation.y=%f, orientation.z=%f, orientation.w=%f" ,
		     current_pose.pose.position.x,
		     current_pose.pose.position.y,
		     current_pose.pose.position.z,
		     current_pose.pose.orientation.x,
		     current_pose.pose.orientation.y,
		     current_pose.pose.orientation.z,
		     current_pose.pose.orientation.w);
              break;
           }
           else
           {
              RCLCPP_WARN(this->get_logger(), "Execution failed.");
              RCLCPP_WARN(this->get_logger(), "Attempted %d...", execution_tries + 1);
              execution_tries++;
           }
        }
        
        break;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Planning failed.");
        RCLCPP_WARN(this->get_logger(), "Attempted %d...", planning_tries + 1);
        planning_tries++;
      }
    }

    publish_status(success);
    return success;
  }

  bool moveToTargetPoseXYZ(const std_msgs::msg::Float64MultiArray &target_6dpose)
  {
    geometry_msgs::msg::Pose target_pose;

    // convert euler angles (roll, pitch, yaw) in zyx convention to quaternion
    quaternion quat = eulerToQuaternion(target_6dpose.data[3], target_6dpose.data[4], target_6dpose.data[5]);
    RCLCPP_INFO(this->get_logger(), "Converted Euler angles to Quaternion: x=%f, y=%f, z=%f, w=%f",
                 quat.x, quat.y, quat.z, quat.w);

    target_pose.position.x = target_6dpose.data[0];
    target_pose.position.y = target_6dpose.data[1];
    target_pose.position.z = target_6dpose.data[2];
    target_pose.orientation.x = quat.x;
    target_pose.orientation.y = quat.y;
    target_pose.orientation.z = quat.z;
    target_pose.orientation.w = quat.w;

    move_group_interface_->setGoalPositionTolerance(0.005);
    move_group_interface_->setGoalOrientationTolerance(0.01);

    move_group_interface_->setPoseTarget(target_pose);
    
    bool success = false;
    
    int planning_tries = 0;
      
    while(planning_tries <= max_tries && !success)
    {
      success = (move_group_interface_->plan(my_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Planning successful, executing the plan.");
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        moveit::core::MoveItErrorCode ecode = moveit::core::MoveItErrorCode::FAILURE;
        
        int execution_tries = 0;
      
        while(execution_tries <= max_tries && ecode != moveit::core::MoveItErrorCode::SUCCESS)
        {
           // how to print the whole trajectory for debugging in ros info?
           RCLCPP_INFO(this->get_logger(), "Planned trajectory has %ld points.", my_plan_.trajectory_.joint_trajectory.points.size());
           for (size_t i = 0; i < my_plan_.trajectory_.joint_trajectory.points.size(); ++i)
           {
               RCLCPP_INFO(this->get_logger(), "Point %ld: positions =", i);
               for (size_t j = 0; j < my_plan_.trajectory_.joint_trajectory.points[i].positions.size(); ++j)
               {
                   RCLCPP_INFO(this->get_logger(), "  Joint %ld: %f", j, my_plan_.trajectory_.joint_trajectory.points[i].positions[j]);
               }
           }

           ecode = move_group_interface_->execute(my_plan_);

           if(ecode == moveit::core::MoveItErrorCode::SUCCESS)
           {
              RCLCPP_INFO(this->get_logger(), "Execution sucessful!");
              geometry_msgs::msg::PoseStamped current_pose = move_group_interface_->getPoseTarget();
              RCLCPP_INFO(this->get_logger(), "Executed! Reached to pose: position.x=%f, position.y=%f, position.z=%f, orientation.x=%f, orientation.y=%f, orientation.z=%f, orientation.w=%f" ,
		     current_pose.pose.position.x,
		     current_pose.pose.position.y,
		     current_pose.pose.position.z,
		     current_pose.pose.orientation.x,
		     current_pose.pose.orientation.y,
		     current_pose.pose.orientation.z,
		     current_pose.pose.orientation.w);
              break;
           }
           else
           {
              RCLCPP_WARN(this->get_logger(), "Execution failed.");
              RCLCPP_WARN(this->get_logger(), "Attempted %d...", execution_tries + 1);
              execution_tries++;
           }
        }
        
        break;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Planning failed.");
        RCLCPP_WARN(this->get_logger(), "Attempted %d...", planning_tries + 1);
        planning_tries++;
      }
    }

    publish_status(success);
    return success;
  }

  bool moveToTargetCartesian(const std_msgs::msg::Float64MultiArray &target_car)
  {
    move_group_interface_->setGoalPositionTolerance(0.01);

    move_group_interface_->setPositionTarget(target_car.data[0], target_car.data[1], target_car.data[2]);
    
    bool success = false;
    
    int planning_tries = 0;
      
    while(planning_tries <= max_tries && !success)
    {
      success = (move_group_interface_->plan(my_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Planning successful, executing the plan.");
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        moveit::core::MoveItErrorCode ecode = moveit::core::MoveItErrorCode::FAILURE;
        
        int execution_tries = 0;
      
        while(execution_tries <= max_tries && ecode != moveit::core::MoveItErrorCode::SUCCESS)
        {
           ecode = move_group_interface_->execute(my_plan_);

           if(ecode == moveit::core::MoveItErrorCode::SUCCESS)
           {
              RCLCPP_INFO(this->get_logger(), "Execution sucessful!");
              geometry_msgs::msg::PoseStamped current_pose = move_group_interface_->getPoseTarget();
              RCLCPP_INFO(this->get_logger(), "Executed! Reached to pose: position.x=%f, position.y=%f, position.z=%f, orientation.x=%f, orientation.y=%f, orientation.z=%f, orientation.w=%f" ,
		     current_pose.pose.position.x,
		     current_pose.pose.position.y,
		     current_pose.pose.position.z,
		     current_pose.pose.orientation.x,
		     current_pose.pose.orientation.y,
		     current_pose.pose.orientation.z,
		     current_pose.pose.orientation.w);
              break;
           }
           else
           {
              RCLCPP_WARN(this->get_logger(), "Execution failed.");
              RCLCPP_WARN(this->get_logger(), "Attempted %d...", execution_tries + 1);
              execution_tries++;
           }
        }
        
        break;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Planning failed.");
        RCLCPP_WARN(this->get_logger(), "Attempted %d...", planning_tries + 1);
        planning_tries++;
      }
    }

    publish_status(success);
    return success;
  }

  bool moveToTargetAngles(const std_msgs::msg::Float64MultiArray &target_angles)
  {
    std::vector<double> target_joints = target_angles.data;

    move_group_interface_->setJointValueTarget(target_joints);

    bool success = false;
    
    int planning_tries = 0;
      
    while(planning_tries <= max_tries && !success)
    {
      success = (move_group_interface_->plan(my_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Planning successful, executing the plan.");
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        
        moveit::core::MoveItErrorCode ecode = moveit::core::MoveItErrorCode::FAILURE;
        
        int execution_tries = 0;
      
        while(execution_tries <= max_tries && ecode != moveit::core::MoveItErrorCode::SUCCESS)
        {
           ecode = move_group_interface_->execute(my_plan_);

           if(ecode == moveit::core::MoveItErrorCode::SUCCESS)
           {
              RCLCPP_INFO(this->get_logger(), "Executed! Reached to joint angles: 1_Joint=%f, 2_Joint=%f, 3_Joint=%f, 4_Joint=%f, 5_Joint=%f, 6_Joint=%f",
                        target_joints[0],
                        target_joints[1],
                        target_joints[2],
                        target_joints[3],
                        target_joints[4],
                        target_joints[5]);
              break;
           }
           else
           {
              RCLCPP_WARN(this->get_logger(), "Execution failed.");
              RCLCPP_WARN(this->get_logger(), "Attempted %d...", execution_tries + 1);
              execution_tries++;
           }
        }
        
        break;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Planning failed.");
        RCLCPP_WARN(this->get_logger(), "Attempted %d...", planning_tries + 1);
        planning_tries++;
      }
    }

    publish_status(success);
    return success;
  }

  quaternion eulerToQuaternion(double roll, double pitch, double yaw)
  {
      quaternion quat;

      double cy = std::cos(yaw * 0.5);
      double sy = std::sin(yaw * 0.5);
      double cr = std::cos(roll * 0.5);
      double sr = std::sin(roll * 0.5);
      double cp = std::cos(pitch * 0.5);
      double sp = std::sin(pitch * 0.5);

      quat.w = cy * cr * cp + sy * sr * sp;
      quat.x = cy * sr * cp - sy * cr * sp;
      quat.y = cy * cr * sp + sy * sr * cp;
      quat.z = sy * cr * cp - cy * sr * sp;

      return quat;
  }

  void publish_status(bool status)
  {
      auto message = std_msgs::msg::Bool();
      message.data = status;
      status_publisher_->publish(message);
      
      RCLCPP_INFO(this->get_logger(), "Published arm status: %s", 
                  status ? "TRUE (completed)" : "FALSE (in progress)");
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pos_q_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pos_xyz_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_car_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_ang_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_publisher_;

  // Plan path
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SetTargetPosition>();
  
  // Delayed initialization
  node->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
