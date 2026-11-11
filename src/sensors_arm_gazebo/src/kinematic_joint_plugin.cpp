// Kinematic puppet: force ALL non-fixed joints each step; links are kinematic + no collide.

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace sensors_arm_gazebo
{
namespace
{
std::string ShortJointName(std::string const & name)
{
  auto pos = name.rfind("::");
  if (pos == std::string::npos) {
    return name;
  }
  return name.substr(pos + 2);
}
}  // namespace

class KinematicJointPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    std::string topic = "joint_states";
    if (sdf->HasElement("joint_states_topic")) {
      topic = sdf->Get<std::string>("joint_states_topic");
    }

    for (auto const & joint : model_->GetJoints()) {
      if (!joint) {
        continue;
      }
      if (joint->GetType() == gazebo::physics::Base::FIXED_JOINT) {
        continue;
      }
      auto const short_name = ShortJointName(joint->GetName());
      joints_.emplace_back(short_name, joint);
      hold_[short_name] = joint->Position(0);
    }

    // Make the whole chain kinematic and non-colliding so ODE cannot jiggle EE.
    for (auto const & link : model_->GetLinks()) {
      if (!link) {
        continue;
      }
      link->SetKinematic(true);
      link->SetCollideMode("none");
      link->SetGravityMode(false);
    }

    sub_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>(
      topic, rclcpp::QoS(10),
      std::bind(&KinematicJointPlugin::OnJointState, this, std::placeholders::_1));

    update_ = gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&KinematicJointPlugin::OnUpdate, this));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "kinematic_joint_plugin model=%s actuated_joints=%zu topic=%s (kinematic links)",
      model_->GetName().c_str(), joints_.size(), topic.c_str());
  }

private:
  void OnJointState(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->name.size() != msg->position.size()) {
      return;
    }
    // Prefer full arm-c messages (with gripper). Ignore short hw-only bursts.
    bool has_gripper = false;
    for (auto const & n : msg->name) {
      if (ShortJointName(n) == "gripper_opening") {
        has_gripper = true;
        break;
      }
    }
    if (!has_gripper && hold_.count("gripper_opening") > 0 && have_cmd_) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      hold_[ShortJointName(msg->name[i])] = msg->position[i];
    }
    auto it = hold_.find("gripper_opening");
    if (it != hold_.end()) {
      hold_["gripper_finger_r"] = it->second;
    }
    have_cmd_ = true;
  }

  void OnUpdate()
  {
    std::map<std::string, double> hold;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_cmd_) {
        for (auto const & j : joints_) {
          if (j.second) {
            hold_[j.first] = j.second->Position(0);
          }
        }
        have_cmd_ = true;
      }
      hold = hold_;
    }

    for (auto const & j : joints_) {
      if (!j.second) {
        continue;
      }
      double q = j.second->Position(0);
      auto it = hold.find(j.first);
      if (it != hold.end()) {
        q = it->second;
      }
      j.second->SetPosition(0, q, false);
      j.second->SetVelocity(0, 0.0);
    }

    for (auto const & link : model_->GetLinks()) {
      if (!link) {
        continue;
      }
      link->SetLinearVel(ignition::math::Vector3d::Zero);
      link->SetAngularVel(ignition::math::Vector3d::Zero);
      link->SetForce(ignition::math::Vector3d::Zero);
      link->SetTorque(ignition::math::Vector3d::Zero);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  gazebo::event::ConnectionPtr update_;
  std::vector<std::pair<std::string, gazebo::physics::JointPtr>> joints_;
  std::mutex mutex_;
  std::map<std::string, double> hold_;
  bool have_cmd_{false};
};

GZ_REGISTER_MODEL_PLUGIN(KinematicJointPlugin)

}  // namespace sensors_arm_gazebo
