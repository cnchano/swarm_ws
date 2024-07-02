// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <drone_plugin/gazebo_ros_drone_force.hpp>

#include <gazebo_ros/node.hpp>
#include <drone_common/msg/pose.hpp>
#include <drone_common/msg/motor_speed.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <memory>
#include <mutex>
#include <thread>

namespace gazebo_plugins
{
  ignition::math::Pose3d _pose;
  std::mutex cmd_mtx;
  std::mutex pose_mtx;
  drone_common::msg::MotorSpeed motor_speed_msg;
  
/// Class to hold private data members (PIMPL pattern)
class GazeboRosDroneForcePrivate
{
public:
  /// Pointer to model.
  gazebo::physics::ModelPtr model_;
  double _rotor_thrust_coeff;
  double _rotor_torque_coeff;
  bool _publish_tf;

  /// Node for ROS communication.
  // gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Node::SharedPtr ros_node_; //Assign sub-node class to variable

  /// Pose publisher
  rclcpp::Publisher<drone_common::msg::Pose>::SharedPtr pose_pub_;

  /// Command subscriber
  rclcpp::Subscription<drone_common::msg::MotorSpeed>::SharedPtr cmd_sub_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  void OnUpdate();
  void onMotorSpeedsMsg(const drone_common::msg::MotorSpeed::SharedPtr _msg);

  void UpdateThrust();

  double CalculateThrust(double w);
  double CalculateTorque(double w);
  double CalculateBodyTorque(double q);
};

GazeboRosDroneForce::GazeboRosDroneForce()
: impl_(std::make_unique<GazeboRosDroneForcePrivate>())
{
  std::cout << "Starting drone_plugin" << std::endl;
  
}

GazeboRosDroneForce::~GazeboRosDroneForce()
{
  std::cout << "Closing  drone_plugin" << std::endl;
}

void GazeboRosDroneForce::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::cout << "Model Name:" << _model->GetName().c_str() << std::endl; //Ouput model name to check

  std::string originalString = _model->GetName().c_str();
  _ns = "/" + originalString + "/";

  if (_sdf->HasElement("updateRate")) {
      _rate = _sdf->GetElement("updateRate")->Get<double>();
    } else {
      _rate = 100.0;
    }
  if (_sdf->HasElement("rotorThrustCoeff")) {
      _rotor_thrust_coeff = _sdf->GetElement("rotorThrustCoeff")->Get<double>();
    } else {
      _rotor_thrust_coeff = 0.01037;
    }
  impl_->_rotor_thrust_coeff = _rotor_thrust_coeff;
  if (_sdf->HasElement("rotorTorqueCoeff")) {
      _rotor_torque_coeff = _sdf->GetElement("rotorTorqueCoeff")->Get<double>();
    } else {
      _rotor_torque_coeff = 0.0000074;
    }
  impl_->_rotor_torque_coeff = _rotor_torque_coeff;
  if (_sdf->HasElement("publishTf")) {
      _publish_tf = _sdf->GetElement("publishTf")->Get<bool>();
    } else {
      _publish_tf = true;
    }
  impl_->_publish_tf = _publish_tf;


  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->ros_node_ = rclcpp::Node::make_shared(originalString + "drone_force"); //Create sub-node //added namespace to node name
  // Get QoS profiles
  // const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();
  impl_->model_ = _model;
  
  // The model pointer gives you direct access to the physics object,
  // for example:
  RCLCPP_INFO(impl_->ros_node_->get_logger(), _model->GetName().c_str());

  impl_->pose_pub_ = impl_->ros_node_->create_publisher<drone_common::msg::Pose>(
      _ns + "pose", 10
      );

  impl_->cmd_sub_ = impl_->ros_node_->create_subscription<drone_common::msg::MotorSpeed>(
      _ns + "motor_speed_cmd", 
      10,
      std::bind(&GazeboRosDroneForcePrivate::onMotorSpeedsMsg, impl_.get(), std::placeholders::_1)
      );
  ros_thread = std::thread(std::bind(&GazeboRosDroneForce::RosThread, this));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosDroneForcePrivate::OnUpdate, impl_.get())
      );
}

void GazeboRosDroneForcePrivate::OnUpdate()
{
  pose_mtx.lock();
  _pose = model_->WorldPose();
  pose_mtx.unlock();

  GazeboRosDroneForcePrivate::UpdateThrust();
}

void GazeboRosDroneForcePrivate::onMotorSpeedsMsg(const drone_common::msg::MotorSpeed::SharedPtr _msg) {
  cmd_mtx.lock();
  motor_speed_msg = *_msg;
  cmd_mtx.unlock();

}

void GazeboRosDroneForce::RosThread() {
    rclcpp::Rate rate(_rate);
    while (rclcpp::ok()) {
      GazeboRosDroneForce::PublishDronePose();
      rclcpp::spin_some(impl_->ros_node_);
      rate.sleep();
    }
}

void GazeboRosDroneForce::PublishDronePose() {
    pose_mtx.lock();
    ignition::math::Pose3d pose = _pose;
    pose_mtx.unlock();
    
    // ignition::math::Vector3 rpy = pose.Rot.GetAsEuler();
    tf2::Quaternion q(pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw(), pose.Rot().W());
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    drone_common::msg::Pose pose_msg;
    pose_msg.x = pose.Pos().X();
    pose_msg.y = pose.Pos().Y();
    pose_msg.z = pose.Pos().Z();
    pose_msg.roll = roll;
    pose_msg.pitch = -pitch;
    pose_msg.yaw = yaw;
    impl_->pose_pub_->publish(pose_msg);
    
    if (_publish_tf) {
      tf2::Transform T;
      T.setOrigin(tf2::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()));
      T.setRotation(tf2::Quaternion(q));

      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = impl_->ros_node_->now();
      transformStamped.header.frame_id = "world";
      transformStamped.child_frame_id = "drone";
      transformStamped.transform.translation.x = T.getOrigin().x();
      transformStamped.transform.translation.y = T.getOrigin().y();
      transformStamped.transform.translation.z = T.getOrigin().z();
      transformStamped.transform.rotation.x = T.getRotation().x();
      transformStamped.transform.rotation.y = T.getRotation().y();
      transformStamped.transform.rotation.z = T.getRotation().z();
      transformStamped.transform.rotation.w = T.getRotation().w();
      // tf_broadcaster_->sendTransform(transformStamped);
    }
  }

  void GazeboRosDroneForcePrivate::UpdateThrust() { 
    cmd_mtx.lock();
    drone_common::msg::MotorSpeed cmd = motor_speed_msg;
    cmd_mtx.unlock();
    
    int n = cmd.name.size();
    for (int i = 0; i < n; ++i) {
      double thrust = GazeboRosDroneForcePrivate::CalculateThrust(cmd.velocity[i]);
      double torque = GazeboRosDroneForcePrivate::CalculateTorque(cmd.velocity[i]);
      //ROS_INFO("torque: %f", torque);
      gazebo::physics::LinkPtr link_ = model_->GetLink(cmd.name[i]); //get child link instead of GetLink()
      if (link_ != NULL) {
        link_->AddLinkForce(ignition::math::Vector3d(0, 0, thrust)); //previously AddLinkForce() but having issues running multiple drones
        link_->AddRelativeTorque(ignition::math::Vector3d(0, 0, torque));
      }
    }
    
    int m = cmd.torque_body_name.size();
    for (int i = 0; i < m; ++i) {
      double body_moment = GazeboRosDroneForcePrivate::CalculateBodyTorque(cmd.body_torque[i]);
      //ROS_INFO("torque: %f", torque);
      gazebo::physics::LinkPtr link_ = model_->GetLink(cmd.torque_body_name[i]);
      if (link_ != NULL) {
        link_->AddLinkForce(ignition::math::Vector3d(0, body_moment, 0));
        
      }
    }

    // double body_torque = cmd.body_torque[0];
    // gazebo::physics::LinkPtr body_link_ = model_->GetLink(cmd.body_name[0]);
    // if (body_link_ != NULL) {
    //   body_link_->AddRelativeTorque(ignition::math::Vector3d(0, 0, body_torque));
    // }
  }


  //apply thrust data from DJI E2000 powertrain data () [equation of force is per rotor]
  double GazeboRosDroneForcePrivate::CalculateThrust(double w) {
    double thrust = (0.0002*w*w) - 0.0083*abs(w) + 1.0069; //_rotor_thrust_coeff * w * w;
    return thrust;
  }
  
  //apply torque given from DJI E2000 powertrain data (extrapolated from power/thrust data) [equation of force is per rotor]
  double GazeboRosDroneForcePrivate::CalculateTorque(double w) {
    double torque = 0; //copysign(100*(538.54*pow(abs(w),-2.282))*(1.2041*0.223459*0.266701*pow((abs(w)*0.266701),2)), w);//new formula:  (2.7488*pow(w,-0.282))  //OLD FORMULA: (538.54*pow(abs(w),-2.282))*(1.2041*0.223459*0.266701*pow((abs(w)*0.266701),2)) //copysign(_rotor_torque_coeff * w * w, w);
    return torque;
  }

  //apply torque given from DJI E2000 powertrain data (extrapolated from power/thrust data) [equation of force is per rotor]
  double GazeboRosDroneForcePrivate::CalculateBodyTorque(double q) {
    double force = q/0.4475; //torque divided by the distance from the body center (see drone.xacro cylinder radius)
    return force;
  }

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosDroneForce)
}  // namespace gazebo_plugins
