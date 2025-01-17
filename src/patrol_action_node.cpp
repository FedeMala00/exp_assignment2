// Copyright 2019 Intelligent Robotics Lab
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

#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"


using namespace std::chrono_literals;
int buffer[4] = {0, 0, 0, 0};
int count = 0;
class Patrol : public plansys2::ActionExecutorClient
{
public:
  Patrol()
  : plansys2::ActionExecutorClient("patrol", 1s)
  {
    // Subscription to ArUco markers topic
    aruco_sub_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
      "/aruco_markers", 10,
      std::bind(&Patrol::aruco_callback, this, std::placeholders::_1));

    // Publisher for Int32MultiArray
    buffer_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/buffer_topic", 10);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    progress_ = 0.0;

    // Publisher for robot velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_vel_pub_->on_activate();
    buffer_pub_->on_activate();

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    cmd_vel_pub_->on_deactivate();
    buffer_pub_->on_deactivate();

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.1;
      send_feedback(progress_, "Patrol running");

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.linear.z = 0.0;
      cmd.angular.x = 0.0;
      cmd.angular.y = 0.0;
      cmd.angular.z = 0.5;
      // RCLCPP_INFO(get_logger(), "Patrolling...");

      cmd_vel_pub_->publish(cmd);
    } else {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.linear.z = 0.0;
      cmd.angular.x = 0.0;
      cmd.angular.y = 0.0;
      cmd.angular.z = 0.0;
      count++; // counts for seeing that it saw only one aruco marker
      if (count + 1 == sizeof(buffer) / sizeof(buffer[0])){ //if reads one more aruco marker, when patrolled is finisced, remove it
        buffer[count+1] = 0;
      }
      cmd_vel_pub_->publish(cmd);

      finish(true, 1.0, "Patrol completed");
    }
  }

  void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->marker_ids.size(); ++i) {
      // RCLCPP_INFO(get_logger(), "Detected marker ID: %ld", msg->marker_ids[i]);
      for(size_t j = 0; j < 4; ++j){
        if(msg->marker_ids[i] == buffer[j]){
          return;
        }
        else if (buffer[j] == 0 && msg->marker_ids[i] > 10){
          buffer[j] = msg->marker_ids[i];
          RCLCPP_INFO(get_logger(), "VALORE INSERITO IN BUFFER: %ld", buffer[j]);

          // Check if buffer is full
          bool buffer_full = true;
          for (int k = 0; k < 4; ++k) {
            if (buffer[k] == 0) {
              buffer_full = false;
              break;
            }
          }

          // Publish buffer if full
          if (buffer_full) {
            std_msgs::msg::Int32MultiArray buffer_msg;
            buffer_msg.data.insert(buffer_msg.data.end(), std::begin(buffer), std::end(buffer));
            buffer_pub_->publish(buffer_msg);
            RCLCPP_INFO(get_logger(), "Buffer full, published buffer");
          }
          return;
        }
      }
    }
  }

  float progress_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32MultiArray>::SharedPtr buffer_pub_;  
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();

  node->set_parameter(rclcpp::Parameter("action_name", "patrol"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}