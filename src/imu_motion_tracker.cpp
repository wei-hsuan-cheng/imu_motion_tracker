#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

// For mesh marker
#include "visualization_msgs/msg/marker.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class IMUMotionTracker : public rclcpp::Node
{
public:
  IMUMotionTracker()
  : Node("imu_motion_tracker")
  {
    // Create a TransformBroadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribe to the "imu_quaternion" topic
    subscription_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/mpu6050_imu/quat", 10,
      std::bind(&IMUMotionTracker::quaternion_callback, this, std::placeholders::_1));
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    mesh_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/imu_motion_tracker/mesh_marker", qos);
    
    RCLCPP_INFO(this->get_logger(), "IMU motion tracker node has been started.");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_marker_pub_;
  visualization_msgs::msg::Marker mesh_marker_;

  void PublishMeshMarker(std::string parent_frame = "imu")
  {
    mesh_marker_.header.frame_id = parent_frame;
    mesh_marker_.id = 0;
    mesh_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mesh_marker_.action = visualization_msgs::msg::Marker::ADD;
    mesh_marker_.scale.x = 1.0;
    mesh_marker_.scale.y = 1.0;
    mesh_marker_.scale.z = 1.0;
    mesh_marker_.color.a = 1.0f;
    mesh_marker_.mesh_use_embedded_materials = true;
    mesh_marker_.mesh_resource = "package://imu_motion_tracker/meshes/mem_mesh/mem_mesh.obj";

    auto current_time = this->get_clock()->now();
    mesh_marker_.header.stamp = current_time;
    mesh_marker_pub_->publish(mesh_marker_);
  }

  void quaternion_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    // Create a TransformStamped message to broadcast
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "map"; // parent frame
    transformStamped.child_frame_id = "imu";    // child frame

    // For this example, we'll use an identity translation.
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;

    // Use the quaternion from the subscribed message for the rotation.
    transformStamped.transform.rotation = *msg;

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transformStamped);

    RCLCPP_DEBUG(this->get_logger(), "Broadcasted TF: %f %f %f %f", msg->w, msg->x, msg->y, msg->z);

    // Publish mesh marker
    PublishMeshMarker("imu");
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMUMotionTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
