#include <chrono>
#include <memory>
#include <cmath>
#include <complex>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

// For mesh marker
#include "visualization_msgs/msg/marker.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// Math libraries
#include "robot_math_utils/robot_math_utils_v1_8.hpp"
#include "filters/hpf.hpp"
#include "filters/lpf.hpp"
#include "kf_cpp/ekf.hpp"

using namespace std::chrono_literals;
using Eigen::Vector3d;


class IMUMotionTracker : public rclcpp::Node
{
public:
  IMUMotionTracker()
  : Node("imu_motion_tracker")
  {
    // Create a TF broadcaster.
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribe to the quaternion topic.
    quat_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/mpu6050_imu/quat", 10,
      std::bind(&IMUMotionTracker::quat_callback_, this, std::placeholders::_1));

    // Subscribe to the acceleration topic.
    acc_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/mpu6050_imu/acc", 10,
      std::bind(&IMUMotionTracker::acc_calback_, this, std::placeholders::_1));

    // Optional: publish a mesh marker for visualization.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    mesh_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/imu_motion_tracker/mesh_marker", qos);

    // New publishers for velocity and position.
    quat_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/imu_motion_tracker/quat", 10);
    pos_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/imu_motion_tracker/pos", 10);
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/imu_motion_tracker/vel", 10);
    acc_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/imu_motion_tracker/acc", 10);

    // Sampling rate and period.
    fs_ = 30.0;
    Ts_ = 1.0 / fs_;

    // Initialize orientation variables
    quat_.w = 1.0;
    quat_.x = 0.0;
    quat_.y = 0.0;
    quat_.z = 0.0;

    // Initialize KF for translation estimation.
    updateEKF();

    // Instantiate high-pass filters.
    pos_hpf_ = std::make_unique<HighPassFilter<Vector3d>>(0.5, Ts_);
    vel_hpf_ = std::make_unique<HighPassFilter<Vector3d>>(0.5, Ts_);
    acc_hpf_ = std::make_unique<HighPassFilter<Vector3d>>(0.5, Ts_);

    // Instantiate low-pass filters.
    pos_lpf_ = std::make_unique<LowPassFilter<Vector3d>>(15.0, Ts_);
    vel_lpf_ = std::make_unique<LowPassFilter<Vector3d>>(15.0, Ts_);
    acc_lpf_ = std::make_unique<LowPassFilter<Vector3d>>(5.0, Ts_);

    // Create a timer to trigger integration and TF broadcast at a fixed rate.
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(Ts_ * 1000)),
      std::bind(&IMUMotionTracker::timer_callback_, this)
    );

    RCLCPP_INFO(this->get_logger(), "IMUMotionTracker node has been started.");
  }

private:
  // Subscribers and Timer
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr quat_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr acc_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF Broadcaster and Mesh Marker publisher
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_marker_pub_;

  // Publishers.
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr quat_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pos_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr acc_pub_;

  // Sampling rate and period
  float fs_, Ts_;

  // Orientation
  geometry_msgs::msg::Quaternion quat_;

  // Translation
  Vector3d acc_;
  int dim_x_, dim_z_, dim_w_;
  VectorXd X_;
  MatrixXd F_;
  MatrixXd H_;
  std::shared_ptr<ExtendedKalmanFilter> ekf_;
  MatrixXd Gamma_;

  // Filters
  std::unique_ptr<HighPassFilter<Vector3d>> pos_hpf_;
  std::unique_ptr<HighPassFilter<Vector3d>> vel_hpf_;
  std::unique_ptr<HighPassFilter<Vector3d>> acc_hpf_;

  std::unique_ptr<LowPassFilter<Vector3d>> pos_lpf_;
  std::unique_ptr<LowPassFilter<Vector3d>> vel_lpf_;
  std::unique_ptr<LowPassFilter<Vector3d>> acc_lpf_;

  // Callback for quaternion
  void quat_callback_(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    quat_ = *msg;
  }

  // Acceleration callback
  void acc_calback_(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    acc_ = Vector3d(msg->x, msg->y, msg->z);
  }


  void updateStateSpaceModel() 
  {
    dim_x_ = 9;
    dim_z_ = 3;
    dim_w_ = 3;

    X_ = VectorXd::Zero(dim_x_);

    F_ = MatrixXd::Zero(dim_x_, dim_x_);
    F_ << 1, 0, 0, Ts_, 0, 0, 0.5 * Ts_ * Ts_, 0, 0,
          0, 1, 0, 0, Ts_, 0, 0, 0.5 * Ts_ * Ts_, 0,
          0, 0, 1, 0, 0, Ts_, 0, 0, 0,
          0, 0, 0, 1, 0, 0, Ts_, 0, 0,
          0, 0, 0, 0, 1, 0, 0, Ts_, 0,
          0, 0, 0, 0, 0, 1, 0, 0, Ts_,
          0, 0, 0, 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1;
    
    H_ = MatrixXd::Zero(dim_z_, dim_x_);
    H_ << 0, 0, 0, 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1;

  }

  void updateEKF()
  {
    // Initialize the state-space model.
    updateStateSpaceModel();

    ekf_ = std::make_shared<ExtendedKalmanFilter>(dim_x_, dim_z_);
    ekf_->setF(F_);
    ekf_->setH(H_);

    // Measurement noise covariance
    double sigma_upsilon = pow(10.0, -1.0);
    ekf_->setMeasurementNoise(MatrixXd::Identity(dim_z_, dim_z_) * sigma_upsilon * sigma_upsilon);

    // Process noise covariance
    double sigma_w = pow(10.0, 0.0);
    Gamma_ = MatrixXd::Zero(dim_x_, dim_w_);
    Gamma_ << Ts_ * Ts_ * Ts_ / 6.0, 0, 0,
              0, Ts_ * Ts_ * Ts_ / 6.0, 0,
              0, 0, Ts_ * Ts_ * Ts_ / 6.0,
              Ts_ * Ts_ / 2.0, 0, 0,
              0, Ts_ * Ts_ / 2.0, 0,
              0, 0, Ts_ * Ts_ / 2.0,
              Ts_, 0, 0,
              0, Ts_, 0,
              0, 0, Ts_;

    ekf_->setProcessNoise(Gamma_ * sigma_w * sigma_w * Gamma_.transpose());

  }

  VectorXd fx(const Eigen::VectorXd &x)
  {
      return ekf_->getF() * x;
  }

  MatrixXd jacobian_F(const Eigen::VectorXd &)
  {
      return ekf_->getF();
  }

  MatrixXd jacobian_H(const Eigen::VectorXd &)
  {
      return ekf_->getH();
  }

  VectorXd hx(const Eigen::VectorXd &x)
  {
      return ekf_->getH() * x;
  }

  void translationEstimation() 
  {
    // Perform LPF and HPF in each dimensions
    // X_.segment(0, 3) = pos_hpf_->filter( X_.segment(0, 3) );
    // X_.segment(3, 3) = vel_hpf_->filter( X_.segment(3, 3) );

    // X_.segment(0, 3) = pos_lpf_->filter( X_.segment(0, 3) );
    // X_.segment(3, 3) = vel_lpf_->filter( X_.segment(3, 3) );

    // X_.segment(0, 3) = pos_hpf_->filter( pos_lpf_->filter( X_.segment(0, 3) ) );
    // X_.segment(3, 3) = vel_hpf_->filter( vel_lpf_->filter( X_.segment(3, 3) ) );

    // X_.segment(6, 3) = acc_lpf_->filter( acc_ );
    // X_.segment(6, 3) = acc_hpf_->filter( acc_ );
    X_.segment(6, 3) = acc_hpf_->filter( acc_lpf_->filter( acc_ ) );
    // X_.segment(6, 3) = acc_;

    // State propagation using the state-space model.
    X_ = F_ * X_;
  }

  void translationKFEstimation()
  {
    // Vector3d measurement = acc_;
    Vector3d measurement = acc_hpf_->filter( acc_lpf_->filter( acc_ ) );

    if (!measurement.allFinite()) {
        RCLCPP_WARN(this->get_logger(), "Measurement contains NaN or Inf, skipping this iteration.");
        return;
    }

    // EKF time update
    ekf_->predict(jacobian_F(ekf_->getState()), fx(ekf_->getState()));

    if (!ekf_->getState().allFinite()) {
        RCLCPP_WARN(this->get_logger(), "State contains NaN or Inf, skipping this iteration.");
        return;
    }

    // EKF measurement update
    ekf_->update(measurement, jacobian_H(ekf_->getState()), hx(ekf_->getState()));

    // EKF estimated position
    VectorXd ekf_estimate = ekf_->getState();
    X_.segment(0, 3) = ekf_estimate.head(3);
    X_.segment(3, 3) = ekf_estimate.segment(3, 3);
    X_.segment(6, 3) = ekf_estimate.tail(3);


  }

  void translationOmit()
  {
    X_ = VectorXd::Zero(dim_x_);
  }

  void publishStates()
  {
    // Publish quaternion.
    quat_pub_->publish(quat_);

    // Publish filtered position.
    geometry_msgs::msg::Vector3 pos_msg;
    pos_msg.x = X_(0);
    pos_msg.y = X_(1);
    pos_msg.z = X_(2);
    pos_pub_->publish(pos_msg);

    // Publish filtered velocity.
    geometry_msgs::msg::Vector3 vel_msg;
    vel_msg.x = X_(3);
    vel_msg.y = X_(4);
    vel_msg.z = X_(5);
    vel_pub_->publish(vel_msg);

    // Publish filtered acceleration.
    geometry_msgs::msg::Vector3 acc_msg;
    acc_msg.x = X_(6);
    acc_msg.y = X_(7);
    acc_msg.z = X_(8);
    acc_pub_->publish(acc_msg);
    
  }

  void publishMeshMarker(const std::string & parent_frame = "imu")
  {
    visualization_msgs::msg::Marker mesh_marker;
    mesh_marker.header.frame_id = parent_frame;
    mesh_marker.header.stamp = this->now();
    mesh_marker.id = 0;
    mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mesh_marker.action = visualization_msgs::msg::Marker::ADD;
    mesh_marker.scale.x = 1.0;
    mesh_marker.scale.y = 1.0;
    mesh_marker.scale.z = 1.0;
    mesh_marker.color.a = 1.0f;
    mesh_marker.mesh_use_embedded_materials = true;

    // Get the package share directory (ensure your package name is correct).
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("imu_motion_tracker");
    mesh_marker.mesh_resource = "package://imu_motion_tracker/meshes/mem_mesh/mem_mesh.obj";

    mesh_marker_pub_->publish(mesh_marker);
  }

  // Timer callback: perform integration and high-pass filtering, publish velocity and position, and broadcast TF.
  void timer_callback_() 
  {

    // Perform translation estimation.
    // translationEstimation();
    // translationKFEstimation();
    translationOmit();

    // Publish states.
    publishStates();
    
    // Broadcast TF transform using the latest quaternion and computed position.
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "world"; // Parent frame.
    transformStamped.child_frame_id = "imu";    // Child frame.

    transformStamped.transform.translation.x = X_(0);
    transformStamped.transform.translation.y = X_(1);
    transformStamped.transform.translation.z = X_(2);
    transformStamped.transform.rotation = quat_;

    tf_broadcaster_->sendTransform(transformStamped);

    publishMeshMarker("imu");
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