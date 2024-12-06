#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "microstrain_inertial_msgs/msg/filter_heading.hpp"

#include "pid.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <limits>
#include <math.h>


#define PARK_ORIGIN_ROT -0.6021965548550632

class HeadingController : public rclcpp::Node
{
  public:
    HeadingController() : Node("heading_controller")
    {
      //Qos profile
      auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);

      // Subscribers
      hsd_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("controller/setpoint", sub_qos,
        std::bind(&HeadingController::callback_setpoint, this, std::placeholders::_1));

      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", sub_qos,
        std::bind(&HeadingController::callback_imu, this, std::placeholders::_1));

      estop_sub_ = this->create_subscription<std_msgs::msg::Bool>("/estop", sub_qos,
        std::bind(&HeadingController::callback_estop, this, std::placeholders::_1));

      heading_sub_ = this->create_subscription<microstrain_inertial_msgs::msg::FilterHeading>("heading", sub_qos,
        std::bind(&HeadingController::callback_heading, this, std::placeholders::_1));

      // Publishers
      rpy_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("controller/rpy", 10);
      error_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("controller/error", 10);
      desired_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("controller/desired", 10);
      twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

      //Desired state
      desired_heading_ = 0;
      desired_speed_ = 0;
      last_speed_ = 0;
      delta_speed_ = 0.05;

      //Current state
      roll_ = 0;
      pitch_ = 0;
      yaw_ = 0;
      roll_dot_ = 0;
      pitch_dot_ = 0;
      yaw_dot_ = 0;

      // PID controllers
      this->declare_parameter("heading_kp", 1.0);
      this->get_parameter("heading_kp",heading_kp_);

      this->declare_parameter("heading_ki", 0.005);
      this->get_parameter("heading_ki", heading_ki_);

      this->declare_parameter("heading_kd", 0.1);
      this->get_parameter("heading_kd", heading_kd_);

      this->declare_parameter("max_vel", 0.5);
      this->get_parameter("max_vel", max_vel_);

      this->declare_parameter("min_vel", 0.15);
      this->get_parameter("min_vel", min_vel_);

      this->declare_parameter("max_ang_acc", 25.0);
      this->get_parameter("max_ang_acc", delta_speed_);

      this->declare_parameter("origin_rotation", -0.6021965548550632);
      this->get_parameter("origin_rotation", origin_rotation_);

      double inf = std::numeric_limits<double>::infinity();

      //Update rate
      double rate = 0.1;
      estop_ = false;

      heading_pid_ = new PID(rate, inf, -inf, heading_kp_, heading_kd_, heading_ki_);

      std::chrono::milliseconds rate_ms((int)(rate*1000));
      timer_ = this->create_wall_timer(rate_ms, std::bind(&HeadingController::callback_timer, this));
    }

  private:
    void callback_timer()
    {
      double heading_error = wrap_angle(desired_heading_ - yaw_);
      double heading_correction = heading_pid_->calculate(heading_error, -yaw_dot_);

      //RCLCPP_WARN(this->get_logger(), "Heading: %f", yaw_);
      //RCLCPP_WARN(this->get_logger(), "Setpoint: %f", desired_heading_);

      heading_correction = last_speed_ + std::min(std::max(heading_correction - last_speed_, -delta_speed_), delta_speed_);
      //WRCLCPP_WARN(this->get_logger(), "Input: %f", heading_correction);

      if (safety_check(hsd_epoch_))
      {
        if (safety_check(rpy_epoch_))
        {
          auto twist = geometry_msgs::msg::Twist();

          //if (desired_speed_ != 0.0)
          //{

          double val = std::max(std::min(heading_correction, M_PI/24), -M_PI/24);
          val = std::abs(val/(M_PI/24));
          double speed = (desired_speed_-val*desired_speed_)+min_vel_;

          twist.angular.z = heading_correction;
          twist.linear.x = speed;

          if (estop_)
          {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            RCLCPP_WARN(this->get_logger(), "Estop engaged.");
          }

          twist_pub_->publish(twist);
          last_speed_ = heading_correction;

          auto error = geometry_msgs::msg::Vector3Stamped();
          error.header.stamp = now();
          error.header.frame_id = "base_link";
          error.vector.x = heading_error;

          error_pub_->publish(error);

          auto desired = geometry_msgs::msg::Vector3Stamped();
          desired.header.stamp = now();
          desired.header.frame_id = "base_link";
          desired.vector.x = desired_heading_;

          desired_pub_->publish(desired);


        }
        // else
        // {
        //     //RCLCPP_WARN(this->get_logger(), "MISSING: HEADING!");
        // }
      }
      // else
      // {
      //     //RCLCPP_WARN(this->get_logger(), "MISSING: SETPOINT!");
      // }
    }

    void callback_estop(const std::shared_ptr<std_msgs::msg::Bool> msg)
    {
        estop_ = msg->data;
    }

    void callback_heading(const std::shared_ptr<microstrain_inertial_msgs::msg::FilterHeading> msg)
    {
        rpy_epoch_ = now();
        //RCLCPP_WARN(this->get_logger(), "Got HEADING!");
        yaw_ = msg->heading_rad;
        yaw_ = yaw_ - origin_rotation_;
    }

    void callback_imu(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
    {
      //RCLCPP_WARN(this->get_logger(), "Got IMU!");

      tf2::Quaternion q(msg->orientation.x,
        msg->orientation.y,msg->orientation.z,
        msg->orientation.w);


      //tf2::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);

      //WITMOTION IMU's -- THIS IS NOT NEEDED IF CALIBRATED WITH X-EAST
      //First, convert NWU to ENU
      //tf2::Quaternion q_enu(0.0, 0.0, sin(-M_PI/4.0), cos(-M_PI/4.0));

      //First, convert NED to ENU
      // tf2::Quaternion q0(0.0, sin(M_PI/2.0), 0.0, cos(M_PI/2.0));
      // tf2::Quaternion q1(0.0, 0.0, sin(M_PI/2.0), cos(M_PI/2.0));
      // tf2::Quaternion q_enu = q1*q0*q;

      //Convert ENU to DP frame
      double z_dp = -origin_rotation_;
      tf2::Quaternion q_dp(0.0, 0.0, sin(z_dp/2.0), cos(z_dp/2.0));

      //tf2::Quaternion q_f = q_dp*q_enu*q;
      tf2::Quaternion q_f = q_dp*q;

      //yaw_ = atan2(2.0 * (q_f.w() * q_f.z() + q_f.x() * q_f.y()), 1 - 2.0* (q_f.y() * q_f.y() + q_f.z() * q_f.z()));

      //tf2::Quaternion p(0, sin(M_PI/4), 0, cos(M_PI/4));

      //tf2::Quaternion qp = p*q;

      roll_dot_ = msg->angular_velocity.x;
      pitch_dot_ = msg->angular_velocity.y;
      yaw_dot_ = msg->angular_velocity.z;

      auto debug_msg = geometry_msgs::msg::Vector3Stamped();
      debug_msg.header = msg->header;
      debug_msg.vector.x = roll_;
      debug_msg.vector.y = pitch_;
      debug_msg.vector.z = yaw_;

      rpy_pub_->publish(debug_msg);
    }

    double quat_to_yaw(double x, double y, double z, double w) {
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        return yaw;
    }

    void callback_setpoint(const std::shared_ptr<geometry_msgs::msg::Vector3Stamped> msg)
    {

      //RCLCPP_WARN(this->get_logger(), "Got setpoint!");
      hsd_epoch_ = now();

      desired_heading_ = msg->vector.z;
      desired_speed_ = msg->vector.x;
    }

    bool safety_check(rclcpp::Time epoch)
    {
      if ((now().nanoseconds() - epoch.nanoseconds()) * 1e-9 > 1.0)
      {
        //RCLCPP_INFO(this->get_logger(), "Missing sensor update rate!");
        return false;
      }
      return true;
    }

    double wrap_angle (double angle)
    {
      angle = fmod(angle + M_PI, 2.0 * M_PI);

      if (angle < 0)
      {
        angle += 2.0*M_PI;
      }

      return angle - M_PI;
    }

    // double speed_to_effort(double speed)
    // {
    //   if (speed < 0)
    //   {
    //     return -(p1_*speed*speed - p2_*speed);
    //   }
    //   return p1_*speed*speed + p2_*speed;
    // }

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr hsd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Subscription<microstrain_inertial_msgs::msg::FilterHeading>::SharedPtr heading_sub_;
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr desired_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    // ROS parameters
    double heading_kp_;
    double heading_ki_;
    double heading_kd_;
    double max_vel_;
    double min_vel_;
    double origin_rotation_;

    // Safety checks
    rclcpp::Time hsd_epoch_;
    rclcpp::Time rpy_epoch_;

    PID *heading_pid_;

    //Desried state
    double desired_heading_;
    double desired_speed_;
    double last_speed_;
    double delta_speed_;

    //Current state
    double roll_;
    double pitch_;
    double yaw_;
    double roll_dot_;
    double pitch_dot_;
    double yaw_dot_;
    bool estop_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto heading_controller = std::make_shared<HeadingController>();

  rclcpp::spin(heading_controller);

  rclcpp::shutdown();

  return 0;
}
