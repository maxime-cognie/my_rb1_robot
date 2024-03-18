#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <my_rb1_ros/Rotate.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sys/types.h>
#include <tf/transform_datatypes.h>

class MyRb1Server {
protected:
  ros::NodeHandle nh_;

  ros::Publisher pub_vel_;
  geometry_msgs::Twist cmd_vel_;
  ros::Subscriber sub_odom_;

  ros::ServiceServer my_rb1_server_;

  struct RotationInfo {
    int8_t init = 0;
    double initial_yaw = 0.0;
    double target_yaw = 180.0;
    double error_yaw = target_yaw - initial_yaw;
    int direction = true;
  };
  RotationInfo rotation_;

public:
  MyRb1Server() {
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    my_rb1_server_ = nh_.advertiseService("/rotate_robot",
                                          &MyRb1Server::RotateServerCB, this);
    ROS_INFO("SERVICE READY !!! \nWaiting for input rotation angle.");
  }

  ~MyRb1Server() {}

  // set the velocity for a given rotation direction
  void compute_vel() {
    if (rotation_.error_yaw > 3) {
      cmd_vel_.angular.z = rotation_.direction * 0.4;
    } else {
      cmd_vel_.angular.z = rotation_.direction * rotation_.error_yaw * 0.008;
    }
  }

  void OdomCB(const nav_msgs::OdometryConstPtr &msg) {
    tfScalar yaw = tf::getYaw(msg->pose.pose.orientation);
    if (rotation_.init == 0) {
      rotation_.initial_yaw = yaw;
      rotation_.init = 1;
    }
    // compute error that is useful only for really close angle because this
    // error doesn't work when you are switching between 180 and -180.
    double error = abs(angles::normalize_angle_positive(rotation_.target_yaw) -
                       angles::normalize_angle_positive(yaw));
    rotation_.error_yaw = angles::to_degrees(error);

    ROS_DEBUG("\n yaw: %.1f, target  %f, error %f", angles::to_degrees(yaw),
              angles::to_degrees(rotation_.target_yaw), rotation_.error_yaw);
  }

  bool RotateServerCB(my_rb1_ros::Rotate::Request &req,
                      my_rb1_ros::Rotate::Response &res) {

    ROS_INFO("\nSERVICE REQUESTED : the robot will rotate for %d degrees",
             req.degrees);
    sub_odom_ = nh_.subscribe("/odom", 10, &MyRb1Server::OdomCB, this);
    ros::Rate loop_rate(20);

    // direction of the rotation +1 clockwise, -1 counter clockwise
    rotation_.direction = req.degrees > 0 ? -1 : 1;

    while (ros::ok() && rotation_.error_yaw > 0.3) {
      if (rotation_.init == 2) {
        compute_vel();
        pub_vel_.publish(cmd_vel_);
      } else if (rotation_.init == 1) {
        rotation_.target_yaw = angles::normalize_angle(
            rotation_.initial_yaw + angles::from_degrees((double)req.degrees));
        rotation_.init = 2;
      } else {
        ROS_DEBUG("Waiting for odom msg!");
      }
      loop_rate.sleep();
      ros::spinOnce();
    }
    cmd_vel_.angular.z = 0.0;
    pub_vel_.publish(cmd_vel_);
    sub_odom_.shutdown();
    rotation_ = RotationInfo();
    ROS_INFO(
        "\nSERVICE SUCCESS : the robot end rotating %d degrees with success",
        req.degrees);
    res.result = "SUCCESS, rotation complete";

    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_server");

  MyRb1Server my_rb1_server;

  ros::spin();
}