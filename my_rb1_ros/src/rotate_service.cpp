#include <cmath>
#include <geometry_msgs/Twist.h>
#include <my_rb1_ros/Rotate.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

class MyRb1Server {
protected:
  ros::NodeHandle nh_;

  ros::Publisher pub_vel_;
  geometry_msgs::Twist cmd_vel_;
  ros::Subscriber sub_odom_;

  ros::ServiceServer my_rb1_server_;
  my_rb1_ros::Rotate::Request req_;
  my_rb1_ros::Rotate::Response res_;

public:
  MyRb1Server() {
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    my_rb1_server_ = nh_.advertiseService("/rotate_robot",
                                          &MyRb1Server::RotateServerCB, this);
  }

  ~MyRb1Server() {}

  void rotate_clockwise() {
    cmd_vel_.angular.z = 0.1;
    pub_vel_.publish(cmd_vel_);
  }

  void rotate_counterclockwise() {
    cmd_vel_.angular.z = -0.1;
    pub_vel_.publish(cmd_vel_);
  }

  void stop_motion() {
    cmd_vel_.angular.z = 0.0;
    pub_vel_.publish(cmd_vel_);
  }

  void RotateCB(const nav_msgs::OdometryConstPtr &msg) {
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    tfScalar roll, pitch, yaw;
    m.getEulerYPR(yaw, pitch, roll);

    ROS_INFO("\n yaw: %.1f ===> req: %d", yaw * 180 / M_PI, req_.degrees);
    if (req_.degrees == 90) {
      if (req_.degrees >= (yaw * 180 / M_PI)) {
        rotate_counterclockwise();
      } else {
        res_.result = "SUCCESS";
        stop_motion();
        ROS_INFO("\n The robot have made a 90 deg counterclockwise spin");

      }
    } else {
      if (req_.degrees <= (yaw * 180 / M_PI)) {
        rotate_clockwise();
      } else {
        res_.result = "SUCCESS";
        stop_motion();
        ROS_INFO("\n The robot have made a 90 deg clockwise spin");
      }
    }
  }

  bool RotateServerCB(my_rb1_ros::Rotate::Request &req,
                      my_rb1_ros::Rotate::Response &res) {
    req_ = req;
    res_ = res;
    sub_odom_ = nh_.subscribe("/odom", 10, &MyRb1Server::RotateCB, this);

    ROS_INFO("The robot is spinning ...");
    while (ros::ok() && res_.result != "SUCCESS") {
      ros::spinOnce();
    }
    sub_odom_.shutdown();
    res.result = res_.result;
    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_server");

  MyRb1Server my_rb1_server;

  ros::spin();
}