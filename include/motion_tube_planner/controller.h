#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <motion_tube_planner/template_generator.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class Controller
{
   public:
    Controller(ros::NodeHandle& nh);

    void init();
    void spin();

    void lasercb(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomcb(const nav_msgs::Odometry::ConstPtr& msg);

    void controlLoop(const ros::TimerEvent& event);

    void visualizeTemplate(unsigned int ind);

   private:
    sensor_msgs::LaserScan::ConstPtr laser_scan_;

    ros::Publisher template_viz_pub_;

    ros::Subscriber odom_sub_;
    ros::Subscriber laser_sub_;

    ros::Timer controller_timer_;

    Eigen::Vector3d odom_;

    Eigen::Affine3d sensor_to_base_;
    Eigen::Affine3d base_to_odom_;

    std::vector<Eigen::Vector2d> footprint_;

    std::unique_ptr<TemplateGenerator> template_generator_;

    std::string frame_id_;

    bool is_generator_init_;
    bool is_odom_set_;

    double d_aug_;
    double d_sample_;
};
