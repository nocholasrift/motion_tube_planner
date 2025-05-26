#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Core>

class Controller
{
   public:
    Controller(ros::NodeHandle& nh);
    void spin();

    void lasercb(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomcb(const nav_msgs::Odometry::ConstPtr& msg);

   private:
    sensor_msgs::LaserScan m_laser_scan_;
    std::vector<std::pair<double, double>> m_footprint_;
    Eigen::Vector3d m_odom_;
};
