#include <XmlRpcValue.h>
#include <motion_tube_planner/controller.h>
#include <tf/tf.h>

Controller::Controller(ros::NodeHandle& nh)
{
    // get footprint parameter
    XmlRpc::XmlRpcValue footprint_param;
    if (!nh.getParam("footprint", footprint_param))
    {
        ROS_ERROR("Failed to get 'footprint' parameter");
        exit(-1);
    }

    if (footprint_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("'footprint' parameter is not an array");
        exit(-1);
    }

    for (int i = 0; i < footprint_param.size(); ++i)
    {
        if (footprint_param[i].getType() != XmlRpc::XmlRpcValue::TypeArray ||
            footprint_param[i].size() != 2)
        {
            ROS_ERROR("Invalid footprint point at index %d", i);
            exit(-1);
        }
        double x = static_cast<double>(footprint_param[i][0]);
        double y = static_cast<double>(footprint_param[i][1]);
        m_footprint_.emplace_back(x, y);
    }
}

void Controller::spin()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();
}

void Controller::lasercb(const sensor_msgs::LaserScan::ConstPtr& msg) { m_laser_scan_ = *msg; }

void Controller::odomcb(const nav_msgs::Odometry::ConstPtr& msg)
{
    m_odom_(0) = msg->pose.pose.position.x;
    m_odom_(1) = msg->pose.pose.position.y;
    m_odom_(2) = tf::getYaw(msg->pose.pose.orientation);
}
