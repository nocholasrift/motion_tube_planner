#include <motion_tube_planner/controller.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_tube_planner");
    ros::NodeHandle nh;

    Controller controller(nh);
    controller.init();
    controller.spin();

    return 0;
}
