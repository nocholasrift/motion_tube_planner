#include <XmlRpcValue.h>
#include <math.h>
#include <motion_tube_planner/controller.h>
#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

Controller::Controller(ros::NodeHandle& nh)
{
    ros::NodeHandle private_nh("~");

    // get footprint parameter
    XmlRpc::XmlRpcValue footprint_param;
    if (!private_nh.getParam("footprint", footprint_param))
    {
        ROS_ERROR("Failed to get 'footprint' parameter");
        exit(-1);
    }

    if (footprint_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("'footprint' parameter is not an array");
        exit(-1);
    }

    if (footprint_param.size() != 4)
    {
        ROS_ERROR("currently only support 4 points for footprint, got %d points",
                  footprint_param.size());
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
        footprint_.emplace_back(x, y);
    }

    private_nh.param("d_aug", d_aug_, 0.1);
    private_nh.param("d_sample", d_sample_, 0.2);
    private_nh.param<std::string>("frame_id", frame_id_, "map");

    is_generator_init_ = false;
    is_odom_set_       = false;

    // publisher and subscribers
    odom_sub_  = nh.subscribe("/odometry/filtered", 1, &Controller::odomcb, this);
    laser_sub_ = nh.subscribe("/front/scan", 1, &Controller::lasercb, this);

    template_viz_pub_ = nh.advertise<visualization_msgs::Marker>("template_viz", 1);

    controller_timer_ = nh.createTimer(ros::Duration(0.1), &Controller::controlLoop, this);
}

void Controller::init()
{
    // spin until laserscan is received for first time
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);

    bool is_frame_set = false;

    ros::Rate rate(10);
    while (ros::ok() && (!is_generator_init_ || !is_frame_set))
    {
        ros::spinOnce();
        try
        {
            geometry_msgs::TransformStamped sensor_to_base_tf = tf_buffer.lookupTransform(
                "base_link", "front_laser", ros::Time(0), ros::Duration(1.0));

            sensor_to_base_ = tf2::transformToEigen(sensor_to_base_tf.transform);

            is_frame_set = true;
        }
        catch (tf2::LookupException& e)
        {
            ROS_WARN("[Controller::init] Waiting for sensor to base transform: %s", e.what());
        }

        rate.sleep();
    }

    // generate a wide variety of templates
    ROS_INFO("Generating templates");
    int ind = 0;
    for (double v = 0.1; v < 1.6; v += 0.1)
    {
        for (double w = -1.5; w < 1.6; w += 0.1)
        {
            // compute horizon based on velocity
            double horizon = 0.5 + ind * .1;
            Eigen::Vector2d input(v, w);
            template_generator_->generateTemplate(input, horizon);
        }
        ++ind;
    }

    ROS_INFO("Generated %lu templates", template_generator_->templates_.size());
}

void Controller::spin()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();
}

void Controller::lasercb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_scan_ = msg;
    if (!is_generator_init_)
    {
        LidarMetadata meta_data;
        meta_data.angle_min       = msg->angle_min;
        meta_data.angle_max       = msg->angle_max;
        meta_data.angle_increment = msg->angle_increment;
        meta_data.range_min       = msg->range_min;
        meta_data.range_max       = msg->range_max;
        is_generator_init_        = true;

        template_generator_ =
            std::make_unique<TemplateGenerator>(footprint_, meta_data, d_aug_, d_sample_);

        ROS_INFO("Initialized Laserscan!");
    }
}

void Controller::odomcb(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_(0) = msg->pose.pose.position.x;
    odom_(1) = msg->pose.pose.position.y;
    odom_(2) = tf::getYaw(msg->pose.pose.orientation);

    geometry_msgs::TransformStamped base_to_odom_tf;
    base_to_odom_tf.header         = msg->header;
    base_to_odom_tf.child_frame_id = "base_link";

    base_to_odom_tf.transform.translation.x = odom_(0);
    base_to_odom_tf.transform.translation.y = odom_(1);
    base_to_odom_tf.transform.translation.z = 0;
    base_to_odom_tf.transform.rotation      = msg->pose.pose.orientation;

    base_to_odom_ = tf2::transformToEigen(base_to_odom_tf.transform);

    is_odom_set_ = true;
}

void Controller::controlLoop(const ros::TimerEvent& event)
{
    if (!is_odom_set_ || !is_generator_init_)
    {
        ROS_WARN("Odom not set or template generator not initialized yet.");
        return;
    }

    double score       = 1e6;
    int champ_template = 0;
    // iterate over all templates and score if they are in free space
    for (unsigned int i = 0; i < template_generator_->templates_.size(); ++i)
    {
        const Template& t = template_generator_->templates_[i];
        // check if the template is in free space
        bool is_free = true;
        int j        = 0;
        for (const auto& ind : t.inds)
        {
            if (ind >= laser_scan_->ranges.size())
            {
                ROS_WARN("Index %u out of bounds for laser scan ranges of size %zu", ind,
                         laser_scan_->ranges.size());
                continue;
            }
            Eigen::Vector2d sample_point = t.all_pts[j];
            /*if (laser_scan_->ranges[ind] < (sample_point - odom_.head<2>()).norm())*/
            if (laser_scan_->ranges[ind] < sample_point.norm())
            {
                ROS_INFO_STREAM("index is: " << ind
                                             << " sample is: " << sample_point.transpose()
                                             << " and range is: " << laser_scan_->ranges[ind]);
                is_free = false;
                break;
            }

            ++j;
        }

        size_t top_len     = t.top_bdry.size();
        Eigen::Vector2d pt = t.top_bdry[top_len / 2];
        Eigen::Vector3d pos =
            base_to_odom_ * sensor_to_base_ * Eigen::Vector3d(pt[0], pt[1], 0);
        double dist = (Eigen::Vector2d(-2.25, 10.) - pos.head<2>()).norm();

        if (is_free)

        {
            std::cout << "found a free template!" << std::endl;
        }

        if (is_free && dist < score)
        {
            score          = dist;
            champ_template = i;
        }
    }

    visualizeTemplate(champ_template);
}

void Controller::visualizeTemplate(unsigned int ind)
{
    const Template& t = template_generator_->templates_[ind];

    ROS_INFO_STREAM("visualizing template with input " << t.input.transpose());
    visualization_msgs::Marker marker;
    marker.header.frame_id    = frame_id_;
    marker.header.stamp       = ros::Time::now();
    marker.ns                 = "template_viz";
    marker.id                 = ind;
    marker.type               = visualization_msgs::Marker::SPHERE_LIST;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = 0.05;  // line width
    marker.scale.y            = 0.05;  // line width
    marker.scale.z            = 0.05;  // line width
    marker.color.r            = 0.0;
    marker.color.g            = 0.0;
    marker.color.b            = 1.0;
    marker.color.a            = 1.0;

    for (const auto& pt : t.all_pts)
    {
        geometry_msgs::Pose p;
        p.position.x    = pt[0];
        p.position.y    = pt[1];
        p.position.z    = 0;
        p.orientation.w = 1.0;  // assuming 2D, no rotation

        geometry_msgs::Pose baselink_p;
        geometry_msgs::Pose odom_p;

        Eigen::Vector3d pt_odom(pt[0], pt[1], 0);
        pt_odom = base_to_odom_ * sensor_to_base_ * pt_odom;

        geometry_msgs::Point pt_o;
        pt_o.x = pt_odom[0];
        pt_o.y = pt_odom[1];
        pt_o.z = 0;
        marker.points.push_back(pt_o);

        /*std::cout << "Boundary point: " << pt_o.x << ", " << pt_o.y << std::endl;*/
    }

    /*for (const auto& pt : t.top_bdry)*/
    /*{*/
    /*    geometry_msgs::Point p;*/
    /*    p.x = pt[0] + odom_[0];*/
    /*    p.y = pt[1] + odom_[1];*/
    /*    p.z = 0.0;  // assuming 2D*/
    /*    marker.points.push_back(p);*/
    /**/
    /*    std::cout << "Top boundary point: " << p.x << ", " << p.y << std::endl;*/
    /*}*/
    /*for (int i = t.right_bdry.size() - 1; i >= 0; --i)*/
    /*{*/
    /*    const Eigen::Vector2d& pt = t.right_bdry[i];*/
    /*    geometry_msgs::Point p;*/
    /*    p.x = pt[0] + odom_[0];*/
    /*    p.y = pt[1] + odom_[1];*/
    /*    p.z = 0.0;  // assuming 2D*/
    /*    marker.points.push_back(p);*/
    /**/
    /*    std::cout << "Right boundary point: " << p.x << ", " << p.y << std::endl;*/
    /*}*/

    template_viz_pub_.publish(marker);
}
