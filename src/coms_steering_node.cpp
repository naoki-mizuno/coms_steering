#include "coms_steering/coms_steering.h"
#include "coms_steering/coms_steering.h"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <serial/serial.h>

#include <csignal>
#include <string>
#include <vector>

ComsSteering* controller_ptr;

void
signal_handler(int signum) {
    if (controller_ptr != nullptr) {
        controller_ptr->off();
        ros::shutdown();
        std::exit(0);
    }
}

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "coms_steering_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p{"~"};

    float frequency;
    std::string topic_prefix;
    nh_p.param("frequency", frequency, static_cast<float>(1000));
    nh_p.param("topic_prefix", topic_prefix, std::string{""});
    if (topic_prefix.back() != '/') {
        topic_prefix += "/";
    }

    ComsSteering controller{topic_prefix};

    ros::Duration{0.5}.sleep();

    try {
        controller.connect();
        ROS_INFO("Connected to steering:");
    }
    catch (const serial::PortNotOpenedException& e) {
        ROS_ERROR_STREAM(e.what());
        return 1;
    }
    catch (const serial::IOException& e) {
        ROS_ERROR_STREAM(e.what());
        return 1;
    }

    ros::Duration{0.5}.sleep();

    controller_ptr = &controller;
    signal(SIGINT, signal_handler);

    std_msgs::Float64 angle;
    ros::Publisher angle_pub = nh.advertise<std_msgs::Float64>(topic_prefix + "angle", 1);
    ros::Rate rate{frequency};

    controller.init();

    controller.on();
    while (ros::ok()) {
        angle.data = controller.get_rad();
        angle_pub.publish(angle);

        rate.sleep();
        ros::spinOnce();
    }
    controller.off();

    return 0;
}
