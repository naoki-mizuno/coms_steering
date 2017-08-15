#include "coms_steering.h"
#include "SteeringController.h"

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <string>
#include <vector>

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "coms_steering_node");
    ros::NodeHandle nh{"~"};

    // Get parameters
    std::string port;
    int baudrate;
    float frequency;
    int origin_offset;
    float wheelbase;
    float steering_gear_ratio;
    float rear_shaft_to_body_center;
    // Limits [rad, pulse count] later converted to [double, pulse_t]
    std::vector<double> limit_ccw;
    std::vector<double> limit_cw;

    nh.getParam("port", port);
    nh.param("baudrate", baudrate, 38400);
    nh.param("frequency", frequency, static_cast<float>(1000));
    nh.param("origin_offset", origin_offset, 0);
    nh.getParam("limit_ccw", limit_ccw);
    nh.getParam("limit_cw", limit_cw);
    nh.getParam("wheelbase", wheelbase);
    nh.getParam("steering_gear_ratio", steering_gear_ratio);
    nh.getParam("rear_shaft_to_body_center", rear_shaft_to_body_center);

    // Convert to [double, pulse_t]
    auto lim_rad_ccw = static_cast<double>(limit_ccw[0]);
    auto lim_rad_cw = static_cast<double>(limit_cw[0]);
    using pulse_t = SteeringController::pulse_t;
    auto lim_pulse_ccw = static_cast<pulse_t>(limit_ccw[1]);
    auto lim_pulse_cw = static_cast<pulse_t>(limit_cw[1]);

    SteeringController steering{
        port,
        static_cast<unsigned>(baudrate),
        // Limit [rad, pulse count] for CCW direction
        std::make_pair(lim_rad_ccw, lim_pulse_ccw),
        // Limit [rad, pulse count] for CW direction
        std::make_pair(lim_rad_cw, lim_pulse_cw),
        origin_offset,
    };
    try {
        steering.connect();
        ROS_INFO("Connected to device:");
        ROS_INFO_STREAM("  PORT: " << port);
        ROS_INFO_STREAM("  BAUD: " << baudrate);
    }
    catch (const serial::PortNotOpenedException& e) {
        ROS_ERROR_STREAM(e.what());
        return 1;
    }
    catch (const serial::IOException& e) {
        ROS_ERROR_STREAM(e.what());
        return 1;
    }

    steering.init();

    ros::Publisher angle_pub = nh.advertise<std_msgs::Float64>("angle", 1);
    ros::Rate r{frequency};

    steering.on();
    std_msgs::Float64 msg_angle;
    while (ros::ok()) {
        // TODO: do stuff here
        const double pi = 3.14159265358979323846;

        std::cout << "CCW" << std::endl;
        std::cout << "180" << std::endl;
        steering.set(pi);
        sleep(1);
        steering.set(0);
        std::cout << "90" << std::endl;
        steering.set(pi / 2);
        sleep(1);
        steering.set(0);
        std::cout << "45" << std::endl;
        steering.set(pi / 4);
        sleep(1);
        steering.set(0);

        std::cout << "CW" << std::endl;
        std::cout << "-180" << std::endl;
        steering.set(-pi);
        sleep(1);
        steering.set(0);
        std::cout << "-90" << std::endl;
        steering.set(-pi / 2);
        sleep(1);
        steering.set(0);
        std::cout << "-45" << std::endl;
        steering.set(-pi / 4);
        sleep(1);
        steering.set(0);

        ros::spinOnce();
        r.sleep();
    }
    steering.off();

    return 0;
}
