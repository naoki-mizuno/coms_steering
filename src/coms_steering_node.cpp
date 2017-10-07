#include "coms_steering/coms_steering.h"
#include "coms_steering/SteeringController.h"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <serial/serial.h>

#include <csignal>
#include <string>
#include <vector>

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "coms_steering_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p{"~"};

    // Get parameters
    std::string port;
    int baud;
    float frequency;
    int origin_offset;
    // Limits [rad, pulse count] later converted to [double, pulse_t]
    std::vector<double> limit_ccw;
    std::vector<double> limit_cw;

    nh_p.getParam("port", port);
    nh_p.param("baud", baud, 38400);
    nh_p.param("frequency", frequency, static_cast<float>(1000));
    nh_p.param("origin_offset", origin_offset, 0);
    nh_p.getParam("limit_ccw", limit_ccw);
    nh_p.getParam("limit_cw", limit_cw);

    // Convert to [double, pulse_t]
    auto lim_rad_ccw = static_cast<double>(limit_ccw[0]);
    auto lim_rad_cw = static_cast<double>(limit_cw[0]);
    using pulse_t = SteeringController::pulse_t;
    auto lim_pulse_ccw = static_cast<pulse_t>(limit_ccw[1]);
    auto lim_pulse_cw = static_cast<pulse_t>(limit_cw[1]);

    ComsSteering controller{
        port,
        static_cast<unsigned>(baud),
        // Limit [rad, pulse count] for CCW direction
        std::make_pair(lim_rad_ccw, lim_pulse_ccw),
        // Limit [rad, pulse count] for CW direction
        std::make_pair(lim_rad_cw, lim_pulse_cw),
        origin_offset,
    };

    try {
        controller.steering().connect();
        ROS_INFO("Connected to steering:");
        ROS_INFO_STREAM("  PORT: " << port);
        ROS_INFO_STREAM("  BAUD: " << baud);
    }
    catch (const serial::PortNotOpenedException& e) {
        ROS_ERROR_STREAM(e.what());
        return 1;
    }
    catch (const serial::IOException& e) {
        ROS_ERROR_STREAM(e.what());
        return 1;
    }

    signal(SIGINT, ComsSteering::abort);

    std_msgs::Float64 angle;
    ros::Publisher angle_pub = nh.advertise<std_msgs::Float64>("angle", 1);
    ros::Subscriber steering_sub = nh.subscribe("cmd_steer",
                                                1,
                                                &ComsSteering::steer_callback,
                                                &controller);
    ros::Rate rate{frequency};

    controller.steering().init();

    controller.steering().on();
    while (ros::ok()) {
        angle.data = controller.steering().get_rad();
        angle_pub.publish(angle);

        rate.sleep();
        ros::spinOnce();
    }
    controller.steering().off();

    return 0;
}
