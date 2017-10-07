#include "coms_steering/coms_steering.h"

using pulse_t = SteeringController::pulse_t;

/* Static methods */
void
ComsSteering::abort(int signum) {
    ROS_INFO("Aborting...");
    SteeringController::abort = true;
    ros::shutdown();
}

ComsSteering::ComsSteering(const std::string& port,
                           const unsigned baud,
                           const std::pair<double, pulse_t>& limit_ccw,
                           const std::pair<double, pulse_t>& limit_cw,
                           const pulse_t origin_offset)
    : steering_{port, baud, limit_ccw, limit_cw, origin_offset}
{
}

void
ComsSteering::steer_callback(const std_msgs::Float64MultiArray& msg) {
    // Position, speed, and acceleration in radians
    auto pos = msg.data[0];
    //auto vel = msg.data[1];
    //auto acc = msg.data[2];
    steering_.set_target_angle(pos);
}
