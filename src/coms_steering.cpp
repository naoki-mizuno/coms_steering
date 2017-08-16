#include "coms_steering.h"

using pulse_t = SteeringController::pulse_t;

ComsSteering::ComsSteering(const std::string& port,
                           const unsigned baudrate,
                           const std::pair<double, pulse_t>& limit_ccw,
                           const std::pair<double, pulse_t>& limit_cw,
                           const pulse_t origin_offset)
    : steering_{port, baudrate, limit_ccw, limit_cw, origin_offset}
{
}

ComsSteering::~ComsSteering()
{
}

void
ComsSteering::steer_callback(const std_msgs::Float64MultiArray& msg) {
    // Position, speed, and acceleration in radians
    auto pos = msg.data[0];
    auto vel = msg.data[1];
    auto acc = msg.data[2];
    steering_.set(pos, vel, acc);
}
