#ifndef COMS_STEERING_H_
#define COMS_STEERING_H_

#include "SteeringController.h"

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

class ComsSteering {
public:
    using pulse_t = SteeringController::pulse_t;

    static
    void
    abort(const int signum);

    /* Constructors, Destructor, and Assignment operators {{{ */
    ComsSteering(const std::string& port,
                 const unsigned baudrate,
                 const std::pair<double, pulse_t>& limit_ccw,
                 const std::pair<double, pulse_t>& limit_cw,
                 const pulse_t origin_offset);


    // Copy constructor
    ComsSteering(const ComsSteering& other) = delete;

    // Move constructor
    ComsSteering(ComsSteering&& other) = delete;

    // Destructor
    ~ComsSteering();

    // Assignment operator
    ComsSteering&
    operator=(const ComsSteering& other) = delete;

    // Move assignment operator
    ComsSteering&
    operator=(ComsSteering&& other) = delete;
    /* }}} */

    inline
    SteeringController&
    steering() {
        return steering_;
    }

    void
    steer_callback(const std_msgs::Float64MultiArray& msg);

private:
    ros::Subscriber steering_sub;
    SteeringController steering_;
};

#endif /* end of include guard */
