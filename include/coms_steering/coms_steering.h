#ifndef COMS_STEERING_H_
#define COMS_STEERING_H_

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <coms_msgs/Steering.h>
#include <serial/serial.h>

#include <cstdio>
#include <string>
#include <utility>

class ComsSteering {
public:
    using pulse_t = long long int;

    static const unsigned READ_WRITE_TIMEOUT_MS = 250;

    // Default speed for Cool Muscle (rad/s)
    static constexpr double DEFAULT_S = 0.01729543733789115;
    // Default acceleration for Cool Muscle (rad/s^2)
    static constexpr double DEFAULT_A = 0.02161929667236394;
    // Default torque for Cool Muscle (percentage)
    static const int DEFAULT_M = 20;

    /* Constructors, Destructor, and Assignment operators {{{ */
    ComsSteering(const std::string& topic_prefix);

    // Copy constructor
    ComsSteering(const ComsSteering& other) = delete;

    // Move constructor
    ComsSteering(ComsSteering&& other) = delete;

    // Destructor
    ~ComsSteering() = default;

    // Assignment operator
    ComsSteering&
    operator=(const ComsSteering& other) = delete;

    // Move assignment operator
    ComsSteering&
    operator=(ComsSteering&& other) = delete;
    /* }}} */

    void
    steer_callback(const coms_msgs::Steering& msg);

    bool
    enable_callback(std_srvs::SetBoolRequest& req,
                    std_srvs::SetBoolResponse& res);

    void
    connect();

    void
    init();

    void
    on();

    void
    off();

    /**
     * Rotates the steering wheel to an angle at the given angular velocity
     *
     * 0 rad is when the steering is at the origin (straight). Limits in the
     * CW and CCW direction is determined by the limit_plus_ccw and
     * limit_minus_cw member variables.
     *
     * Also note that positive angles are in the CCW direction, and negative
     * angles are in the CW direction. Sign of the angular velocity is
     * ignored.
     *
     * Unlike ComsSteering::set_block, which blocks execution until the
     * target angle is reached, this method returns as soon as it sends the
     * command to the actuator.
     *
     * \param[in] ang the target angle in radians
     *
     * \param[in] ang_vel the target angular velocity in rad/s
     *
     * \param[in] ang_acc the target angular acceleration in rad/s^2
     */
    void
    set(const double ang,
        const double ang_vel = DEFAULT_S,
        const double ang_acc = DEFAULT_A);

    /**
     * Rotates the steering wheel to an angle at the given angular velocity
     *
     * Same as ComsSteering::set except that this method blocks
     * execution until the target angle is reached.
     *
     * \param[in] ang the target angle in radians
     *
     * \param[in] ang_vel the target angular velocity in rad/s
     *
     * \param[in] ang_acc the target angular acceleration in rad/s^2
     */
    void
    set_block(const double ang,
              const double ang_vel = DEFAULT_S,
              const double ang_acc = DEFAULT_A);

    /**
     * Sends the emergency stop command to the actuator
     */
    void
    emergency();

    /**
     * Sends the release emergency command to the actuator
     */
    void
    release_emergency();

    /**
     * Returns the raw pulse count obtained from the actuator
     */
    pulse_t
    get_pulse_count();

    /**
     * Returns the current angle of the steering wheel in radians
     *
     * 0 is when the steering wheel is at the center, positive angles are in
     * the CCW direction, and negative angles are in the CW direction.
     */
    double
    get_rad();

    /**
     * Sets the port to communicate through
     */
    void
    set_port(const std::string& port);

    /**
     * Sets the baud rate for the Cool Muscle actuator
     */
    void
    set_baud(const unsigned baud);

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_p;
    ros::Subscriber steering_sub;
    ros::ServiceServer enable_service;

    // Cool Muscle actuator for braking
    serial::Serial to_cool_muscle;

    // Rotation limits in [radians, pulse counts]
    std::pair<double, pulse_t> limit_plus_ccw;
    std::pair<double, pulse_t> limit_minus_cw;

    // Offset distance between mechanical and electrical origins
    pulse_t origin_offset;

    // Steering wheel rotation is CCW
    int is_ccw;

    /**
     * Converts pulse count to radians
     */
    double
    pulse2rad(const pulse_t pulse);

    /**
     * Converts radians to pulse counts
     */
    pulse_t
    rad2pulse(const double rad);

    /**
     * Checks whether the rotation direction of the wheel has changed.
     * @param is_ccw
     * @param tgt_ang
     * @param cur_ang
     * @return true if it has changed, false otherwise
     */
    bool
    direction_changed(bool is_ccw, double tgt_ang, double cur_ang);
    /**
     * Writes a line to the cool muscle after appending a CRLF
     */
    void
    write_line(const std::string& line);

    /**
     * Writes a line to the cool muscle after appending a CRLF
     *
     * The line written is in the format: "{line}{val}\r\n"
     */
    template<typename T>
    inline
    void
    write_line(const std::string& line, const T val) {
        write_line(line + std::to_string(val));
    }

    /**
     * Reads one line from the cool muscle
     *
     * Given an argument, populates that variable according to the format.
     * This method loops and keeps on reading line-by-line until a string with
     * the given format is matched and the variable is populated.
     *
     * \param[in] fmt format of the line. Same as what you give to scanf
     *
     * \param[in] var variable to be populated
     *
     * \param[in] allow_empty whether the data could be an empty string. In
     *                        this case, exactly one line is read regardless
     *                        of whether it matches the given format. If the
     *                        format matches, var is populated.
     */
    template<typename T>
    void
    read_line(const std::string& fmt, T& var, const bool allow_empty = false);
};

#endif /* end of include guard */
