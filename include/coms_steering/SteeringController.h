#ifndef BRAKE_CONTROLLER_H_
#define BRAKE_CONTROLLER_H_

#include <ros/ros.h>
#include <serial/serial.h>

#include <cstdio>
#include <string>
#include <utility>

class SteeringController {
public:
    using pulse_t = long long int;

    static const unsigned READ_WRITE_TIMEOUT_MS = 250;
    // Flag for aborting program
    static bool abort;

    // Default speed for Cool Muscle (rad/s)
    static constexpr double DEFAULT_S = 0.01729543733789115;
    // Default acceleration for Cool Muscle (rad/s^2)
    static constexpr double DEFAULT_A = 0.02161929667236394;
    // Default torque for Cool Muscle (percentage)
    static const int DEFAULT_M = 20;

    // Rotate infinitely
    static const int CONTINUOUS_ROTATION = 1000000000;

    /* Constructors, Destructor, and Assignment operators {{{ */
    SteeringController(const std::string& port,
                       const unsigned baud,
                       const std::pair<double, pulse_t>& limit_ccw,
                       const std::pair<double, pulse_t>& limit_cw,
                       const pulse_t origin_offset = 0);

    // Copy constructor
    SteeringController(const SteeringController& other) = delete;

    // Move constructor
    SteeringController(SteeringController&& other) = delete;

    // Destructor
    ~SteeringController() = default;

    // Assignment operator
    SteeringController&
    operator=(const SteeringController& other) = delete;

    // Move assignment operator
    SteeringController&
    operator=(SteeringController&& other) = delete;
    /* }}} */

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
     * Unlike SteeringController::set_block, which blocks execution until the
     * target angle is reached, this method returns as soon as it sends the
     * command to the actuator.
     *
     * Note that even though this method does not block execution, some
     * steering wheels, such as the one that we use, doesn't accept new
     * target angles until it finishes the current commanded angle. For
     * that, use set_target_angle, which does a PID control on the angular
     * velocity, which can be adjusted while the actuator is moving.
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
     * Same as SteeringController::set except that this method blocks
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
     * Sets the target angle for the PID control
     *
     * Uses PID control to reach the target angle.
     *
     * @param ang the target angle in radians
     */
    void
    set_target_angle(const double ang);

    /**
     * Starts the PID control for the steering wheel angle
     *
     * @param KP
     * @param KI
     * @param KD
     */
    void
    start_control_loop(const double KP,
                       const double KI,
                       const double KD,
                       const double control_rate);

    /**
     * Stops the PID control for the steering wheel angle
     */
    void
    stop_control_loop();

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
    // Cool Muscle actuator for braking
    serial::Serial to_cool_muscle;

    // Rotation limits in [radians, pulse counts]
    std::pair<double, pulse_t> limit_plus_ccw;
    std::pair<double, pulse_t> limit_minus_cw;

    // Offset distance between mechanical and electrical origins
    pulse_t origin_offset;

    bool stop_control;
    // Target angle
    double ang_tgt;
    // For calculating the integral term
    double ang_err_sum;
    // For calculating the derivative term
    double ang_err_prev;

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
