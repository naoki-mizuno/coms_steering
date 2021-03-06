#include "coms_steering/coms_steering.h"

using pulse_t = ComsSteering::pulse_t;

ComsSteering::ComsSteering(const std::string& topic_prefix)
    : nh{}
    , nh_p{"~"}
    , is_ccw{false}
{
    // Get parameters
    std::string port;
    int baud;
    int origin_offset;
    // Limits [rad, pulse count] later converted to [double, pulse_t]
    std::vector<double> limit_ccw;
    std::vector<double> limit_cw;

    nh_p.getParam("port", port);
    nh_p.param("baud", baud, 38400);
    nh_p.param("origin_offset", origin_offset, 0);
    nh_p.getParam("limit_ccw", limit_ccw);
    nh_p.getParam("limit_cw", limit_cw);

    // Convert to [double, pulse_t]
    auto lim_rad_ccw = static_cast<double>(limit_ccw[0]);
    auto lim_rad_cw = static_cast<double>(limit_cw[0]);
    auto lim_pulse_ccw = static_cast<ComsSteering::pulse_t>(limit_ccw[1]);
    auto lim_pulse_cw = static_cast<ComsSteering::pulse_t>(limit_cw[1]);
    // Limit [rad, pulse count] for CCW direction
    limit_plus_ccw = std::make_pair(lim_rad_ccw, lim_pulse_ccw);
    // Limit [rad, pulse count] for CW direction
    limit_minus_cw = std::make_pair(lim_rad_cw, lim_pulse_cw);

    // Serial port configurations
    to_cool_muscle.setPort(port);
    to_cool_muscle.setBaudrate(baud);
    to_cool_muscle.setTimeout(serial::Timeout::max(),
                              READ_WRITE_TIMEOUT_MS, 0,
                              READ_WRITE_TIMEOUT_MS, 0);

    steering_sub = nh.subscribe(topic_prefix + "cmd_steer",
                                1,
                                &ComsSteering::steer_callback,
                                this);
    enable_service = nh.advertiseService(topic_prefix + "enable",
                                         &ComsSteering::enable_callback,
                                         this);
}

void
ComsSteering::steer_callback(const coms_msgs::Steering& msg) {
    // Position, speed, and acceleration in radians
    set(msg.pos, msg.vel, msg.acc);
}

bool
ComsSteering::enable_callback(std_srvs::SetBoolRequest& req,
                              std_srvs::SetBoolResponse& res) {
    std::string msg = "Steering wheel actuator turned ";
    if (req.data) {
        on();
        msg += "on";
    }
    else {
        off();
        msg += "off";
    }

    res.success = true;
    res.message = msg;

    return true;
}

void
ComsSteering::connect() {
    if (to_cool_muscle.isOpen()) {
        return;
    }

    to_cool_muscle.open();
}

void
ComsSteering::init() {
    int response_code;

    // Enable motor
    on();

    // Send in-position status, in/out change status, disable echo
    write_line("K23.1=", 0b01111);
    // Origin search speed
    write_line("K42.1=10");
    // Origin search acceleration
    write_line("K43.1=10");
    // Set unit of software limit and offset to 1 pulse unit, CW rotation
    write_line("K45.1=222");
    // Use origin sensor
    write_line("K46.1=2");
    // Offset distance between mechanical and electrical origins
    write_line("K48.1=", origin_offset);
    // Set motor to free when powered on
    write_line("K68.1=0");

    // Initiate origin search
    write_line("|.1");
    // Wait for origin search to complete (response should be 8)
    read_line("Ux.1=%d", response_code);

    // Set soft limit (for the + side == CCW direction)
    write_line("K58=", limit_plus_ccw.second);
    // Set soft limit (for the - side == CW direction)
    write_line("K59=", limit_minus_cw.second);

    // Disable motor
    off();
}

void
ComsSteering::on() {
    write_line("(.1");
    int response_code;
    // Data could be empty when already on
    read_line("Ux.1=%d", response_code, true);
    // TODO: check if response_code was 8
}

void
ComsSteering::off() {
    write_line(").1");
}

void
ComsSteering::set(const double ang,
                  const double ang_vel /* = DEFAULT_S */,
                  const double ang_acc /* = DEFAULT_A */) {
    if (direction_changed(is_ccw, ang, get_rad())) {
        // Stop motor
        write_line("].1");
    }
    is_ccw = (ang - get_rad()) > 0;
    // Software limit to min/max angle
    auto cmd_ang = ang;
    cmd_ang = ang > limit_plus_ccw.first ? limit_plus_ccw.first : cmd_ang;
    cmd_ang = ang < limit_minus_cw.first ? limit_minus_cw.first : cmd_ang;
    // Prevent hang
    auto cmd_ang_vel = ang_vel;
    cmd_ang_vel = cmd_ang_vel == 0 ? DEFAULT_S : cmd_ang_vel;
    auto cmd_ang_acc = ang_acc;
    cmd_ang_acc = cmd_ang_acc == 0 ? DEFAULT_A : cmd_ang_acc;

    // Position
    write_line("P.1=", rad2pulse(cmd_ang));
    // Speed
    write_line("S.1=", rad2pulse(cmd_ang_vel));
    // Acceleration
    write_line("A.1=", rad2pulse(cmd_ang_acc));
    // Torque limit
    write_line("M.1=", DEFAULT_M);
    // Go!
    write_line("^.1");
}

void
ComsSteering::set_block(const double ang,
                        const double ang_vel /* = DEFAULT_S */,
                        const double ang_acc /* = DEFAULT_A */) {
    set(ang, ang_vel, ang_acc);
    // Status should be 8
    int response_code;
    read_line("Ux.1=%d", response_code);
}

void
ComsSteering::emergency() {
    write_line("*");
}

void
ComsSteering::release_emergency() {
    write_line("*1");
}

pulse_t
ComsSteering::get_pulse_count() {
    write_line("?96.1");
    pulse_t pulse;
    read_line("Px.1=%lld", pulse);
    return pulse;
}

double
ComsSteering::get_rad() {
    return pulse2rad(get_pulse_count());
}

void
ComsSteering::set_port(const std::string& port) {
    to_cool_muscle.setPort(port);
}

void
ComsSteering::set_baud(const unsigned baud) {
    to_cool_muscle.setBaudrate(baud);
}

double
ComsSteering::pulse2rad(const pulse_t pulse) {
    double max_rad;
    pulse_t max_pulse;
    if (pulse > 0) {
        // CCW
        max_rad = limit_plus_ccw.first;
        max_pulse = limit_plus_ccw.second;
    }
    else {
        // CW
        max_rad = limit_minus_cw.first;
        max_pulse = limit_minus_cw.second;
    }
    auto rad_per_pulse = max_rad / max_pulse;
    return pulse * rad_per_pulse;
}

pulse_t
ComsSteering::rad2pulse(const double rad) {
    double max_rad;
    pulse_t max_pulse;
    if (rad > 0) {
        // CCW
        max_rad = limit_plus_ccw.first;
        max_pulse = limit_plus_ccw.second;
    }
    else {
        // CW
        max_rad = limit_minus_cw.first;
        max_pulse = limit_minus_cw.second;
    }
    auto pulse_per_rad = max_pulse / max_rad;
    return static_cast<pulse_t>(rad * pulse_per_rad);
}

bool
ComsSteering::direction_changed(bool is_ccw,
                                double tgt_ang,
                                double cur_ang) {
    if (is_ccw && tgt_ang - cur_ang < 0) {
        // Rotating CCW, commanded to turn CW
        return true;
    }
    else if (!is_ccw && tgt_ang - cur_ang > 0) {
        // Rotating CW, commanded to turn CCW
        return true;
    }
    return false;
}

void
ComsSteering::write_line(const std::string& line) {
    to_cool_muscle.write(line + "\r\n");
}

template<typename T>
void
ComsSteering::read_line(const std::string& fmt,
                        T& var,
                        const bool allow_empty /* = false */) {
    int read_tokens = 0;
    while (read_tokens == 0) {
        // Read a line, and then parse it
        std::string line;
        while (line.empty()) {
            line = to_cool_muscle.readline();

            if (allow_empty) {
                break;
            }
        }

        // Trim CRLF
        while (line.back() == '\r' || line.back() == '\n') {
            line.pop_back();
        }

        read_tokens = sscanf(line.c_str(), fmt.c_str(), &var);

        if (allow_empty) {
            return;
        }
    }
}
