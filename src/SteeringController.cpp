#include "coms_steering/SteeringController.h"

using pulse_t = SteeringController::pulse_t;

bool SteeringController::abort = false;

/* Constructors, Destructor, and Assignment operators {{{ */
SteeringController::SteeringController(const std::string& port,
                                       const unsigned baudrate,
                                       const std::pair<double, pulse_t>& limit_ccw,
                                       const std::pair<double, pulse_t>& limit_cw,
                                       const pulse_t origin_offset /* = 0 */)
    : limit_plus_ccw{limit_ccw}
    , limit_minus_cw{limit_cw}
    , origin_offset{origin_offset}
{
    to_cool_muscle.setPort(port);
    to_cool_muscle.setBaudrate(baudrate);
    to_cool_muscle.setTimeout(serial::Timeout::max(),
                              READ_WRITE_TIMEOUT_MS, 0,
                              READ_WRITE_TIMEOUT_MS, 0);
}

SteeringController::~SteeringController()
{
}
/* }}} */

void
SteeringController::connect() {
    if (to_cool_muscle.isOpen()) {
        return;
    }

    to_cool_muscle.open();
}

void
SteeringController::init() {
    int response_code;

    // Enable motor
    on();

    // Send in-position status, in/out change status, disable echo
    writeline("K23.1=", 0b01111);
    // Origin search speed
    writeline("K42.1=10");
    // Origin search acceleration
    writeline("K43.1=10");
    // Set unit of software limit and offset to 1 pulse unit, CW rotation
    writeline("K45.1=222");
    // Use origin sensor
    writeline("K46.1=2");
    // Offset distance between mechanical and electrical origins
    writeline("K48.1=", origin_offset);

    // Initiate origin search
    writeline("|.1");
    // Wait for origin search to complete (response should be 8)
    readline("Ux.1=%d", response_code);

    // Set soft limit (for the + side == CCW direction)
    writeline("K58=", limit_plus_ccw.second);
    // Set soft limit (for the - side == CW direction)
    writeline("K59=", limit_minus_cw.second);

    // Disable motor
    off();
}

void
SteeringController::on() {
    writeline("(.1");
    int response_code;
    // Data could be empty when already on
    readline("Ux.1=%d", response_code, true);
    // TODO: check if response_code was 8
}

void
SteeringController::off() {
    writeline(").1");
}

void
SteeringController::set(const double ang,
                        const double ang_vel /* = DEFAULT_S */,
                        const double ang_acc /* = DEFAULT_A */) {
    int response_code;
    // Position
    writeline("P.1=", rad2pulse(ang));
    // Speed
    writeline("S.1=", rad2pulse(ang_vel));
    // Acceleration
    writeline("A.1=", rad2pulse(ang_acc));
    // Torque limit
    writeline("M.1=", DEFAULT_M);
    // Go!
    writeline("^.1");
    // Status should be 8
    readline("Ux.1=%d", response_code);
}

void
SteeringController::emergency() {
    writeline("*");
}

void
SteeringController::release_emergency() {
    writeline("*1");
}

pulse_t
SteeringController::get_pulse_count() {
    writeline("?96.1");
    pulse_t pulse;
    readline("Px.1=%lld", pulse);
    return pulse;
}

double
SteeringController::get_rad() {
    return pulse2rad(get_pulse_count());
}

void
SteeringController::set_port(const std::string& port) {
    to_cool_muscle.setPort(port);
}

void
SteeringController::set_baudrate(const unsigned baudrate) {
    to_cool_muscle.setBaudrate(baudrate);
}

double
SteeringController::pulse2rad(const pulse_t pulse) {
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
SteeringController::rad2pulse(const double rad) {
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
    return rad * pulse_per_rad;
}

void
SteeringController::writeline(const std::string& line) {
    to_cool_muscle.write(line + "\r\n");
}

template<typename T>
void
SteeringController::readline(const std::string& fmt,
                             T& var,
                             const bool allow_empty /* = false */) {
    unsigned read_tokens = 0;
    while (read_tokens == 0 && !SteeringController::abort) {
        // Read a line, and then parse it
        std::string line;
        while (line.empty() && !SteeringController::abort) {
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
