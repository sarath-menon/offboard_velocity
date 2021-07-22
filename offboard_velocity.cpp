/**
 * @file offboard_velocity.cpp
 * @brief Example that demonstrates offboard velocity control in local NED and
 * body coordinates
 *
 * @authors Author: Julian Oes <julian@oes.ch>,
 *                  Shakthi Prashanth <shakthi.prashanth.m@intel.com>
 */

#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
// #define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

// Handles Action's result
inline void action_error_exit(Action::Result result, const std::string& message)
{
    if (result != Action::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles Offboard's result
inline void offboard_error_exit(Offboard::Result result, const std::string& message)
{
    if (result != Offboard::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles connection result
inline void connection_error_exit(ConnectionResult result, const std::string& message)
{
    if (result != ConnectionResult::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Logs during Offboard control
inline void offboard_log(const std::string& offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}

/**
 * Does Offboard control using attitude commands.
 *
 * returns true if everything went well in Offboard control, exits with a log
 * otherwise.
 */
bool offb_ctrl_attitude(mavsdk::Offboard& offboard)
{
    const std::string offb_mode = "ATTITUDE";

    // Send it once before starting offboard, otherwise it will be rejected.
    Offboard::Attitude roll{};
    Offboard::Attitude thrust_value{};

    roll.roll_deg = 30.0f;
    roll.thrust_value = 0.6f;
    offboard.set_attitude(roll);

    Offboard::Result offboard_result = offboard.start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    offboard_log(offb_mode, "Thrust 0.1");
    thrust_value.thrust_value = 0.1f;
    offboard.set_attitude(thrust_value);
    sleep_for(seconds(2)); // Let yaw settle.

    // offboard_log(offb_mode, "Thrust 0.2");
    // thrust_value.thrust_value = 0.2f;
    // offboard.set_attitude(thrust_value);
    // sleep_for(seconds(2)); // Let yaw settle.

    // offboard_log(offb_mode, "Thrust 0.3");
    // thrust_value.thrust_value = 0.3f;
    // offboard.set_attitude(thrust_value);
    // sleep_for(seconds(2)); // Let yaw settle.

    Offboard::Attitude attitude;
    attitude.thrust_value = 0.3f;
    attitude.roll_deg = 0.3f;
    attitude.pitch_deg = 0.0f;
    attitude.yaw_deg = 0.0f;
    offboard.set_attitude(attitude);
    sleep_for(seconds(5)); // Let yaw settle.

    offboard_log(offb_mode, "Thrust 0.5");
    thrust_value.thrust_value = 0.5f;
    offboard.set_attitude(thrust_value);
    sleep_for(seconds(5)); // Let yaw settle.

    offboard_log(offb_mode, "ROLL 30");
    offboard.set_attitude(roll);
    sleep_for(seconds(5)); // rolling

    offboard_log(offb_mode, "ROLL -30");
    roll.roll_deg = -30.0f;
    offboard.set_attitude(roll);
    sleep_for(seconds(5)); // Let yaw settle.

    offboard_log(offb_mode, "ROLL 0");
    roll.roll_deg = 0.0f;
    offboard.set_attitude(roll);
    sleep_for(seconds(2)); // Let yaw settle.

    // offboard_log(offb_mode, "Thrust 0.1");
    // thrust_value.thrust_value = 0.1f;
    // offboard.set_attitude(thrust_value);
    // sleep_for(seconds(2)); // Let yaw settle.

    // Offboard::PositionNedYaw position_ned_yaw;

    // position_ned_yaw.north_m = 0.1;
    // position_ned_yaw.down_m = 0;
    // position_ned_yaw.east_m = 0;
    // position_ned_yaw.yaw_deg = 0;

    // offboard.set_position_ned(position_ned_yaw);
    // sleep_for(seconds(2)); // Let yaw settle.

    offboard.set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
    sleep_for(seconds(2)); // Let yaw settle.

    // Now, stop offboard mode.
    // offboard_result = offboard.stop();
    // offboard_error_exit(offboard_result, "Offboard stop failed: ");
    // offboard_log(offb_mode, "Offboard stopped");

    return true;
}

void wait_until_discover(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system..." << std::endl;
    std::promise<void> discover_promise;
    auto discover_future = discover_promise.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk, &discover_promise]() {
        const auto system = mavsdk.systems().at(0);

        if (system->is_connected()) {
            std::cout << "Discovered system" << std::endl;
            discover_promise.set_value();
        }
    });

    discover_future.wait();
}

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

int main(int argc, char** argv)
{
    Mavsdk mavsdk;
    std::string connection_url;
    ConnectionResult connection_result;

    if (argc == 2) {
        connection_url = argv[1];
        connection_result = mavsdk.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Wait for the system to connect via heartbeat
    wait_until_discover(mavsdk);

    // System got discovered.
    auto system = mavsdk.systems().at(0);
    auto action = Action{system};
    auto offboard = Offboard{system};
    auto telemetry = Telemetry{system};
    auto mocap = Mocap{system};

    // Send mocap command to Mavsdk
    Mocap::AttitudePositionMocap mocap_msg;

    mocap_msg.position_body.x_m = 0;
    mocap_msg.position_body.y_m = 0;
    mocap_msg.position_body.z_m = 0;

    mocap_msg.q.w = 1;
    mocap_msg.q.x = 0;
    mocap_msg.q.y = 0;
    mocap_msg.q.z = 0;

    // Mocap Covariance Matrix
    std::vector<int> v = {1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1};
    std::cout << "Size:" << size(mocap_msg.pose_covariance.covariance_matrix);
    mocap_msg.pose_covariance.covariance_matrix.insert(
        mocap_msg.pose_covariance.covariance_matrix.end(), std::begin(v), std::end(v));

    Mocap::Result result = mocap.set_attitude_position_mocap(mocap_msg);
    std::cout << "Mocap Status: " << result << std::endl;

    std::cout << "System is ready" << std::endl;
    std::cout << Mocap::AngularVelocityBody();
    std::cout << Mocap::SpeedBody();

    // Arm vehicle
    std::cout << "Arming..." << std::endl;
    const Action::Result arm_result = action.arm();

    if (arm_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Arming failed:" << arm_result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }

    //  using attitude control
    bool ret = offb_ctrl_attitude(offboard);
    if (ret == false) {
        return EXIT_FAILURE;
    }

    std::cout << "Killing Motors";
    action.kill();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    return EXIT_SUCCESS;
}
