// Example that demonstrates offboard control using attitude, velocity control
// in NED (North-East-Down), and velocity control in body (Forward-Right-Down)
// coordinates.
//

#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

void usage(const std::string &bin_name) {
  std::cerr
      << "Usage : " << bin_name << " <connection_url>\n"
      << "Connection URL format should be :\n"
      << " For TCP : tcp://[server_host][:server_port]\n"
      << " For UDP : udp://[bind_host][:bind_port]\n"
      << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
      << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk &mavsdk) {
  std::cout << "Waiting to discover system...\n";
  auto prom = std::promise<std::shared_ptr<System>>{};
  auto fut = prom.get_future();

  // We wait for new systems to be discovered, once we find one that has an
  // autopilot, we decide to use it.
  mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
    auto system = mavsdk.systems().back();

    if (system->has_autopilot()) {
      std::cout << "Discovered autopilot\n";

      // Unsubscribe again as we only want to find one system.
      mavsdk.subscribe_on_new_system(nullptr);
      prom.set_value(system);
    }
  });

  // We usually receive heartbeats at 1Hz, therefore we should find a
  // system after around 3 seconds max, surely.
  if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
    std::cerr << "No autopilot found.\n";
    return {};
  }

  // Get discovered system now.
  return fut.get();
}

// Check telemetry health
void print_health(Telemetry::Health health) {
  std::cout << "Got health: " << '\n';

  std::cout << "Gyro calibration:  "
            << (health.is_gyrometer_calibration_ok ? "ok" : "not ok") << '\n';
  std::cout << "Accel calibration: "
            << (health.is_accelerometer_calibration_ok ? "ok" : "not ok")
            << '\n';
  std::cout << "Mag calibration:   "
            << (health.is_magnetometer_calibration_ok ? "ok" : "not ok")
            << '\n';
  std::cout << "Local position:    "
            << (health.is_local_position_ok ? "ok" : "not ok") << '\n';
  std::cout << "Global position:   "
            << (health.is_global_position_ok ? "ok" : "not ok") << '\n';
  std::cout << "Home position:     "
            << (health.is_home_position_ok ? "ok" : "not ok") << '\n';
}

void print_rc_status(Telemetry::RcStatus rc_status) {
  std::cout << "RC available: " << (rc_status.is_available ? "yes" : "no")
            << '\n';
  std::cout << "RC available once: "
            << (rc_status.was_available_once ? "yes" : "no") << '\n';
  std::cout << "RC RSSI: " << rc_status.signal_strength_percent << '\n';
}

int main(int argc, char **argv) {
  if (argc != 2) {
    usage(argv[0]);
    return 1;
  }

  Mavsdk mavsdk;
  ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

  if (connection_result != ConnectionResult::Success) {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }

  auto system = get_system(mavsdk);
  if (!system) {
    return 1;
  }

  // Instantiate plugins.
  auto telemetry = Telemetry{system};
  auto mocap = Mocap{system};
  std::cout << "System is ready\n";
  sleep_for(seconds(1));

  // // // Send mocap command to Mavsdk
  // Mocap::AttitudePositionMocap mocap_msg;

  // mocap_msg.time_usec = 0;

  // mocap_msg.position_body.x_m = 0;
  // mocap_msg.position_body.y_m = 0;
  // mocap_msg.position_body.z_m = 0;

  // mocap_msg.q.w = 0.737812757492065;
  // mocap_msg.q.x = -0.00313977641053498;
  // mocap_msg.q.y = -0.00384858623147011;
  // mocap_msg.q.z = 0.674987196922302;

  // std::vector<float> v = {NAN};

  // std::cout << "Size:" << size(mocap_msg.pose_covariance.covariance_matrix);
  // mocap_msg.pose_covariance.covariance_matrix.insert(
  //     mocap_msg.pose_covariance.covariance_matrix.end(), std::begin(v),
  //     std::end(v));

  // mocap_msg.pose_covariance.covariance_matrix = v;

  // // Set posiiton from mocap
  // for (int i = 0; i < 100; i++) {
  //   Mocap::Result result = mocap.set_attitude_position_mocap(mocap_msg);
  //   std::cout << "Position sent" << result << std::endl;
  //   sleep_for(std::chrono::milliseconds(50));
  // }

  // Subscribe telemetry health
  telemetry.subscribe_health(std::bind(&print_health, std::placeholders::_1));
  telemetry.subscribe_rc_status(
      std::bind(&print_rc_status, std::placeholders::_1));
  sleep_for(std::chrono::seconds(3));

  sleep_for(std::chrono::milliseconds(500));

  // We are relying on auto-disarming but let's keep watching the telemetry for
  // a bit longer.
  sleep_for(seconds(1));
  std::cout << "Finished...\n";

  return 0;
}