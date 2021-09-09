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

//
// Does Offboard control using NED co-ordinates.
//
// returns true if everything went well in Offboard control
//

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

  std::cout << "System is ready\n";

  for (unsigned i = 0; i < 10; ++i) {
    std::cout << "Position: " << telemetry.position() << '\n';
    std::cout << "Home Position: " << telemetry.home() << '\n';
    std::cout << "Attitude: " << telemetry.attitude_quaternion() << '\n';
    std::cout << "Attitude: " << telemetry.attitude_euler() << '\n';
    std::cout << "Angular velocity: "
              << telemetry.attitude_angular_velocity_body() << '\n';
    std::cout << "Fixed wing metrics: " << telemetry.fixedwing_metrics()
              << '\n';
    std::cout << "Ground Truth: " << telemetry.ground_truth() << '\n';
    std::cout << "Velocity: " << telemetry.velocity_ned() << '\n';
    std::cout << "GPS Info: " << telemetry.gps_info() << '\n';
    std::cout << "Battery: " << telemetry.battery() << '\n';
    std::cout << "Actuators: " << telemetry.actuator_control_target() << '\n';
    std::cout << "Flight mode: " << telemetry.flight_mode() << '\n';
    std::cout << "Landed state: " << telemetry.landed_state()
              << "(in air: " << telemetry.in_air() << ")" << '\n';

    sleep_for(std::chrono::milliseconds(500));
  }

  // We are relying on auto-disarming but let's keep watching the telemetry for
  // a bit longer.
  sleep_for(seconds(3));
  std::cout << "Finished...\n";

  return 0;
}