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
bool offb_ctrl_ned(mavsdk::Offboard &offboard) {
  std::cout << "Starting Offboard velocity control in NED coordinates\n";

  // Send it once before starting offboard, otherwise it will be rejected.
  const Offboard::VelocityNedYaw stay{};
  offboard.set_velocity_ned(stay);

  Offboard::Result offboard_result = offboard.start();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard start failed: " << offboard_result << '\n';
    return false;
  }

  std::cout << "Offboard started\n";
  std::cout << "Turn to face East\n";

  Offboard::VelocityNedYaw turn_east{};
  turn_east.yaw_deg = 90.0f;
  offboard.set_velocity_ned(turn_east);
  sleep_for(seconds(1)); // Let yaw settle.

  {
    const float step_size = 0.01f;
    const float one_cycle = 2.0f * (float)M_PI;
    const unsigned steps = 2 * unsigned(one_cycle / step_size);

    std::cout << "Go North and back South\n";

    for (unsigned i = 0; i < steps; ++i) {
      float vx = 5.0f * sinf(i * step_size);
      Offboard::VelocityNedYaw north_and_back_south{};
      north_and_back_south.north_m_s = vx;
      north_and_back_south.yaw_deg = 90.0f;
      offboard.set_velocity_ned(north_and_back_south);
      sleep_for(milliseconds(10));
    }
  }

  std::cout << "Turn to face West\n";
  Offboard::VelocityNedYaw turn_west{};
  turn_west.yaw_deg = 270.0f;
  offboard.set_velocity_ned(turn_west);
  sleep_for(seconds(2));

  std::cout << "Go up 2 m/s, turn to face South\n";
  Offboard::VelocityNedYaw up_and_south{};
  up_and_south.down_m_s = -2.0f;
  up_and_south.yaw_deg = 180.0f;
  offboard.set_velocity_ned(up_and_south);
  sleep_for(seconds(4));

  std::cout << "Go down 1 m/s, turn to face North\n";
  Offboard::VelocityNedYaw down_and_north{};
  up_and_south.down_m_s = 1.0f;
  offboard.set_velocity_ned(down_and_north);
  sleep_for(seconds(4));

  offboard_result = offboard.stop();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard stop failed: " << offboard_result << '\n';
    return false;
  }
  std::cout << "Offboard stopped\n";

  return true;
}

//
// Does Offboard control using body co-ordinates.
// Body coordinates really means world coordinates rotated by the yaw of the
// vehicle, so if the vehicle pitches down, the forward axis does still point
// forward and not down into the ground.
//
// returns true if everything went well in Offboard control.
//
bool offb_ctrl_body(mavsdk::Offboard &offboard) {
  std::cout << "Starting Offboard velocity control in body coordinates\n";

  // Send it once before starting offboard, otherwise it will be rejected.
  Offboard::VelocityBodyYawspeed stay{};
  offboard.set_velocity_body(stay);

  Offboard::Result offboard_result = offboard.start();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard start failed: " << offboard_result << '\n';
    return false;
  }
  std::cout << "Offboard started\n";

  std::cout << "Turn clock-wise and climb\n";
  Offboard::VelocityBodyYawspeed cc_and_climb{};
  cc_and_climb.down_m_s = -1.0f;
  cc_and_climb.yawspeed_deg_s = 60.0f;
  offboard.set_velocity_body(cc_and_climb);
  sleep_for(seconds(5));

  std::cout << "Turn back anti-clockwise\n";
  Offboard::VelocityBodyYawspeed ccw{};
  ccw.down_m_s = -1.0f;
  ccw.yawspeed_deg_s = -60.0f;
  offboard.set_velocity_body(ccw);
  sleep_for(seconds(5));

  std::cout << "Wait for a bit\n";
  offboard.set_velocity_body(stay);
  sleep_for(seconds(2));

  std::cout << "Fly a circle\n";
  Offboard::VelocityBodyYawspeed circle{};
  circle.forward_m_s = 5.0f;
  circle.yawspeed_deg_s = 30.0f;
  offboard.set_velocity_body(circle);
  sleep_for(seconds(15));

  std::cout << "Wait for a bit\n";
  offboard.set_velocity_body(stay);
  sleep_for(seconds(5));

  std::cout << "Fly a circle sideways\n";
  circle.right_m_s = -5.0f;
  circle.yawspeed_deg_s = 30.0f;
  offboard.set_velocity_body(circle);
  sleep_for(seconds(15));

  std::cout << "Wait for a bit\n";
  offboard.set_velocity_body(stay);
  sleep_for(seconds(8));

  offboard_result = offboard.stop();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard stop failed: " << offboard_result << '\n';
    return false;
  }
  std::cout << "Offboard stopped\n";

  return true;
}

//
// Does Offboard control using attitude commands.
//
// returns true if everything went well in Offboard control.
//
bool offb_ctrl_attitude(mavsdk::Offboard &offboard) {
  std::cout << "Starting Offboard attitude control\n";

  // Send it once before starting offboard, otherwise it will be rejected.
  Offboard::Attitude roll{};
  roll.roll_deg = 30.0f;
  roll.thrust_value = 0.6f;
  offboard.set_attitude(roll);

  Offboard::Result offboard_result = offboard.start();
  if (offboard_result != Offboard::Result::Success) {
    std::cerr << "Offboard start failed: " << offboard_result << '\n';
    return false;
  }
  std::cout << "Offboard started\n";

  std::cout << "Roll 30 degrees to the right\n";
  offboard.set_attitude(roll);
  sleep_for(seconds(2));

  std::cout << "Stay horizontal\n";
  roll.roll_deg = 0.0f;
  offboard.set_attitude(roll);
  sleep_for(seconds(1));

  std::cout << "Roll 30 degrees to the left\n";
  roll.roll_deg = -30.0f;
  offboard.set_attitude(roll);
  sleep_for(seconds(2));

  std::cout << "Stay horizontal\n";
  roll.roll_deg = 0.0f;
  offboard.set_attitude(roll);
  sleep_for(seconds(2));

  // offboard_result = offboard.stop();
  // if (offboard_result != Offboard::Result::Success) {
  //   std::cerr << "Offboard stop failed: " << offboard_result << '\n';
  //   return false;
  // }
  // std::cout << "Offboard stopped\n";

  return true;
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
  auto action = Action{system};
  auto offboard = Offboard{system};
  auto telemetry = Telemetry{system};

  // while (!telemetry.health_all_ok()) {
  //   std::cout << "Waiting for system to be ready\n";
  //   sleep_for(seconds(1));
  // }
  std::cout << "System is ready\n";

  const auto arm_result = action.arm();
  if (arm_result != Action::Result::Success) {
    std::cerr << "Arming failed: " << arm_result << '\n';
    return 1;
  }
  std::cout << "Armed\n";

  const auto takeoff_result = action.takeoff();
  if (takeoff_result != Action::Result::Success) {
    std::cerr << "Takeoff failed: " << takeoff_result << '\n';
    return 1;
  }

  auto in_air_promise = std::promise<void>{};
  auto in_air_future = in_air_promise.get_future();
  telemetry.subscribe_landed_state(
      [&telemetry, &in_air_promise](Telemetry::LandedState state) {
        if (state == Telemetry::LandedState::InAir) {
          std::cout << "Taking off has finished\n.";
          telemetry.subscribe_landed_state(nullptr);
          in_air_promise.set_value();
        }
      });
  in_air_future.wait_for(seconds(10));
  if (in_air_future.wait_for(seconds(3)) == std::future_status::timeout) {
    std::cerr << "Takeoff timed out.\n";
    return 1;
  }

  //  using attitude control
  if (!offb_ctrl_attitude(offboard)) {
    return 1;
  }

  // //  using local NED co-ordinates
  // if (!offb_ctrl_ned(offboard)) {
  //   return 1;
  // }

  // //  using body co-ordinates
  // if (!offb_ctrl_body(offboard)) {
  //   return 1;
  // }

  const auto land_result = action.land();
  if (land_result != Action::Result::Success) {
    std::cerr << "Landing failed: " << land_result << '\n';
    return 1;
  }

  // Check if vehicle is still in air
  while (telemetry.in_air()) {
    std::cout << "Vehicle is landing...\n";
    sleep_for(seconds(1));
  }
  std::cout << "Landed!\n";

  // We are relying on auto-disarming but let's keep watching the telemetry for
  // a bit longer.
  sleep_for(seconds(3));
  std::cout << "Finished...\n";

  return 0;
}