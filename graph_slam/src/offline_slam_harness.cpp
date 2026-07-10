// Offline synchronous SLAM harness -- dev tool for deterministic A/B (refactor plan Step 0
// fallback / "Option C"). Reads a rosbag, feeds the recorded inputs to the GraphSLAM core in
// STAMP ORDER, single-threaded: no ROS transport, no executor, no queues -> bit-identical
// output across runs and machines (fixes the record-order scramble that broke `ros2 bag play`
// replay). Logs the pose at every prediction/correction event to a CSV for scoring.
//
// Usage: graph_slam_offline <bag_dir> <out_csv> [sigma_v_frac sigma_theta_coeff slip_inflation]

#include "graph_slam/graph_slam.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <lart_msgs/msg/cone_array.hpp>
#include <lart_msgs/msg/dynamics.hpp>
#include <lart_msgs/msg/mission.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

static double stamp_sec(const builtin_interfaces::msg::Time & s)
{
  return static_cast<double>(s.sec) + static_cast<double>(s.nanosec) * 1e-9;
}

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "usage: graph_slam_offline <bag_dir> <out_csv> "
                 "[sigma_v_frac sigma_theta_coeff slip_inflation]\n";
    return 1;
  }
  rclcpp::init(argc, argv);  // for logging only; no node/executor is created

  const std::string bag_dir = argv[1];
  const std::string out_csv = argv[2];
  const double sigma_v_frac      = (argc > 3) ? std::stod(argv[3]) : 0.025;
  const double sigma_theta_coeff = (argc > 4) ? std::stod(argv[4]) : 1.22e-4;
  const double slip_inflation    = (argc > 5) ? std::stod(argv[5]) : 1.0;

  // ---- read the whole bag into typed, per-topic buffers ----
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_dir;
  storage_options.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  rosbag2_cpp::Reader reader;
  reader.open(storage_options, converter_options);

  rclcpp::Serialization<lart_msgs::msg::Dynamics> dyn_ser;
  rclcpp::Serialization<geometry_msgs::msg::Vector3Stamped> imu_ser;
  rclcpp::Serialization<lart_msgs::msg::ConeArray> cone_ser;
  rclcpp::Serialization<lart_msgs::msg::Mission> mission_ser;
  rclcpp::Serialization<rosgraph_msgs::msg::Clock> clock_ser;

  std::vector<std::pair<double, lart_msgs::msg::Dynamics::SharedPtr>> dyns;        // (sim stamp, msg)
  std::vector<std::pair<double, geometry_msgs::msg::Vector3Stamped::SharedPtr>> imus;
  std::vector<std::pair<double, lart_msgs::msg::ConeArray::SharedPtr>> cones;      // (recv->sim, msg)
  std::vector<std::pair<double, double>> clockmap;                                  // (recv_wall, sim)
  lart_msgs::msg::Mission::SharedPtr mission;

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    const std::string & topic = bag_msg->topic_name;
    const double recv_t = static_cast<double>(bag_msg->time_stamp) * 1e-9;
    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    if (topic == "/acu_origin/dynamics") {
      auto m = std::make_shared<lart_msgs::msg::Dynamics>();
      dyn_ser.deserialize_message(&serialized, m.get());
      dyns.emplace_back(stamp_sec(m->header.stamp), m);
    } else if (topic == "/imu/angular_velocity") {
      auto m = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
      imu_ser.deserialize_message(&serialized, m.get());
      imus.emplace_back(stamp_sec(m->header.stamp), m);
    } else if (topic == "/mapping/cones") {
      auto m = std::make_shared<lart_msgs::msg::ConeArray>();
      cone_ser.deserialize_message(&serialized, m.get());
      cones.emplace_back(recv_t, m);  // cones are unstamped (header==0); use recv, mapped below
    } else if (topic == "/mission") {
      if (!mission) {
        auto m = std::make_shared<lart_msgs::msg::Mission>();
        mission_ser.deserialize_message(&serialized, m.get());
        mission = m;
      }
    } else if (topic == "/clock") {
      auto m = std::make_shared<rosgraph_msgs::msg::Clock>();
      clock_ser.deserialize_message(&serialized, m.get());
      clockmap.emplace_back(recv_t, static_cast<double>(m->clock.sec) + m->clock.nanosec * 1e-9);
    }
  }
  reader.close();

  // ---- map cone receive-time (wall) onto the sim-time axis via /clock ----
  std::sort(clockmap.begin(), clockmap.end());
  auto wall_to_sim = [&](double w) -> double {
    if (clockmap.empty()) return w;
    if (w <= clockmap.front().first) return clockmap.front().second;
    if (w >= clockmap.back().first) return clockmap.back().second;
    auto hi = std::lower_bound(clockmap.begin(), clockmap.end(), std::make_pair(w, -1e18));
    auto lo = hi - 1;
    const double a = (w - lo->first) / (hi->first - lo->first);
    return lo->second + a * (hi->second - lo->second);
  };
  for (auto & c : cones) c.first = wall_to_sim(c.first);

  // ---- merge into one time-ordered event stream (stable: IMU before DYN at equal t) ----
  enum EvType { EV_IMU = 0, EV_DYN = 1, EV_CONE = 2 };
  struct Ev { double t; int type; size_t idx; };
  std::vector<Ev> events;
  events.reserve(dyns.size() + imus.size() + cones.size());
  for (size_t i = 0; i < imus.size(); ++i) events.push_back({imus[i].first, EV_IMU, i});
  for (size_t i = 0; i < dyns.size(); ++i) events.push_back({dyns[i].first, EV_DYN, i});
  for (size_t i = 0; i < cones.size(); ++i) events.push_back({cones[i].first, EV_CONE, i});
  std::stable_sort(events.begin(), events.end(), [](const Ev & a, const Ev & b) {
    if (a.t != b.t) return a.t < b.t;
    return a.type < b.type;  // IMU updates omega before the DYN prediction that consumes it
  });

  // ---- drive the SLAM synchronously ----
  GraphSLAM slam;
  slam.set_odom_noise_params(sigma_v_frac, sigma_theta_coeff, slip_inflation);
  if (mission) slam.set_mission(mission);

  std::ofstream csv(out_csv);
  csv << "t,x,y,yaw,event\n";
  csv << std::fixed;
  csv.precision(9);
  for (const auto & e : events) {
    if (e.type == EV_IMU) {
      slam.set_angular_velocity(imus[e.idx].second);
    } else if (e.type == EV_DYN) {
      slam.process_dynamics(dyns[e.idx].second);
      slam.compute_predicted_pose(e.t);
      const Eigen::Vector3d p = slam.get_current_pose();
      csv << e.t << "," << p[0] << "," << p[1] << "," << p[2] << ",P\n";
    } else {  // EV_CONE
      slam.process_observations(cones[e.idx].second);
      const Eigen::Vector3d p = slam.get_current_pose();
      csv << e.t << "," << p[0] << "," << p[1] << "," << p[2] << ",C\n";
    }
  }
  csv.close();

  std::cerr << "[offline] events=" << events.size()
            << " dyn=" << dyns.size() << " imu=" << imus.size()
            << " cone=" << cones.size()
            << " odom(sigma_v_frac=" << sigma_v_frac
            << ", sigma_theta_coeff=" << sigma_theta_coeff
            << ", slip=" << slip_inflation << ") -> " << out_csv << "\n";
  rclcpp::shutdown();
  return 0;
}
