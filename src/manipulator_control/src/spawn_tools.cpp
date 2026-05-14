#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

double TABLE_SURFACE_Z = 0.77;

struct SpawnSlot
{
  const char * entity_name;
  /** Path relative to share/.../manipulator_control/sdf/ */
  const char * sdf_relative;
  double x;
  double y;
  /** World-frame origin height for the model pose. */
  double z;
  /** Fixed-frame X→Y→Z Euler (rad): roll about X, pitch about Y, yaw about Z. */
  double roll_rad;
  double pitch_rad;
  double yaw_rad;
};

static void hamiltonProduct(
  double ax, double ay, double az, double aw,
  double bx, double by, double bz, double bw,
  double & ox, double & oy, double & oz, double & ow)
{
  ox = aw * bx + ax * bw + ay * bz - az * by;
  oy = aw * by - ax * bz + ay * bw + az * bx;
  oz = aw * bz + ax * by - ay * bx + az * bw;
  ow = aw * bw - ax * bx - ay * by - az * bz;
}

/** Rotation R = Rz(yaw) Ry(pitch) Rx(roll) (aerospace / fixed-frame sequence). */
static void setOrientationRPY(
  ros_gz_interfaces::srv::SpawnEntity::Request & req,
  double roll_rad, double pitch_rad, double yaw_rad)
{
  const double hr = roll_rad * 0.5;
  const double hp = pitch_rad * 0.5;
  const double hy = yaw_rad * 0.5;

  const double qrx = std::sin(hr);
  const double qrw = std::cos(hr);
  const double qry = 0.0;
  const double qrz = 0.0;

  const double qpx = 0.0;
  const double qpy = std::sin(hp);
  const double qpz = 0.0;
  const double qpw = std::cos(hp);

  const double qzx = 0.0;
  const double qzy = 0.0;
  const double qzz = std::sin(hy);
  const double qzw = std::cos(hy);

  double ix, iy, iz, iw;
  hamiltonProduct(qpx, qpy, qpz, qpw, qrx, qry, qrz, qrw, ix, iy, iz, iw);
  double ox, oy, oz, ow;
  hamiltonProduct(qzx, qzy, qzz, qzw, ix, iy, iz, iw, ox, oy, oz, ow);

  req.entity_factory.pose.orientation.x = ox;
  req.entity_factory.pose.orientation.y = oy;
  req.entity_factory.pose.orientation.z = oz;
  req.entity_factory.pose.orientation.w = ow;
}

class ToolSpawner : public rclcpp::Node
{
public:
  ToolSpawner()
  : Node("tool_spawner")
  {
    sdf_dir_ = ament_index_cpp::get_package_share_directory("manipulator_control") + "/sdf/";

    client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>(
      "/world/manipulator_world/create");

    RCLCPP_INFO(this->get_logger(), "Waiting for Gazebo spawn service...");
    while (!client_->wait_for_service(2s)) {
      RCLCPP_INFO(this->get_logger(), "  still waiting...");
    }
    RCLCPP_INFO(this->get_logger(), "Spawn service ready!");
  }

  /** Lay out props in a fixed 3×2 grid on the tabletop for repeatable top-camera experiments. */
  void spawnAll()
  {
    RCLCPP_INFO(this->get_logger(), "Spawning tool / prop set (grid layout for overhead YOLO)...");
    RCLCPP_INFO(this->get_logger(), "SDF directory: %s", sdf_dir_.c_str());

    // Columns spaced for long tools (extra gap between screwdriver and pencil on the top row).
    static double COL_L = 0.16;
    static double COL_M = 0.28;
    static double COL_R = 0.44;
    static double ROW_TOP = 0.14;
    static double ROW_BOT = -0.14;

    static const std::vector<SpawnSlot> kSlots = {
      {"cup", "Milk_cup/model.sdf", COL_L, ROW_TOP, TABLE_SURFACE_Z + 0.0, 0.0, 0.0, 0.0},
      {"bottle",
        "Bottle/model.sdf",
        0.320, -0.09, 0.77,0.0,
        0.0,
        0.785398},  // π/
    };

    for (const auto & slot : kSlots) {
      const std::string path = sdf_dir_ + slot.sdf_relative;
      spawnEntityFromFile(
        slot.entity_name, path, slot.x, slot.y, slot.z,
        slot.roll_rad, slot.pitch_rad, slot.yaw_rad);
      std::this_thread::sleep_for(350ms);
    }

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Spawned %zu objects (3×2 grid).", kSlots.size());
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  std::string sdf_dir_;
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr client_;

  /// Spawn from disk path so large meshes do not inline over the ROS–GZ bridge.
  void spawnEntityFromFile(
    const std::string & name, const std::string & absolute_sdf_path,
    double x, double y, double z, double roll_rad, double pitch_rad, double yaw_rad)
  {
    auto req = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    req->entity_factory.sdf = "";
    req->entity_factory.sdf_filename = absolute_sdf_path;
    req->entity_factory.name = name;
    req->entity_factory.relative_to = "world";
    req->entity_factory.pose.position.x = x;
    req->entity_factory.pose.position.y = y;
    req->entity_factory.pose.position.z = z;
    setOrientationRPY(*req, roll_rad, pitch_rad, yaw_rad);

    auto future = client_->async_send_request(req).future.share();

    auto rc = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), future, 60s);

    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      if (future.get()->success) {
        RCLCPP_INFO(
          this->get_logger(),
          "Spawned %-22s at (%.3f, %.3f, %.3f) rpy=(%.2f, %.2f, %.2f)",
          name.c_str(), x, y, z, roll_rad, pitch_rad, yaw_rad);
      } else {
        RCLCPP_WARN(
          this->get_logger(), "Gazebo rejected (success=false): %s — check SDF/mesh paths.",
          name.c_str());
      }
    } else if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for spawn: %s", name.c_str());
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Spawn future failed (code %d): %s", static_cast<int>(rc),
        name.c_str());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ToolSpawner>();
  node->spawnAll();
  rclcpp::shutdown();
  return 0;
}
