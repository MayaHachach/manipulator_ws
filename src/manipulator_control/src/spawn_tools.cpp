#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <unistd.h>

#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>
#include <stdexcept>
#include <filesystem>

using namespace std::chrono_literals;

// Table / spawn region — match spawn_blocks.cpp (table_world + task layout)
static constexpr double TABLE_SURFACE_Z = 0.77;

// Cup: model link is offset z=0.065; tune if mesh sinks/floats
static constexpr double CUP_SPAWN_Z     = TABLE_SURFACE_Z + 0;
// Screwdriver: mesh origin assumed near table contact; tune if needed
static constexpr double SCREWDRIVER_SPAWN_Z = TABLE_SURFACE_Z + 0.008;

static constexpr double SPAWN_X_MIN    =  0.2;
static constexpr double SPAWN_X_MAX    =  0.3;
static constexpr double SPAWN_Y_MIN    = -0.15;
static constexpr double SPAWN_Y_MAX    =  0.15;

static constexpr double MIN_TOOL_DIST  =  0.12;
static constexpr double DROP_EXCL      =  0.10;

struct Zone { double x, y; };
static const Zone RED_DROP  = {0.20, -0.20};
static const Zone BLUE_DROP = {0.20,  0.20};


std::string readFile(const std::string & path)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("Cannot open: " + path);
  }
  return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

std::string replaceAll(std::string str, const std::string & from, const std::string & to)
{
  size_t pos = 0;
  while ((pos = str.find(from, pos)) != std::string::npos) {
    str.replace(pos, from.size(), to);
    pos += to.size();
  }
  return str;
}

/// Local file URI for GZ mesh paths (encode spaces for "Plastic_Cup", etc.)
std::string toFileUri(const std::string & abs_path)
{
  std::string u = "file://";
  for (char c : abs_path) {
    if (c == ' ') {
      u += "%20";
    } else {
      u += c;
    }
  }
  return u;
}

double dist2D(double x1, double y1, double x2, double y2)
{
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

bool tooCloseToDropZones(double x, double y)
{
  return dist2D(x, y, RED_DROP.x, RED_DROP.y) < DROP_EXCL ||
         dist2D(x, y, BLUE_DROP.x, BLUE_DROP.y) < DROP_EXCL;
}

bool tooCloseToTools(
  double x, double y, const std::vector<std::pair<double, double>> & placed)
{
  for (const auto & [tx, ty] : placed) {
    if (dist2D(x, y, tx, ty) < MIN_TOOL_DIST) {
      return true;
    }
  }
  return false;
}

bool pickSpawnXY(
  double & x, double & y, const std::vector<std::pair<double, double>> & placed)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> rX(SPAWN_X_MIN, SPAWN_X_MAX);
  std::uniform_real_distribution<double> rY(SPAWN_Y_MIN, SPAWN_Y_MAX);

  for (int attempt = 0; attempt < 400; ++attempt) {
    x = rX(gen);
    y = rY(gen);
    if (!tooCloseToDropZones(x, y) && !tooCloseToTools(x, y, placed)) {
      return true;
    }
  }
  return false;
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

  void spawnAll()
  {
    // make some logs to debug
    RCLCPP_INFO(this->get_logger(), "Spawning tools...");
    RCLCPP_INFO(this->get_logger(), "SDF Directory: %s", sdf_dir_.c_str());
    const std::string pencil_sdf =
      sdf_dir_ + "pencil_gz_sim/pencil/model.sdf";
    RCLCPP_INFO(this->get_logger(), "Pencil SDF: %s", pencil_sdf.c_str());
    const std::string cup_sdf =
      sdf_dir_ + "Milk_cup/model.sdf";
    RCLCPP_INFO(this->get_logger(), "Milk cup SDF: %s", cup_sdf.c_str());

    std::vector<std::pair<double, double>> placed;

    // ── 1) Pencil — load by file path so GZ resolves meshes/ relative to model.sdf
    {
      double x = 0.0, y = 0.0;
      if (!pickSpawnXY(x, y, placed)) {
        RCLCPP_ERROR(this->get_logger(), "No valid position for pencil_1");
        return;
      }

      spawnEntityFromFile("pencil_1", pencil_sdf, x, y, SCREWDRIVER_SPAWN_Z);
      placed.push_back({x, y});
      std::this_thread::sleep_for(500ms);
    }
    RCLCPP_INFO(this->get_logger(), "Pencil spawned");
    // ── 2) Cup — load by file path with relative mesh URIs (same as screwdriver)
    {
      double x = 0.0, y = 0.0;
      if (!pickSpawnXY(x, y, placed)) {
        RCLCPP_ERROR(this->get_logger(), "No valid position for plastic_cup_1");
        return;
      }

      spawnEntityFromFile("plastic_cup_1", cup_sdf, x, y, CUP_SPAWN_Z);
      placed.push_back({x, y});
      std::this_thread::sleep_for(500ms);
    }
    RCLCPP_INFO(this->get_logger(), "Cup spawned");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Tools spawned (pencil, then cup).");
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  std::string sdf_dir_;
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr client_;

  /// Prefer spawning from disk: avoids bridge hangs with large inline SDF strings.
  void spawnEntityFromFile(
    const std::string & name, const std::string & absolute_sdf_path,
    double x, double y, double z)
  {
    auto req = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    req->entity_factory.sdf = "";
    req->entity_factory.sdf_filename = absolute_sdf_path;
    req->entity_factory.name = name;
    req->entity_factory.relative_to = "world";
    req->entity_factory.pose.position.x = x;
    req->entity_factory.pose.position.y = y;
    req->entity_factory.pose.position.z = z;
    req->entity_factory.pose.orientation.w = 1.0;

    auto future = client_->async_send_request(req).future.share();

    auto rc = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), future, 60s);

    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      if (future.get()->success) {
        RCLCPP_INFO(
          this->get_logger(), "Spawned %-20s at (%.3f, %.3f, %.3f)", name.c_str(), x, y, z);
      } else {
        RCLCPP_WARN(this->get_logger(), "Gazebo rejected (success=false): %s", name.c_str());
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
