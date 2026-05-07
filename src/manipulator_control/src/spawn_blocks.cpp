#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>
#include <stdexcept>

using namespace std::chrono_literals;

static constexpr double BLOCK_H        = 0.025;
static constexpr double BLOCK_Z        = 0.77 + BLOCK_H / 2.0;

static constexpr double SPAWN_X_MIN    =  0.15;
static constexpr double SPAWN_X_MAX    =  0.28;
static constexpr double SPAWN_Y_MIN    = -0.15;
static constexpr double SPAWN_Y_MAX    =  0.15;

static constexpr double MIN_BLOCK_DIST =  0.08;
static constexpr double DROP_EXCL      =  0.10;

struct Zone { double x, y; };
static const Zone RED_DROP  = {0.20, -0.20};
static const Zone BLUE_DROP = {0.20,  0.20};


// ── Helpers ───────────────────────────────────────────────────────────────────
std::string readFile(const std::string & path)
{
  std::ifstream f(path);
  if (!f.is_open()) throw std::runtime_error("Cannot open: " + path);
  return std::string(std::istreambuf_iterator<char>(f),
                     std::istreambuf_iterator<char>());
}

std::string replaceAll(std::string str,
                       const std::string & from,
                       const std::string & to)
{
  size_t pos = 0;
  while ((pos = str.find(from, pos)) != std::string::npos) {
    str.replace(pos, from.size(), to);
    pos += to.size();
  }
  return str;
}

double dist2D(double x1, double y1, double x2, double y2)
{
  return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

bool tooCloseToDropZones(double x, double y)
{
  return dist2D(x, y, RED_DROP.x,  RED_DROP.y)  < DROP_EXCL ||
         dist2D(x, y, BLUE_DROP.x, BLUE_DROP.y) < DROP_EXCL;
}

bool tooCloseToBlocks(double x, double y,
                      const std::vector<std::pair<double,double>> & placed)
{
  for (const auto & [bx, by] : placed) {
    if (dist2D(x, y, bx, by) < MIN_BLOCK_DIST) return true;
  }
  return false;
}


// ── BlockSpawner node ─────────────────────────────────────────────────────────
class BlockSpawner : public rclcpp::Node
{
public:
  BlockSpawner() : Node("block_spawner")
  {
    sdf_dir_ = ament_index_cpp::get_package_share_directory("manipulator_control")
               + "/sdf/";

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
    // ── Block definitions: name + RGBA color ─────────────────────────────────
    struct BlockDef { std::string name; std::string rgba; };
    std::vector<BlockDef> blocks = {
      {"red_block_1",  "1.0 0.0 0.0 1.0"},
      {"red_block_2",  "1.0 0.0 0.0 1.0"},
      {"red_block_3",  "1.0 0.0 0.0 1.0"},
      {"blue_block_1", "0.0 0.0 1.0 1.0"},
      {"blue_block_2", "0.0 0.0 1.0 1.0"},
    };

    // ── Load the single block template once ──────────────────────────────────
    std::string block_template = readFile(sdf_dir_ + "block.sdf");

    // ── Random position generator ─────────────────────────────────────────────
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> rX(SPAWN_X_MIN, SPAWN_X_MAX);
    std::uniform_real_distribution<double> rY(SPAWN_Y_MIN, SPAWN_Y_MAX);

    std::vector<std::pair<double,double>> placed;

    // ── Spawn each block ──────────────────────────────────────────────────────
    for (const auto & block : blocks) {
      // Find valid random position
      double x = 0.0, y = 0.0;
      bool found = false;
      for (int attempt = 0; attempt < 400; ++attempt) {
        x = rX(gen);
        y = rY(gen);
        if (!tooCloseToDropZones(x, y) && !tooCloseToBlocks(x, y, placed)) {
          found = true;
          break;
        }
      }
      if (!found) {
        RCLCPP_ERROR(this->get_logger(),
          "No valid position for %s", block.name.c_str());
        continue;
      }

      // Fill in placeholders: name and color
      std::string sdf = replaceAll(block_template, "BLOCK_NAME",  block.name);
                  sdf = replaceAll(sdf,             "BLOCK_COLOR", block.rgba);

      spawnEntity(block.name, sdf, x, y, BLOCK_Z);
      placed.push_back({x, y});
      std::this_thread::sleep_for(400ms);
    }

    // ── Spawn drop zone markers ───────────────────────────────────────────────
    std::string marker_template = readFile(sdf_dir_ + "marker.sdf");

    // Red marker
    std::string red_marker = replaceAll(marker_template, "MARKER_NAME",     "red_drop_marker");
                red_marker = replaceAll(red_marker,      "MARKER_COLOR",    "1.0 0.0 0.0 0.8");
                red_marker = replaceAll(red_marker,      "MARKER_EMISSIVE", "0.8 0.0 0.0 1.0");

    // Blue marker
    std::string blue_marker = replaceAll(marker_template, "MARKER_NAME",     "blue_drop_marker");
                blue_marker = replaceAll(blue_marker,     "MARKER_COLOR",    "0.0 0.0 1.0 0.8");
                blue_marker = replaceAll(blue_marker,     "MARKER_EMISSIVE", "0.0 0.0 0.8 1.0");

    std::this_thread::sleep_for(300ms);
    spawnEntity("red_drop_marker",  red_marker,  RED_DROP.x,  RED_DROP.y,  0.771);
    spawnEntity("blue_drop_marker", blue_marker, BLUE_DROP.x, BLUE_DROP.y, 0.771);

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "All blocks and markers spawned!");
    RCLCPP_INFO(this->get_logger(), "Red  drop: (%.2f, %.2f)", RED_DROP.x,  RED_DROP.y);
    RCLCPP_INFO(this->get_logger(), "Blue drop: (%.2f, %.2f)", BLUE_DROP.x, BLUE_DROP.y);
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  std::string sdf_dir_;
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr client_;

  void spawnEntity(const std::string & name, const std::string & sdf,
                   double x, double y, double z)
  {
    auto req = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    req->entity_factory.sdf         = sdf;
    req->entity_factory.name        = name;
    req->entity_factory.relative_to = "world";
    req->entity_factory.pose.position.x    = x;
    req->entity_factory.pose.position.y    = y;
    req->entity_factory.pose.position.z    = z;
    req->entity_factory.pose.orientation.w = 1.0;

    auto future = client_->async_send_request(req).future.share();

    if (rclcpp::spin_until_future_complete(
          this->get_node_base_interface(), future, 10s)
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      if (future.get()->success) {
        RCLCPP_INFO(this->get_logger(),
          "Spawned %-20s at (%.3f, %.3f, %.3f)", name.c_str(), x, y, z);
      } else {
        RCLCPP_WARN(this->get_logger(), "Gazebo rejected: %s", name.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Timeout: %s", name.c_str());
    }
  }
};


// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BlockSpawner>();
  node->spawnAll();
  rclcpp::shutdown();
  return 0;
}
