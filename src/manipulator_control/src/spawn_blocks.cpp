#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <chrono>
#include <thread>
#include <sstream>

using namespace std::chrono_literals;

// ── Block dimensions ──────────────────────────────────────────────────────────
static constexpr double BLOCK_L = 0.040;   // long axis along X
static constexpr double BLOCK_W = 0.025;   // short axis along Y
static constexpr double BLOCK_H = 0.025;   // height along Z
static constexpr double BLOCK_Z = 0.77 + BLOCK_H / 2.0;

// ── Spawn zone (world frame) ──────────────────────────────────────────────────
static constexpr double SPAWN_X_MIN =  0.15;
static constexpr double SPAWN_X_MAX =  0.28;
static constexpr double SPAWN_Y_MIN = -0.10;
static constexpr double SPAWN_Y_MAX =  0.10;

// ── Drop zones (world frame) ──────────────────────────────────────────────────
struct Zone { double x, y; };
static const Zone RED_DROP  = {0.30, -0.20};
static const Zone BLUE_DROP = {0.30,  0.20};

// ── Safety margins ────────────────────────────────────────────────────────────
static constexpr double MIN_BLOCK_DIST = 0.08;
static constexpr double DROP_EXCL      = 0.10;


// ── SDF generator ─────────────────────────────────────────────────────────────
std::string makeBlockSDF(const std::string & name, const std::string & rgba)
{
  std::ostringstream ss;
  ss << "<?xml version=\"1.0\"?>\n"
     << "<sdf version=\"1.9\">\n"
     << "  <model name=\"" << name << "\">\n"
     << "    <static>false</static>\n"
     << "    <link name=\"block_link\">\n"
     << "      <pose>0 0 0 0 0 0</pose>\n"
     << "      <inertial>\n"
     << "        <mass>0.05</mass>\n"
     << "        <inertia>\n"
     << "          <ixx>3e-6</ixx><ixy>0</ixy><ixz>0</ixz>\n"
     << "          <iyy>5e-6</iyy><iyz>0</iyz>\n"
     << "          <izz>6e-6</izz>\n"
     << "        </inertia>\n"
     << "      </inertial>\n"
     << "      <collision name=\"collision\">\n"
     << "        <geometry>\n"
     << "          <box><size>" << BLOCK_L << " " << BLOCK_W << " " << BLOCK_H << "</size></box>\n"
     << "        </geometry>\n"
     << "      </collision>\n"
     << "      <visual name=\"visual\">\n"
     << "        <geometry>\n"
     << "          <box><size>" << BLOCK_L << " " << BLOCK_W << " " << BLOCK_H << "</size></box>\n"
     << "        </geometry>\n"
     << "        <material>\n"
     << "          <ambient>" << rgba << "</ambient>\n"
     << "          <diffuse>" << rgba << "</diffuse>\n"
     << "          <specular>0.1 0.1 0.1 1</specular>\n"
     << "        </material>\n"
     << "      </visual>\n"
     << "    </link>\n"
     << "  </model>\n"
     << "</sdf>\n";
  return ss.str();
}


// ── Distance helpers ──────────────────────────────────────────────────────────
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


// ── Block spawner node ────────────────────────────────────────────────────────
class BlockSpawner : public rclcpp::Node
{
public:
  BlockSpawner() : Node("block_spawner")
  {
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
    struct BlockDef { std::string name; std::string rgba; };
    std::vector<BlockDef> blocks = {
      {"red_block_1",  "1.0 0.0 0.0 1.0"},
      {"red_block_2",  "1.0 0.0 0.0 1.0"},
      {"red_block_3",  "1.0 0.0 0.0 1.0"},
      {"blue_block_1", "0.0 0.0 1.0 1.0"},
      {"blue_block_2", "0.0 0.0 1.0 1.0"},
    };

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> rX(SPAWN_X_MIN, SPAWN_X_MAX);
    std::uniform_real_distribution<double> rY(SPAWN_Y_MIN, SPAWN_Y_MAX);

    std::vector<std::pair<double,double>> placed;

    for (const auto & block : blocks) {
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
          "Could not find valid position for %s!", block.name.c_str());
        continue;
      }

      // yaw = 0.0 → long axis aligned with X axis → gripper can pick directly
      spawnBlock(block.name, block.rgba, x, y, BLOCK_Z, 0.0);
      placed.push_back({x, y});
      std::this_thread::sleep_for(400ms);
    }

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "All 5 blocks spawned successfully!");
    RCLCPP_INFO(this->get_logger(), "Block orientation: long axis along X (yaw=0)");
    RCLCPP_INFO(this->get_logger(), "Spawn zone:  X[%.2f, %.2f]  Y[%.2f, %.2f]",
      SPAWN_X_MIN, SPAWN_X_MAX, SPAWN_Y_MIN, SPAWN_Y_MAX);
    RCLCPP_INFO(this->get_logger(), "Red  drop:   X=%.2f  Y=%.2f",
      RED_DROP.x, RED_DROP.y);
    RCLCPP_INFO(this->get_logger(), "Blue drop:   X=%.2f  Y=%.2f",
      BLUE_DROP.x, BLUE_DROP.y);
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr client_;

  void spawnBlock(const std::string & name, const std::string & rgba,
                  double x, double y, double z, double yaw)
  {
    auto req = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();

    req->entity_factory.sdf         = makeBlockSDF(name, rgba);
    req->entity_factory.name        = name;
    req->entity_factory.relative_to = "world";

    req->entity_factory.pose.position.x = x;
    req->entity_factory.pose.position.y = y;
    req->entity_factory.pose.position.z = z;

    // yaw=0 → identity quaternion → long axis along X
    req->entity_factory.pose.orientation.x = 0.0;
    req->entity_factory.pose.orientation.y = 0.0;
    req->entity_factory.pose.orientation.z = std::sin(yaw / 2.0);  // = 0
    req->entity_factory.pose.orientation.w = std::cos(yaw / 2.0);  // = 1

    auto future_and_id = client_->async_send_request(req);
    auto future = future_and_id.future.share();

    if (rclcpp::spin_until_future_complete(
          this->get_node_base_interface(), future, 10s)
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = future.get();
      if (result->success) {
        RCLCPP_INFO(this->get_logger(),
          "Spawned %-14s at (%.3f, %.3f, %.3f)",
          name.c_str(), x, y, z);
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Gazebo rejected spawn for %s", name.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
        "Service call timed out for %s", name.c_str());
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