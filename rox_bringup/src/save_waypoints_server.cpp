// Neobotix GmbH
// Author: Adarsh Karan K P

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <yaml-cpp/yaml.h>

#include <mutex>
#include <fstream>
#include <map>
#include <regex>

class SaveWaypointsServer : public rclcpp::Node
{
public:
  SaveWaypointsServer()
  : Node("save_waypoints_server")
  {
    // Params
    waypoints_topic_ = declare_parameter<std::string>("waypoints_topic", "/waypoints");
    output_file_     = declare_parameter<std::string>("output_file", std::string(getenv("HOME")) + "/waypoints.yaml");

    // Subscription for MarkerArray - waypoints
    sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      waypoints_topic_, 10,
      std::bind(&SaveWaypointsServer::markersCallback, this, std::placeholders::_1));

    // Service to save waypoints
    srv_ = create_service<std_srvs::srv::Trigger>(
      "save_waypoints",
      std::bind(&SaveWaypointsServer::handleSave, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Listening on %s, service /save_waypoints ready.", waypoints_topic_.c_str());
  }

private:
  void markersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    latest_markers_ = *msg;
  }

  void handleSave(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    try {
      visualization_msgs::msg::MarkerArray copy;
      {
        std::lock_guard<std::mutex> lk(mutex_);
        if (!latest_markers_) {
          res->success = false;
          res->message = "No MarkerArray received yet.";
          return;
        }
        copy = *latest_markers_;
      }

      // Extract waypoints from markers
      auto data = extractWaypoints(copy);

      std::ofstream fout(output_file_);
      if (!fout.is_open()) {
        res->success = false;
        res->message = "Failed to open file for writing. The directory or file may not exist.";
        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
        return;
      }

      YAML::Emitter out;
      out << YAML::BeginMap;
      out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginMap;

      for (const auto & kv : data) {
        const auto & name = kv.first;
        const auto & pose = kv.second;
        out << YAML::Key << name << YAML::Value << YAML::BeginMap;

        out << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << pose.position.x;
        out << YAML::Key << "y" << YAML::Value << pose.position.y;
        out << YAML::Key << "z" << YAML::Value << pose.position.z;
        out << YAML::EndMap; // position

        out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << pose.orientation.x;
        out << YAML::Key << "y" << YAML::Value << pose.orientation.y;
        out << YAML::Key << "z" << YAML::Value << pose.orientation.z;
        out << YAML::Key << "w" << YAML::Value << pose.orientation.w;
        out << YAML::EndMap; // orientation

        out << YAML::EndMap; // point_N
      }

      out << YAML::EndMap;   // waypoints
      out << YAML::EndMap;   // root

      fout << out.c_str();
      fout.close();

      res->success = true;
      res->message = "Saved " + std::to_string(data.size()) + " waypoints to " + output_file_;
      RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    } catch (const std::exception & e) {
      res->success = false;
      res->message = std::string("Error: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    }
  }

  struct PoseWrapper {
    geometry_msgs::msg::Pose pose;
  };

  std::map<std::string, geometry_msgs::msg::Pose> extractWaypoints(
      const visualization_msgs::msg::MarkerArray & arr)
  {
    std::map<int, std::string> text_by_id;
    std::map<int, geometry_msgs::msg::Pose> pose_by_id;

    for (const auto & m : arr.markers) {
      if (m.type == visualization_msgs::msg::Marker::TEXT_VIEW_FACING && !m.text.empty()) {
        text_by_id[m.id] = m.text;
      }
      if (m.type == visualization_msgs::msg::Marker::CUBE ||
          m.type == visualization_msgs::msg::Marker::SPHERE ||
          m.type == visualization_msgs::msg::Marker::CYLINDER ||
          m.type == visualization_msgs::msg::Marker::ARROW) {
          pose_by_id[m.id] = m.pose;
      }
    }

    std::map<std::string, geometry_msgs::msg::Pose> result;
    for (const auto & kv : text_by_id) {
      int id = kv.first;
      const std::string & name = kv.second;
      geometry_msgs::msg::Pose pose;

      auto it = pose_by_id.find(id);
      if (it != pose_by_id.end()) {
        pose = it->second;
      } else {
        // fallback: use text marker pose itself
        for (const auto & m : arr.markers) {
          if (m.id == id) {
            pose = m.pose;
            break;
          }
        }
      }
      result[name] = pose;
    }
    return result;
  }

  // Members
  std::string waypoints_topic_;
  std::string output_file_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

  std::mutex mutex_;
  std::optional<visualization_msgs::msg::MarkerArray> latest_markers_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SaveWaypointsServer>());
  rclcpp::shutdown();
  return 0;
}
