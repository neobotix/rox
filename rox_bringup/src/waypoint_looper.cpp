// Neobotix GmbH
// Author: Adarsh Karan K P

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

class WaypointLooper : public rclcpp::Node {
public:

  WaypointLooper()
  : Node("waypoint_looper_seq"),
    running_(false), // to track if the loop is running
    goal_in_flight_(false), // to track if a goal is currently being processed
    loop_idx_(0), // current loop iteration
    wp_idx_(0) // current waypoint index
  {
    yaml_file_    = declare_parameter<std::string>("yaml_file", std::string(getenv("HOME")) + "/waypoints.yaml");
    frame_id_     = declare_parameter<std::string>("frame_id", "map");
    repeat_count_ = static_cast<size_t>(declare_parameter<int>("repeat_count", 100)); //casted to to avoid signed/unsigned comparison warnings
    delay_ms_     = declare_parameter<int>("wait_at_waypoint_ms", 200);
    action_name_  = declare_parameter<std::string>("action_name", "/navigate_to_pose");
    stop_on_fail_ = declare_parameter<bool>("stop_on_failure", true);

    client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, action_name_);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "start_waypoint_loop",
      std::bind(&WaypointLooper::startCallback, this, std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "stop_waypoint_loop",
      std::bind(&WaypointLooper::stopCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  // services
  void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (running_) { res->success = false; res->message = "Already running"; return; }
    if (!loadYaml() || waypoints_.empty()) { res->success = false; res->message = "No poses in YAML"; return; }
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      res->success = false; res->message = "NavigateToPose server not up"; return;
    }

    running_        = true;
    goal_in_flight_ = false;
    loop_idx_       = 0;
    wp_idx_         = 0;

    sendNext();
    res->success = true;
    res->message = "Started";
  }

  void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    running_ = false;
    goal_in_flight_ = false;
    if (delay_timer_) delay_timer_->cancel();
    res->success = true;
    res->message = "Stopped";
  }

  // yaml
  bool loadYaml() {
    try {
      YAML::Node root = YAML::LoadFile(yaml_file_);
      YAML::Node wps = root["waypoints"];
      waypoints_.clear();

      // loop through waypoints
      for (auto it = wps.begin(); it != wps.end(); ++it) {
        const std::string name = it->first.as<std::string>();
        const YAML::Node & p = it->second;
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id_;
        ps.pose.position.x = p["position"]["x"].as<double>();
        ps.pose.position.y = p["position"]["y"].as<double>();
        ps.pose.position.z = p["position"]["z"].as<double>();
        ps.pose.orientation.x = p["orientation"]["x"].as<double>();
        ps.pose.orientation.y = p["orientation"]["y"].as<double>();
        ps.pose.orientation.z = p["orientation"]["z"].as<double>();
        ps.pose.orientation.w = p["orientation"]["w"].as<double>();
        waypoints_.push_back({name, ps});
      }
      return true;
    } 
    catch (const YAML::BadFile & e) {
      RCLCPP_ERROR(get_logger(), "Failed to open YAML file: %s", e.what());
      return false;
    }
    catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "YAML error: %s", e.what());
      return false;
    }
  }

  // action sequencing
  void sendNext()
  {
    if (!running_ || goal_in_flight_) return;

    if (loop_idx_ >= repeat_count_) { finish(); return; }

    if (wp_idx_ >= waypoints_.size()) {
      wp_idx_ = 0;
      ++loop_idx_;
      if (loop_idx_ >= repeat_count_) { finish(); return; }
    }

    const auto & pair = waypoints_[wp_idx_];
    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose = pair.second;
    goal.pose.header.stamp = now();

    RCLCPP_INFO(get_logger(), "Loop %zu/%zu -> %s (%zu/%zu)",
                loop_idx_ + 1, repeat_count_, pair.first.c_str(),
                wp_idx_ + 1, waypoints_.size());

    goal_in_flight_ = true;

    auto options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> gh){
        if (!gh) {
          RCLCPP_ERROR(this->get_logger(), "Goal rejected");
          goal_in_flight_ = false;
          running_ = false;
        }
      };

    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
      {
        goal_in_flight_ = false;

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Reached");
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Result code %d", (int)result.code);
            if (stop_on_fail_) { running_ = false; finish(); return; }
            break;
        }

        ++wp_idx_;

        if (!running_) return;

        if (delay_ms_ > 0) {
          if (delay_timer_) delay_timer_->cancel();
          delay_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(delay_ms_),
            [this]() {
              delay_timer_->cancel();
              sendNext();
            });
        } else {
          sendNext();
        }
      };

    client_->async_send_goal(goal, options);
  }

  void finish()
  {
    running_ = false;
    goal_in_flight_ = false;
    RCLCPP_INFO(get_logger(), "Completed %zu loops.", repeat_count_);
  }

  // members
  int delay_ms_;
  std::string yaml_file_, frame_id_, action_name_;
  bool stop_on_fail_;

  bool running_, goal_in_flight_;
  size_t repeat_count_, loop_idx_, wp_idx_;

  std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> waypoints_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;
  rclcpp::TimerBase::SharedPtr delay_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointLooper>());
  rclcpp::shutdown();
  return 0;
}
