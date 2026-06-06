#include <bot_interfaces/srv/matched_correspondences2d.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

namespace
{
class MatcherProbeClient : public rclcpp::Node
{
public:
  MatcherProbeClient()
  : Node("matcher_probe_client")
  {
    client_ = create_client<bot_interfaces::srv::MatchedCorrespondences2d>("/xfeat_lightglue");

    timer_ = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&MatcherProbeClient::send_request, this));
  }

private:
  void send_request()
  {
    if (!timer_) {
      return;
    }
    timer_->cancel();
    timer_.reset();

    RCLCPP_INFO(get_logger(), "Sending standalone matcher probe request");

    if (!client_->wait_for_service(std::chrono::milliseconds(1000))) {
      RCLCPP_WARN(get_logger(), "Matcher service /xfeat_lightglue is not available");
      return;
    }

    auto request = std::make_shared<bot_interfaces::srv::MatchedCorrespondences2d::Request>();
    request->selected_cam_ids = {0u, 1u};

    builtin_interfaces::msg::Time zero_stamp;
    zero_stamp.sec = 0;
    zero_stamp.nanosec = 0;
    request->selected_stamps = {zero_stamp, zero_stamp};
    request->choose_ref = true;
    request->ref_cam_id = 0;

    try {
      client_->async_send_request(
        request,
        std::bind(&MatcherProbeClient::handle_response, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Standalone matcher probe queued successfully");
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Standalone matcher probe threw exception: %s", ex.what());
    }
  }

  void handle_response(
    rclcpp::Client<bot_interfaces::srv::MatchedCorrespondences2d>::SharedFuture future)
  {
    const auto response = future.get();

    RCLCPP_INFO(
      get_logger(),
      "Standalone matcher probe response: success=%s ref_cam_id=%u matched_pairs=%zu message=%s",
      response->success ? "true" : "false",
      response->ref_cam_id,
      response->matched_pairs.size(),
      response->message.c_str());
  }

  rclcpp::Client<bot_interfaces::srv::MatchedCorrespondences2d>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MatcherProbeClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
