// another_node.cpp in AnotherPkg package

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using std::placeholders::_1;

class MultiLocLaunchTester : public rclcpp::Node
{
  public:
    MultiLocLaunchTester() : Node("multi_loc_launch_tester")
    {
      // Get a list of namespaces from the parameters
      this->declare_parameter<std::vector<std::string>>("localization_modules", std::vector<std::string>{"ndt"});
      this->get_parameter("localization_modules", namespaces_);

      // Create a subscription for each namespace
      for (const std::string& ns : namespaces_)
      {
        std::string topic_name = "/" + ns + "/topic";

        auto callback = [this, ns](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            this->pose_with_covariance_stamped_callback(msg, ns);
        };

        subscriptions_.push_back(this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_name, 10, callback));
        RCLCPP_INFO(this->get_logger(), "Registered '%s'", ns.c_str());
      }
    }

  private:
    void pose_with_covariance_stamped_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, const std::string &str)
    {
      RCLCPP_INFO(this->get_logger(), "I heard from '%s'", str.c_str());
    }

    std::vector<std::string> namespaces_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> subscriptions_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiLocLaunchTester>());
  rclcpp::shutdown();
  return 0;
}