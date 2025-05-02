//12123123
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <stdio.h>

class DogController : public rclcpp::Node
{
public:
  DogController() : Node("dog_controller")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&DogController::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 1.0;  // 控制狗前进的速度

    printf("Publishing: '%f'\n", message.linear.x);
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DogController>());
  rclcpp::shutdown();
  return 0;
}
