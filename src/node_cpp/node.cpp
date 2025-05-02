#include <rclcpp/rclcpp.hpp>
#include <protocol/srv/motion_result_cmd.hpp>

class BasicCmd : public rclcpp::Node
{
public:
    BasicCmd(const std::string &name) : Node(name)
    {
        client_ = this->create_client<protocol::srv::MotionResultCmd>("/Cyberdog32/motion_result_cmd");
        // Wait for the service server to be available
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service to appear.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the service to appear...");
        }
    }

    void send_request()
    {
        auto request = std::make_shared<protocol::srv::MotionResultCmd::Request>();
        // Set the request parameters if needed
        request->motion_id = 111;

        // Send the request and get a future object
        auto future_result = client_->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Service call succeeded.");
            auto response = future_result.get();
            // Process the response if needed
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

private:
    rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicCmd>("basic_cmd_node");
    node->send_request();
    rclcpp::shutdown();
    return 0;
}
