#include <rclcpp/rclcpp.hpp>
#include <protocol/srv/motion_result_cmd.hpp>
#include <protocol/msg/motion_servo_response.hpp>
#include <protocol/msg/motion_servo_cmd.hpp>
#include <chrono>
#include <memory>
#include <csignal>

using MotionResultCmdSrv = protocol::srv::MotionResultCmd;
using MotionServoCmdMsg = protocol::msg::MotionServoCmd;

class ForwardController : public rclcpp::Node
{
public:
    ForwardController()
        : Node("forward_controller"), 
          current_motion_id_(0),
          is_moving_(false)
    {
        // 初始化服务客户端
        motion_result_client_ = this->create_client<MotionResultCmdSrv>(
            "/its_mygo_1/motion_result_cmd");
        
        // 初始化发布者
        servo_publisher_ = this->create_publisher<MotionServoCmdMsg>(
            "/its_mygo_1/motion_servo_cmd", 10);

        // 创建定时器（100ms周期）
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ForwardController::controlLoop, this));
        
        // 创建停止定时器（5秒后触发）
        stop_timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&ForwardController::stopMotion, this));

        RCLCPP_INFO(this->get_logger(), "ForwardController initialized");
    }

    void executeMotion(int motion_id, float speed, float direction)
    {
        if (is_moving_) {
            RCLCPP_WARN(this->get_logger(), "Already in motion!");
            return;
        }

        current_motion_id_ = motion_id;
        printf("MotionID=\n");
        std::cout<<motion_id<<"\n";
        speed_x_ = speed;
        direction_ = direction;
        is_moving_ = true;
        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), 
            "Motion started: ID=%d, Speed=%.1f, Dir=%.1f", 
            motion_id, speed, direction);
    }

private:
    // 主控制循环
    void controlLoop()
    {
        if (!is_moving_) return;

        auto current_time = this->now();
        auto elapsed = (current_time - start_time_).seconds();

        MotionServoCmdMsg msg;
        msg.motion_id = current_motion_id_;
        msg.cmd_type = 1;  // 控制指令类型
        msg.value = 2;     // 控制参数

        // 阶段控制逻辑
        if (elapsed < 2.0) {
            // 直行阶段
            msg.vel_des = {speed_x_, 0.0, 0.0};
            msg.step_height = {0.05, 0.05};
        }
        else if (elapsed < 5.0) {
            // 转向阶段
            msg.vel_des = {0.0, speed_x_, 0.0};
        //    msg.cmd_type = 2;  // 转向模式
            msg.step_height = {0.02, 0.02};
        }
        else {
            // 停止阶段
            msg.vel_des = {0.0, 0.0, 0.0};
            msg.cmd_type = 0;
            stopMotion();
            return;
        }

        RCLCPP_DEBUG(this->get_logger(),
            "Stage: %.1fs | Vel: [%.2f, %.2f] | Type: %d",
            elapsed, msg.vel_des[0], msg.vel_des[1], msg.cmd_type);

        servo_publisher_->publish(msg);
    }

    // 停止运动
    void stopMotion()
    {
        if (!is_moving_) return;

        is_moving_ = false;
        speed_x_ = 0.0;
        direction_ = 0.0;
        stop_timer_->cancel();
        // 发送停止指令
        MotionServoCmdMsg stop_msg;
        stop_msg.motion_id = current_motion_id_;
        stop_msg.cmd_type = 0;
        servo_publisher_->publish(stop_msg);

        RCLCPP_INFO(this->get_logger(), "Motion stopped");
    }

    // 服务回调函数
 /*   void handleMotionResult(
        MotionResultCmdSrv::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(),
            "Motion result received: ID=%d, Result=%d",
            response->motion_id, response->result_code);
    }
*/
    // 成员变量
    rclcpp::Client<MotionResultCmdSrv>::SharedPtr motion_result_client_;
    rclcpp::Publisher<MotionServoCmdMsg>::SharedPtr servo_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr stop_timer_;

    bool is_moving_;
    int current_motion_id_;
    float speed_x_;
    float direction_;
    rclcpp::Time start_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    printf("1\n");    
    auto controller = std::make_shared<ForwardController>();
    printf("2\n");
    // 发送初始运动指令
    controller->executeMotion(303, 0.3, 90.0);
    printf("3\n");
    // 设置信号处理
    std::signal(SIGINT, [](int sig) {
        RCLCPP_INFO(rclcpp::get_logger("signal"), "Received SIGINT");
        rclcpp::shutdown();
    });
    printf("4\n");
    try {
        rclcpp::spin(controller);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("exception"), "Error: %s", e.what());
    }
    printf("5\n");
    rclcpp::shutdown();
    return 0;
}