#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/move_cmd.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class SimplePiletNode : public rclcpp::Node {
public:
    SimplePiletNode()
        : Node("simple_pilet_node") {
        this->declare_parameter<bool>("start_run", false);
        // 声明目标位置参数
        this->declare_parameter<double>("target_x", 0.0);
        this->declare_parameter<double>("target_y", 0.0);
        this->declare_parameter<double>("target_yaw", 0.0);

        // 声明 P 控制器增益
        this->declare_parameter<double>("kp_x", 0.1);
        this->declare_parameter<double>("kp_y", 0.1);
        this->declare_parameter<double>("kp_yaw", 0.1);

        // 声明速度限制
        this->declare_parameter<double>("max_vx", 0.5);
        this->declare_parameter<double>("max_vy", 0.5);
        this->declare_parameter<double>("max_vyaw", 0.4);

        // 初始化成员变量
        target_x_   = this->get_parameter("target_x").as_double();
        target_y_   = this->get_parameter("target_y").as_double();
        target_yaw_ = this->get_parameter("target_yaw").as_double();

        kp_x_   = this->get_parameter("kp_x").as_double();
        kp_y_   = this->get_parameter("kp_y").as_double();
        kp_yaw_ = this->get_parameter("kp_yaw").as_double();

        max_vx_          = this->get_parameter("max_vx").as_double();
        max_vy_          = this->get_parameter("max_vy").as_double();
        max_vyaw_        = this->get_parameter("max_vyaw").as_double();
        run_p_controller = this->get_parameter("start_run").as_bool();

        // 创建参数回调
        parameter_callback_handle_ = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto& param : parameters) {
                if (param.get_name() == "target_x") {
                    target_x_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated target_x: %.3f", target_x_);
                } else if (param.get_name() == "target_y") {
                    target_y_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated target_y: %.3f", target_y_);
                } else if (param.get_name() == "target_yaw") {
                    target_yaw_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated target_yaw: %.3f", target_yaw_);
                } else if (param.get_name() == "kp_x") {
                    kp_x_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated kp_x: %.3f", kp_x_);
                } else if (param.get_name() == "kp_y") {
                    kp_y_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated kp_y: %.3f", kp_y_);
                } else if (param.get_name() == "kp_yaw") {
                    kp_yaw_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated kp_yaw: %.3f", kp_yaw_);
                } else if (param.get_name() == "max_vx") {
                    max_vx_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated max_vx: %.3f", max_vx_);
                } else if (param.get_name() == "max_vy") {
                    max_vy_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated max_vy: %.3f", max_vy_);
                } else if (param.get_name() == "max_vyaw") {
                    max_vyaw_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated max_vyaw: %.3f", max_vyaw_);
                } else if (param.get_name() == "start_run") {
                    run_p_controller= param.as_bool();
                }
            }

            return result;
        });

        // 创建发布器
        move_cmd_pub_ = this->create_publisher<robot_interfaces::msg::MoveCmd>("robot_move_cmd", 10);

        // 创建订阅器 - 机器人位置信息
        // 假设 dog 模型是第一个发布的模型（索引 0）
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/world/dog_world/dynamic_pose/info", 10, [this](const geometry_msgs::msg::PoseArray& msg) {
                if (msg.poses.empty()) {
                    return;
                }

                // 使用第一个模型的位置
                const auto& pose = msg.poses[0];
                current_x_       = pose.position.x;
                current_y_       = pose.position.y;

                // 从四元数提取 yaw 角
                tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch;
                m.getRPY(roll, pitch, current_yaw_);

                pose_updated_ = true;
                RCLCPP_DEBUG(this->get_logger(), "Robot pose: x=%.3f y=%.3f yaw=%.3f", current_x_, current_y_, current_yaw_);
            });

        // 创建定时器，定期更新控制命令
        control_timer_ = this->create_wall_timer(100ms, std::bind(&SimplePiletNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "节点初始化完成");
    }

private:
    // 控制循环
    void control_loop() {
        // 计算位置误差
        double error_x   = target_x_ - current_x_;
        double error_y   = target_y_ - current_y_;
        double error_yaw = target_yaw_ - current_yaw_;

        // 角度差绕回 [-pi, pi]
        if (error_yaw > M_PI)
            error_yaw -= 2 * M_PI;
        if (error_yaw < -M_PI)
            error_yaw += 2 * M_PI;

        // P 控制器计算速度指令（先在世界坐标系下算）
        double vx_world   = kp_x_ * error_x;
        double vy_world   = kp_y_ * error_y;
        double vyaw       = kp_yaw_ * error_yaw;

        // 将平面速度从世界坐标系转到机器人坐标系(body)
        const double cy = std::cos(current_yaw_);
        const double sy = std::sin(current_yaw_);
        double vx = cy * vx_world - sy * vy_world;
        double vy = sy * vx_world + cy * vy_world;

        // 限制速度
        vx   = std::clamp(vx,-max_vx_,max_vx_);
        vy   = std::clamp(vy,-max_vy_,max_vy_);
        vyaw   = std::clamp(vyaw,-max_vyaw_,max_vyaw_);

        // 发布命令
        robot_interfaces::msg::MoveCmd cmd;
        if (run_p_controller) {
            cmd.step_mode = 2;
            cmd.wheel_vel = 0.0f;
            cmd.vx        = static_cast<float>(vx);
            cmd.vy        = static_cast<float>(vy);
            cmd.vz        = static_cast<float>(vyaw);
        } else {
            cmd.step_mode = 1;
            cmd.wheel_vel = 0.0f;
            cmd.vx        = 0.0;
            cmd.vy        = 0.0;
            cmd.vz        = 0.0;
        }
        move_cmd_pub_->publish(cmd);

        RCLCPP_INFO(
            this->get_logger(), "误差: x=%.3f y=%.3f yaw=%.3f | Cmd: vx=%.3f vy=%.3f vyaw=%.3f", error_x, error_y, error_yaw, cmd.vx, cmd.vy,
            cmd.vz);
    }

    // 成员变量
    rclcpp::Publisher<robot_interfaces::msg::MoveCmd>::SharedPtr move_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    // 当前位置和状态
    double current_x_{0.0}, current_y_{0.0}, current_yaw_{0.0};
    bool pose_updated_{false};

    // 目标位置参数（私有成员变量）
    double target_x_{0.0};
    double target_y_{0.0};
    double target_yaw_{0.0};

    // P控制器增益参数（私有成员变量）
    double kp_x_{0.1};
    double kp_y_{0.1};
    double kp_yaw_{0.1};

    // 速度限制参数（私有成员变量）
    double max_vx_{0.5};
    double max_vy_{0.5};
    double max_vyaw_{0.4};
    bool run_p_controller{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePiletNode>());
    rclcpp::shutdown();
    return 0;
}
