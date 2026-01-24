#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_interfaces/msg/move_cmd.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class TestMoveNode : public rclcpp::Node{
public:
    TestMoveNode():  Node("test_move_node")
    {
        // declare parameters so rqt param plugin can modify them
        auto vx_descriptor = rcl_interfaces::msg::ParameterDescriptor();
        vx_descriptor.description = "Velocity in X direction";
        vx_descriptor.floating_point_range.resize(1);
        vx_descriptor.floating_point_range[0].from_value = -1.0;
        vx_descriptor.floating_point_range[0].to_value = 1.0;
        this->declare_parameter<double>("vx", 0.0, vx_descriptor);

        auto vy_descriptor = rcl_interfaces::msg::ParameterDescriptor();
        vy_descriptor.description = "Velocity in Y direction";
        vy_descriptor.floating_point_range.resize(1);
        vy_descriptor.floating_point_range[0].from_value = -1.0;
        vy_descriptor.floating_point_range[0].to_value = 1.0;
        this->declare_parameter<double>("vy", 0.0, vy_descriptor);

        auto vz_descriptor = rcl_interfaces::msg::ParameterDescriptor();
        vz_descriptor.description = "Velocity in Z direction";
        vz_descriptor.floating_point_range.resize(1);
        vz_descriptor.floating_point_range[0].from_value = -1.0;
        vz_descriptor.floating_point_range[0].to_value = 1.0;
        this->declare_parameter<double>("vz", 0.0, vz_descriptor);


        this->declare_parameter<int>("step_type", 0);

        // create publisher
        pub_ = this->create_publisher<robot_interfaces::msg::MoveCmd>("robot_move_cmd", 10);

        // set up parameter change callback: publish a new MoveCmd whenever parameters change
        param_cb_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params){
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto & p : params) {
                    if (p.get_name() == "vx") {
                        if (p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            vx_ = static_cast<float>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "vx must be a number";
                            return result;
                        }
                    } else if (p.get_name() == "vy") {
                        if (p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            vy_ = static_cast<float>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "vy must be a number";
                            return result;
                        }
                    } else if (p.get_name() == "vz") {
                        if (p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            vz_ = static_cast<float>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "vz must be a number";
                            return result;
                        }
                    } else if (p.get_name() == "step_type") {
                        if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            step_type_ = static_cast<uint32_t>(p.as_int());
                        } else if (p.get_type() == rclcpp::PARAMETER_DOUBLE) {
                            // allow integers provided as double
                            step_type_ = static_cast<uint32_t>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "step_type must be an integer";
                            return result;
                        }
                    }
                }
                return result;
            }
        );

        // publish initial message based on default parameters
        vx_ = static_cast<float>(this->get_parameter("vx").as_double());
        vy_ = static_cast<float>(this->get_parameter("vy").as_double());
        vz_ = static_cast<float>(this->get_parameter("vz").as_double());
        step_type_ = static_cast<uint32_t>(this->get_parameter("step_type").as_int());
        publish_move_cmd();

        update_timer=this->create_wall_timer(100ms ,[this](){
            publish_move_cmd();
        });
    }

private:
    void publish_move_cmd()
    {
        robot_interfaces::msg::MoveCmd msg;
        msg.step_mode = step_type_;
        msg.wheel_vel = 0.0f; // unused by this test node
        msg.vx = vx_;
        msg.vy = vy_;
        msg.vz = vz_;
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published MoveCmd: step_type=%u vx=%.3f vy=%.3f vz=%.3f",
                    step_type_, vx_, vy_, vz_);
    }

    rclcpp::Publisher<robot_interfaces::msg::MoveCmd>::SharedPtr pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rclcpp::TimerBase::SharedPtr update_timer;

    float vx_{0.0f}, vy_{0.0f}, vz_{0.0f};
    uint32_t step_type_{0};
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TestMoveNode>());
    rclcpp::shutdown();
    return 0;
}