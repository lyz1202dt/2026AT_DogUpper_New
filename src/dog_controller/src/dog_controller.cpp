#include "dog_controller/dog_controller.hpp"
#include <algorithm>
#include <robot_interfaces/msg/robot.hpp>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <iomanip>

namespace dog_controller {
DogController::DogController() : csv_initialized_(false) {}

controller_interface::CallbackReturn DogController::on_init() {
    state_publisher   = get_node()->create_publisher<robot_interfaces::msg::Robot>("legs_status", 10);
    target_subscriber = get_node()->create_subscription<robot_interfaces::msg::Robot>(
        "legs_target", 10, [this](const robot_interfaces::msg::Robot& msg) { joints_target = msg; });
    joints_name_ = {"lf_joint1", "lf_joint2", "lf_joint3", "rf_joint1", "rf_joint2", "rf_joint3",
                    "lb_joint1", "lb_joint2", "lb_joint3", "rb_joint1", "rb_joint2", "rb_joint3"};
    joint_kp[0]=0.0;
    joint_kp[1]=0.0;
    joint_kp[2]=0.0;
    joint_kd[0]=0.0;
    joint_kd[1]=0.0;
    joint_kd[2]=0.0;

    auto node=get_node();
    node->declare_parameter("joint1_kp",50.0);
    node->declare_parameter("joint1_kd",3.0);
    node->declare_parameter("joint2_kp",50.0);
    node->declare_parameter("joint2_kd",3.0);
    node->declare_parameter("joint3_kp",50.0);
    node->declare_parameter("joint3_kd",3.0);
    node->declare_parameter("record_lf_torque",false);
    node->declare_parameter("joint_torque_filter_gate",0.8);
    node->declare_parameter("joint_omega_filter_gate",0.8);

    param_cb_=node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful=true;
        for (const auto& param : params) {
            if(param.get_name()=="joint1_kp")
                joint_kp[0]=param.as_double();
            else if(param.get_name()=="joint1_kd")
                joint_kd[0]=param.as_double();
            else if(param.get_name()=="joint2_kp")
                joint_kp[1]=param.as_double();
            else if(param.get_name()=="joint2_kd")
                joint_kd[1]=param.as_double();
            else if(param.get_name()=="joint3_kp")
                joint_kp[2]=param.as_double();
            else if(param.get_name()=="joint3_kd")
                joint_kd[2]=param.as_double();
            else if(param.get_name()=="record_lf_torque")
            {
                csv_initialized_=param.as_bool();
                if(csv_initialized_)
                {
                    start_time_ = std::chrono::steady_clock::now();
                    csv_file_.open("/home/lyz/Project/26_AT_new_dog/log/lf_joint_torque.csv");
                    csv_file_ << "time_ms,lf_joint1_torque,lf_joint2_torque,lf_joint3_torque\n";
                    RCLCPP_INFO(get_node()->get_logger(),"打开CSV文件");
                }
                else {
                    csv_file_.close();
                }
            }
            else if(param.get_name()=="joint_torque_filter_gate")   //设置电机力矩低通滤波器增益
                joint_torque_filter_gate=param.as_double();
            else if(param.get_name()=="joint_omega_filter_gate")   //设置电机转速低通滤波器增益
                joint_omega_filter_gate=param.as_double();
        }
        return result;
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DogController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    auto node=get_node();
    joint_kp[0]=node->get_parameter("joint1_kp").as_double();
    joint_kd[0]=node->get_parameter("joint1_kd").as_double();
    joint_kp[1]=node->get_parameter("joint2_kp").as_double();
    joint_kd[1]=node->get_parameter("joint2_kd").as_double();
    joint_kp[2]=node->get_parameter("joint3_kp").as_double();
    joint_kd[2]=node->get_parameter("joint3_kd").as_double();
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn DogController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    soft_start_start_time = std::chrono::steady_clock::now();
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn DogController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DogController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    auto joints_num = joints_name_.size();


    for (size_t i = 0; i < joints_num; i++) // 从接口读取关节状态
    {
        joints_state.legs[i / 3].joints[i % 3].rad    = state_interfaces_[i * 3 + 0].get_value();
        joints_state.legs[i / 3].joints[i % 3].omega  = joint_omega_filter_gate*joints_state.legs[i / 3].joints[i % 3].omega+(1.0-joint_omega_filter_gate)*state_interfaces_[i * 3 + 1].get_value();
        joints_state.legs[i / 3].joints[i % 3].torque = joint_torque_filter_gate* joints_state.legs[i / 3].joints[i % 3].torque+(1.0-joint_torque_filter_gate)*state_interfaces_[i * 3 + 2].get_value();
    }
    state_publisher->publish(joints_state);    // 发布关节状态

    if (csv_file_.is_open()) {  //如果需要记录，那么记录关节数据
        RCLCPP_INFO(get_node()->get_logger(),"记录数据");
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
        csv_file_ << elapsed << "," 
                  << std::fixed << std::setprecision(6)
                  << joints_state.legs[0].joints[0].torque << ","
                  << joints_state.legs[0].joints[1].torque << ","
                  << joints_state.legs[0].joints[2].torque << "\n";
        csv_file_.flush();
    }

    auto now = std::chrono::steady_clock::now();
    double elapsed_s = std::chrono::duration<double>(now - soft_start_start_time).count();
    double ramp = std::clamp(elapsed_s / soft_start_duration_.count(), 0.0, 1.0);

   for (size_t i = 0; i < joints_num; i++) // 将计算结果写入硬件层
    {
        // command_interfaces_[i * 3 + 0].set_value((double)joints_target.legs[i / 3].joints[i % 3].rad);
        // command_interfaces_[i * 3 + 1].set_value((double)joints_target.legs[i / 3].joints[i % 3].omega);
        // command_interfaces_[i * 3 + 2].set_value((double)joints_target.legs[i / 3].joints[i % 3].torque);
        
        double effort=joint_kp[i%3]*(joints_target.legs[i / 3].joints[i % 3].rad-joints_state.legs[i / 3].joints[i % 3].rad)+
                    joint_kd[i%3]*(joints_target.legs[i / 3].joints[i % 3].omega-joints_state.legs[i / 3].joints[i % 3].omega);
        
        effort = std::clamp(effort,-12.0,12.0);
        double sum_effort=effort+(double)joints_target.legs[i / 3].joints[i % 3].torque;
        sum_effort = std::clamp(sum_effort,-20.0,20.0);

        command_interfaces_[i].set_value(sum_effort * ramp);
            
    }
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration DogController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name_) {
        //cfg.names.push_back(name + "/position");
        //cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration DogController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}
} // namespace dog_controller

PLUGINLIB_EXPORT_CLASS(dog_controller::DogController, controller_interface::ControllerInterface)