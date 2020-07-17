#include <string>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp> // This is deprecated in newer versions of ROS.
#include <abb_librws/rws_state_machine_interface.h>

class SmartGripper : public rclcpp::Node {
public:

    SmartGripper(std::string node_name, const std::string &robot_ip) : Node(node_name), ip_(robot_ip) {}


    bool Init() {

        

        // This object is our way to interface with the robot. 
        rws_state_machine_interface_  = std::make_shared<abb::rws::RWSStateMachineInterface>(ip_);

        // We check that we can connect.
        auto runtime_info = rws_state_machine_interface_->collectRuntimeInfo();
        RCLCPP_INFO(this->get_logger(), "Connecting to Robot...");
        if (!runtime_info.rws_connected) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "OK!");

        // And we make sure that the StateMachine RAPID task is running! 
        RCLCPP_INFO(this->get_logger(), "Checking for StateMachine...");
        if (!rws_state_machine_interface_->isRAPIDRunning().isTrue()) {
            RCLCPP_ERROR(this->get_logger(), "StateMachine is not running on the robot.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "OK!");

        // Before we can use the gripper, it needs to be calibrated. 
        // Either we can do it programmatically, or we can also calibrate on the 
        // FlexPendant while we also start the StateMachine.
        RCLCPP_INFO(this->get_logger(), "Calibrating SmartGrippers...");
        if (!rws_state_machine_interface_->services().sg().rightCalibrate()) {
            RCLCPP_ERROR(this->get_logger(), "Could not calibrate!");  
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "OK!");


        // We set up a subscription on the topic /grip [std_msgs/msg/Bool]
        // With a callback to grip_callback(std_msgs::msgs::Bool::SharedPtr msg)
        grip_sub_ = this->create_subscription<std_msgs::msg::Bool>("grip", 10, std::bind(&SmartGripper::grip_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Init complete!");
        return true;
    }

private:
    std::string ip_;
    std::shared_ptr<abb::rws::RWSStateMachineInterface> 
    rws_state_machine_interface_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grip_sub_;
    bool grip_callback(std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received grip request: %d", msg->data);
        bool retval;
        // All the boilerplate code is just so that we can execute these commands.
        if (msg->data) { 
            retval = rws_state_machine_interface_->services().sg().rightGripIn();
        } else {
            retval = rws_state_machine_interface_->services().sg().rightGripOut();
        }

        if(!retval) {
            RCLCPP_ERROR(this->get_logger(), "Unable to grip."); 
        }   
        return retval;
    }
}; 
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto sg = std::make_shared<SmartGripper>("sg", "192.168.125.1");
    if (sg->Init()){
        RCLCPP_INFO(sg->get_logger(), "Spinning node!");
        rclcpp::spin(sg->get_node_base_interface());
    } else {
        RCLCPP_ERROR(sg->get_logger(), "Could not initiate. Terminating..");
    }
    rclcpp::shutdown();
    return 0;
}