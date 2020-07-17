#include <sg/sg_control.hpp>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto sg = std::make_shared<SmartGripper>("sg", "192.168.125.1");
    sg->Init();
    rclcpp::spin(sg->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}