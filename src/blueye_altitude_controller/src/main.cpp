#include "blueye_altitude_controller/altitude_controller.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AltitudeController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}