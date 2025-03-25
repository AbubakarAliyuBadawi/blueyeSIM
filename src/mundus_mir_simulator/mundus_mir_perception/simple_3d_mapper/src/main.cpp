#include "simple_3d_mapper/simple_3d_mapper.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<simple_3d_mapper>());
    rclcpp::shutdown();
    return 0;
}