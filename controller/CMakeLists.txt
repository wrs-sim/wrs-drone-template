choreonoid_add_simple_controller(YourDroneController YourDroneController.cpp)
target_link_libraries(YourDroneController CnoidCFDPlugin)

ament_target_dependencies(
    YourDroneController rclcpp geometry_msgs sensor_msgs)

choreonoid_add_simple_controller(
    TwistPublisherController TwistPublisherController.cpp)

ament_target_dependencies(
    TwistPublisherController rclcpp geometry_msgs sensor_msgs)