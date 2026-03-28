/**
   Twist Publisher Controller
   @author Kenta Suzuki
*/

#include <cnoid/Joystick>
#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <memory>
#include <thread>
#include <mutex>

class TwistPublisherController : public cnoid::SimpleController
{
public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override;
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;
    virtual void unconfigure() override;

private:
    enum ControlMode { Mode1, Mode2 };

    cnoid::Joystick joystick;

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::executors::StaticSingleThreadedExecutor::UniquePtr executor;
    std::thread executorThread;
    std::mutex commandMutex;

    int currentMode;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TwistPublisherController)

bool TwistPublisherController::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());

    publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this](){ executor->spin(); });

    return true;
}

bool TwistPublisherController::initialize(cnoid::SimpleControllerIO* io)
{
    currentMode = Mode2;

    for(auto opt : io->options()) {
        if(opt == "mode1") {
            currentMode = Mode1;
        }
        if(opt == "mode2") {
            currentMode = Mode2;
        }
    }

    return true;
}

bool TwistPublisherController::control()
{
    joystick.readCurrentState();

    double pos[4];
    for(int i = 0; i < 4; ++i) {
        pos[i] = joystick.getPosition(i);
        if(fabs(pos[i]) < 0.2) {
            pos[i] = 0.0;
        }
    }

    // vel[z, r, p, y]
    static double vel[] = { 2.0, 2.0, 2.0, 1.047 };

    {
        std::lock_guard<std::mutex> lock(commandMutex);
        auto message = geometry_msgs::msg::Twist();
        if(currentMode == Mode1) {
            message.linear.x = vel[2] * pos[1] * -1.0;
            message.linear.y = vel[1] * pos[2] * -1.0;
            message.linear.z = vel[0] * pos[3] * -1.0;
            message.angular.x = 0.0;
            message.angular.y = 0.0;
            message.angular.z = vel[3] * pos[0] * -1.0;
        } else if(currentMode == Mode2) {
            message.linear.x = vel[2] * pos[1] * -1.0;
            message.linear.y = vel[1] * pos[0] * -1.0;
            message.linear.z = vel[0] * pos[3] * -1.0;
            message.angular.x = 0.0;
            message.angular.y = 0.0;
            message.angular.z = vel[3] * pos[2] * -1.0;
        }
        publisher->publish(message);
    }

    return true;
}

void TwistPublisherController::unconfigure()
{
    if(executor) {
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}