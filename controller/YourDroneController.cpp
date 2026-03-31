/**
   YourDrone Controller
   @author Kenta Suzuki
*/

#include <cnoid/EigenUtil>
#include <cnoid/RateGyroSensor>
#include <cnoid/Rotor>
#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <thread>

namespace {

const double delta[] = { 2.000, 2.0, 2.0, 1.047 };
const double pgain[] = { 0.020, 0.1, 0.1, 0.010 };
const double dgain[] = { 0.005, 0.1, 0.1, 0.001 };

} // namespace

class YourDroneController : public cnoid::SimpleController
{
public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override;
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool start() override;
    virtual bool control() override;
    virtual void stop() override;
    virtual void unconfigure() override;

private:
    cnoid::SimpleControllerIO* io;
    cnoid::BodyPtr ioBody;
    cnoid::DeviceList<cnoid::Rotor> rotors;
    cnoid::RateGyroSensor* gyroSensor;
    cnoid::Vector4 zref, zprev;
    cnoid::Vector4 dzref, dzprev;
    cnoid::Vector2 xref, xprev;
    cnoid::Vector2 dxref, dxprev;
    double timeStep;
    bool is_powered_on;

    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
    geometry_msgs::msg::Twist command;
    rclcpp::executors::StaticSingleThreadedExecutor::UniquePtr executor;
    std::thread executorThread;
    std::mutex commandMutex;
    std::string topic_name;
    std::string controller_name;

    cnoid::Vector4 getZRPY();
    cnoid::Vector2 getXY();
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(YourDroneController)

bool YourDroneController::configure(cnoid::SimpleControllerConfig* config)
{
    controller_name = config->controllerName();
    return true;
}

bool YourDroneController::initialize(cnoid::SimpleControllerIO* io)
{
    this->io = io;
    ioBody = io->body();
    rotors = io->body()->devices();
    gyroSensor = ioBody->findDevice<cnoid::RateGyroSensor>("GyroSensor");
    is_powered_on = true;

    topic_name.clear();
    bool is_topic = false;
    for(auto opt : io->options()) {
        if(opt == "topic") {
            is_topic = true;
        } else if(is_topic) {
            topic_name = opt;
            break;
        }
    }
    if(topic_name.empty()) {
        topic_name = "cmd_vel";
    }

    io->enableInput(ioBody->rootLink(), cnoid::Link::LinkPosition);
    io->enableInput(gyroSensor);

    for(auto& rotor : rotors) {
        io->enableInput(rotor);
    }

    zref = zprev = getZRPY();
    dzref = dzprev = cnoid::Vector4::Zero();
    xref = xprev = getXY();
    dxref = dxprev = cnoid::Vector2::Zero();

    timeStep = io->timeStep();

    return true;
}

bool YourDroneController::start()
{
    node = std::make_shared<rclcpp::Node>(controller_name);

    subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        topic_name, 1, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(commandMutex);
            command = *msg;
        });

    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this]() { executor->spin(); });

    return true;
}

bool YourDroneController::control()
{
    // vel[z, r, p, y]
    static double vel[] = { 2.0, 2.0, 2.0, 1.047 };
    double val[] = { command.linear.z, command.linear.y, command.linear.x, command.angular.z };
    for(int i = 0; i < 4; ++i) {
        if(val[i] > 0.0) {
            val[i] = val[i] > vel[i] ? vel[i] : val[i];
        } else if(val[i] < 0.0) {
            val[i] = val[i] < -vel[i] ? -vel[i] : val[i];
        }
        val[i] = val[i] / vel[i] * -1.0;
    }

    double pos[4];
    for(int i = 0; i < 4; ++i) {
        pos[i] = val[i];
        if(fabs(pos[i]) < 0.2) {
            pos[i] = 0.0;
        }
    }

    cnoid::Vector4 fz = cnoid::Vector4::Zero();
    cnoid::Vector4 z = getZRPY();
    cnoid::Vector4 dz = (z - zprev) / timeStep;
    if(gyroSensor) {
        cnoid::Vector3 w = gyroSensor->w();
        dz[1] = w[0];
        dz[2] = w[1];
        dz[3] = w[2];
    }
    cnoid::Vector4 ddz = (dz - dzprev) / timeStep;

    cnoid::Vector2 x = getXY();
    cnoid::Vector2 dx = (x - xprev) / timeStep;
    cnoid::Vector2 ddx = (dx - dxprev) / timeStep;
    cnoid::Vector2 dx_local = Eigen::Rotation2Dd(-z[3]) * dx;
    cnoid::Vector2 ddx_local = Eigen::Rotation2Dd(-z[3]) * ddx;

    double cc = cos(z[1]) * cos(z[2]);
    double gc = ioBody->mass() * 9.80665 / 4.0 / cc;

    if((fabs(cnoid::degree(z[1])) > 45.0) || (fabs(cnoid::degree(z[2])) > 45.0)) {
        is_powered_on = false;
    }

    static const double P = 1.0;
    static const double D = 1.0;

    for(int i = 0; i < 4; ++i) {
        if(i == 0 || i == 3) {
            dzref[i] = -delta[i] * pos[i];
            fz[i] = (dzref[i] - dz[i]) * pgain[i] + (0.0 - ddz[i]) * dgain[i];
        } else {
            int j = i - 1;
            dxref[j] = -delta[i] * pos[i];
            zref[i] = P * (dxref[j] - dx_local[1 - j]) + D * (0.0 - ddx_local[1 - j]);
            zref[i] = (i != 1 ? 1.0 : -1.0) * zref[i];
            fz[i] = (zref[i] - z[i]) * pgain[i] + (0.0 - dz[i]) * dgain[i];
        }
    }
    zprev = z;
    dzprev = dz;
    xprev = x;
    dxprev = dx;

    static const double ATD[] = { -1.0, 1.0, -1.0, 1.0 };
    double thr[4] = { 0.0 };
    if(is_powered_on) {
        thr[0] = gc + fz[0] - fz[1] - fz[2] - fz[3];
        thr[1] = gc + fz[0] + fz[1] - fz[2] + fz[3];
        thr[2] = gc + fz[0] + fz[1] + fz[2] - fz[3];
        thr[3] = gc + fz[0] - fz[1] + fz[2] + fz[3];
    }

    for(size_t i = 0; i < rotors.size(); ++i) {
        cnoid::Rotor* rotor = rotors[i];
        rotor->force() = thr[i];
        rotor->torque() = ATD[i] * thr[i];
        rotor->notifyStateChange();
    }

    return true;
}

void YourDroneController::stop()
{
    if(executor) {
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}

void YourDroneController::unconfigure()
{
}

cnoid::Vector4 YourDroneController::getZRPY()
{
    auto T = ioBody->rootLink()->position();
    double z = T.translation().z();
    cnoid::Vector3 rpy = cnoid::rpyFromRot(T.rotation());
    return cnoid::Vector4(z, rpy[0], rpy[1], rpy[2]);
}

cnoid::Vector2 YourDroneController::getXY()
{
    auto p = ioBody->rootLink()->translation();
    return cnoid::Vector2(p.x(), p.y());
}