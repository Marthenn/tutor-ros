#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interface/msg/sphere.hpp"

class SpherePublisher : public rclcpp::Node {
public:
    SpherePublisher() : Node("sphere_publisher") {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Radius of the sphere";

        this->declare_parameter("radius", 1.0);
        publisher_ = this->create_publisher<tutorial_interface::msg::Sphere>("sphere", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&SpherePublisher::timer_callback, this));
    }
private:
    void timer_callback() {
        auto message = tutorial_interface::msg::Sphere();
        auto radius = this->get_parameter("radius").as_double();
        if (radius <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Radius must be positive, it is now randomized between 1 and 100");
            message.radius = random() % 100 + 1;
        } else {
            message.radius = this->get_parameter("radius").as_double();
        }
        message.center.x = random() % 100;
        message.center.y = random() % 100;
        message.center.z = random() % 100;
        RCLCPP_INFO(this->get_logger(), "Publishing: radius = %f", message.radius);
        RCLCPP_INFO(this->get_logger(), "Publishing: Point = (%f, %f, %f)", message.center.x, message.center.y, message.center.z);
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tutorial_interface::msg::Sphere>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpherePublisher>());
    rclcpp::shutdown();
    return 0;
}