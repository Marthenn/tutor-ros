#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interface/msg/sphere.hpp"
#include "tutorial_interface/srv/volume.hpp"

class SphereSubscriber : public rclcpp::Node {
public:
    SphereSubscriber() : Node("sphere_subscriber") {
        subscription_ = this->create_subscription<tutorial_interface::msg::Sphere>(
                "sphere", 10, std::bind(&SphereSubscriber::topic_callback, this, std::placeholders::_1));
        client_ = this->create_client<tutorial_interface::srv::Volume>("volume");

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
    }

private:
    void topic_callback(const tutorial_interface::msg::Sphere::SharedPtr message) {
        // Handle incoming Sphere messages
        RCLCPP_INFO(this->get_logger(), "Received Sphere message\nradius: %f", message->radius);

        auto request = std::make_shared<tutorial_interface::srv::Volume::Request>();
        request->spr = *message;

        auto result = client_->async_send_request(request, std::bind(&SphereSubscriber::service_callback, this, std::placeholders::_1));
    }

    void service_callback(rclcpp::Client<tutorial_interface::srv::Volume>::SharedFuture future) {
    if (future.valid()) {
        if (future.get()) {
            RCLCPP_INFO(this->get_logger(), "Volume of the sphere is %f", future.get()->volume);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service 'volume' returned an invalid response");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service 'volume'");
    }
}


    rclcpp::Subscription<tutorial_interface::msg::Sphere>::SharedPtr subscription_;
    rclcpp::Client<tutorial_interface::srv::Volume>::SharedPtr client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SphereSubscriber>());
    rclcpp::shutdown();
    return 0;
}