#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interface/srv/volume.hpp"

class SphereVolume : public rclcpp::Node {
public:
    SphereVolume() : Node("sphere_volume") {
        service_ = this->create_service<tutorial_interface::srv::Volume>(
            "volume", std::bind(&SphereVolume::calculate_volume, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Sphere volume has been started");
    }

private:
    void calculate_volume(
        const tutorial_interface::srv::Volume::Request::SharedPtr request,
        tutorial_interface::srv::Volume::Response::SharedPtr response) const {
        response->volume = 4.0 / 3.0 * M_PI * pow(request->spr.radius, 3);
        RCLCPP_INFO(this->get_logger(), "Incoming request\nradius: %f", request->spr.radius);
        RCLCPP_INFO(this->get_logger(), "Sending back response: %f", response->volume);
    }

    rclcpp::Service<tutorial_interface::srv::Volume>::SharedPtr service_;
};

int main(){
    rclcpp::init(0, nullptr);
    rclcpp::spin(std::make_shared<SphereVolume>());
    rclcpp::shutdown();
    return 0;
}