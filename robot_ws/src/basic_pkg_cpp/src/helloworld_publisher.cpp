#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/string.hpp" // string type interface

using namespace std::chrono_literals; // time namespace

class HelloworldPublisher : public rclcpp::Node // inherit Node class
{
public:
    HelloworldPublisher()
    : Node("helloworld_publisher"), count_(0) // Node class constructor, count init
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // QOS setting depth=10
        helloworld_publisher_ = this->create_publisher<std_msgs::msg::String>( // topic type
            "helloworld", qos_profile // topic name, QoS setting
        ); // create publisher
        timer_ = this->create_wall_timer( // callback function every 1 second
            1s, std::bind(&HelloworldPublisher::publish_helloworld_msg, this)
        );
    }
private:
    void publish_helloworld_msg() // callback function
    {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello World: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Message published: '%s", msg.data.c_str()); // logging
        helloworld_publisher_->publish(msg);
    }
    // private member variable initialization
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr helloworld_publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloworldPublisher>(); // shared pointer node
    rclcpp::spin(node); // run callback function inside node
    rclcpp::shutdown();
    return 0;
}