#include <functional>
#include <memory> 

#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/string.hpp" // string type interface

using std::placeholders::_1; // used in bind function

class HelloworldSubscriber : public rclcpp::Node
{
public:
    HelloworldSubscriber()
    : Node("Helloworld_subscriber") // Node constructor
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // QoS depth=10
        helloworld_subscriber_ = this->create_subscription<std_msgs::msg::String>( // topic type = String
            "helloworld", // topic name
            qos_profile, // QoS setting
            std::bind(&HelloworldSubscriber::subscribe_topic_message, this, _1) // callback function
        );
    }
private:
    // callback function, input message received from publisher
    void subscribe_topic_message(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr helloworld_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloworldSubscriber>(); // shared pointer node
    rclcpp::spin(node); // run callback function inside node
    rclcpp::shutdown();
    return 0;
}