#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace rclcpp;
using namespace std;

class SmartphoneNode : public Node // MODIFY NAME
{
public:
    SmartphoneNode() : Node("smartphone") // MODIFY NAME
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::String>("robot_news",10,
                                                        bind(&SmartphoneNode::callback_robot_news,this,placeholders::_1));      //do co tham so o callback_robot_news nen can placeholders
        RCLCPP_INFO(this->get_logger(),"Smartphone has been started.");
    }

private:
    void callback_robot_news(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s" , msg->data.c_str());
    }
    Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = std::make_shared<SmartphoneNode>(); // MODIFY NAME
    spin(node);
    shutdown();
    return 0;
}