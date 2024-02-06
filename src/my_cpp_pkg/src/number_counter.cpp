#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
using namespace rclcpp;
using namespace std;

class NumberCouterNode : public Node 
{
public:
    NumberCouterNode() : Node("number_counter") , counter(0)
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number",10,
                                                bind(&NumberCouterNode::callback_number,this,placeholders::_1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count",10);
        RCLCPP_INFO(this->get_logger(),"Number Counter has been started.");
    }

private:
    void callback_number(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter+=msg->data;
        RCLCPP_INFO(this->get_logger(),"%d",counter);
        NumberCouterNode::publishNumberCounts();
    }
    void publishNumberCounts()
    {
        auto msg = example_interfaces::msg::Int64();        //constructor
        msg.data = counter;
        publisher_->publish(msg);
    }
    Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    int64_t counter;
};

int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = std::make_shared<NumberCouterNode>(); 
    spin(node);
    shutdown();
    return 0;
}