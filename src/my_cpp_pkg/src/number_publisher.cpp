#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
using namespace rclcpp;
using namespace std;

class NumberPublisherNode : public Node 
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        this->declare_parameter("publish_number",2);
        this->declare_parameter("frequency",1.0);
        number_ = this->get_parameter("publish_number").as_int();   //type of data: as_type()
        frequency_ = this->get_parameter("frequency").as_double();

        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number",10);
        timer_ = this->create_wall_timer(chrono::milliseconds(int (1000.0 / frequency_)),
                                        bind(&NumberPublisherNode::publishNumbers,this));
        RCLCPP_INFO(this->get_logger(),"Number Publisher has been started.");
    }

private:
    void publishNumbers()
    {
        example_interfaces::msg::Int64 counter_;    //constructor
        counter_.data = number_;
        publisher_->publish(counter_);
    }
    Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;        //SharedPtr is important (don't forget it in C++)
    TimerBase::SharedPtr timer_;
    int number_;
    double frequency_;
};

int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>(); 
    spin(node);
    shutdown();
    return 0;
}