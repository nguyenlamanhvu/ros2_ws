#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

using namespace rclcpp;
using namespace std;

class HardwareStatusPublisherNode : public Node 
{
public:
    HardwareStatusPublisherNode() : Node("hardware_status_publisher") 
    {
        hw_status_publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status",10);
        timer_ = this->create_wall_timer(chrono::seconds(1),
                                         bind(&HardwareStatusPublisherNode::callback_hw_status_publisher,this));   
        RCLCPP_INFO(this->get_logger(),"Hardware Status Publisher has been started.");
    }

private:
    void callback_hw_status_publisher()
    {
        my_robot_interfaces::msg::HardwareStatus msg;
        msg.temperature = 45;
        msg.are_motors_ready = true;
        msg.debug_message = "Nothing special here";
        hw_status_publisher_->publish(msg);
    }
    Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr hw_status_publisher_;
    TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>(); 
    spin(node);
    shutdown();
    return 0;
}