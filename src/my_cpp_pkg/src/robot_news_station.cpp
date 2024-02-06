#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"  // equal to "from example_interfaces.msg import String"

using namespace rclcpp;
using namespace std;

class RobotNewsStationNode : public Node 
{
public:
    RobotNewsStationNode() : Node("robot_news_station") , robot_names_("C3D3")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news",10);
        timer_ = this->create_wall_timer(chrono::milliseconds(500),
                                            bind(&RobotNewsStationNode::publishNews,this));
        RCLCPP_INFO(this->get_logger(),"Robot News Station has been started.");
    }

private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::String();       //use String type in example_interfaces/msg
        msg.data = string("Hi, this is ") + robot_names_ + string(" from the Robot News Station");
        publisher_->publish(msg);
    }
    string robot_names_;
    Publisher<example_interfaces::msg::String>::SharedPtr publisher_;       //publisher_ : SharedPtr
    TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>(); 
    spin(node);
    shutdown();
    return 0;
}