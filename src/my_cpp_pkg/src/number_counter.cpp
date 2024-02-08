#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace rclcpp;
using namespace std;
using placeholders::_1;
using placeholders::_2;

class NumberCouterNode : public Node 
{
public:
    NumberCouterNode() : Node("number_counter") , counter(0)
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number",10,
                                                bind(&NumberCouterNode::callback_number,this,placeholders::_1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count",10);
        service_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter",
            bind(&NumberCouterNode::callback_reset_counter,this,_1,_2));
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
    void callback_reset_counter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                                const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if(request->data == true){
            counter = 0;
            response->set__success(true);
            response->message = "Counter has been reset";
            RCLCPP_INFO(this->get_logger(),"Counter has been reset.");
        } 
        else {
            response->set__success(false);
            response->message = "Counter hasn't been reset";
            RCLCPP_INFO(this->get_logger(),"Counter hasn't been reset.");
        }
    }
    Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    Service<example_interfaces::srv::SetBool>::SharedPtr service_;
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