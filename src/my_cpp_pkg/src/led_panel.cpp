#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_state.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace rclcpp;
using namespace std;
using placeholders::_1;
using placeholders::_2;

class LedPanelNode : public Node 
{
public:
    LedPanelNode() : Node("led_panel")//, led_states(3,0)     //init led_states[0],led_states[1],led_states[2]: 0
    {
        this->declare_parameter("led_states",vector<int64_t>{0,0,0});
        led_states = this->get_parameter("led_states").as_integer_array();
        led_panel_publisher_ = this->create_publisher<my_robot_interfaces::msg::LedState>("led_panel_state",10);
        led_state_timer_ = this->create_wall_timer(chrono::seconds(4),bind(&LedPanelNode::led_panel_publish,this));
        set_led_service_ = this->create_service<my_robot_interfaces::srv::SetLed>("set_led",
                                                bind(&LedPanelNode::callback_set_led,this,_1,_2));
        RCLCPP_INFO(this->get_logger(),"Led panel has been started.");
    }

private:
    void led_panel_publish()
    {
        auto msg = my_robot_interfaces::msg::LedState();
        msg.ledstatus = led_states;
        led_panel_publisher_->publish(msg);
    }
    void callback_set_led(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                          const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        if(request->led_number > (int64_t)led_states.size() || request->led_number <= 0)
        {
            response->success = false;
            return;
        }
        led_states.at(request->led_number-1) = request->state ? 1:0;
        response->success = true;
        led_panel_publish();
    }
    vector<int64_t> led_states;
    Publisher<my_robot_interfaces::msg::LedState>::SharedPtr led_panel_publisher_;
    Service<my_robot_interfaces::srv::SetLed>::SharedPtr set_led_service_;
    TimerBase::SharedPtr led_state_timer_;
};

int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = std::make_shared<LedPanelNode>(); 
    spin(node);
    shutdown();
    return 0;
}