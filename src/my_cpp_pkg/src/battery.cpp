#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace rclcpp;
using namespace std;

class BatteryNode : public Node 
{
public:
    BatteryNode() : Node("battery"), battery_state(false)
    {
        //get init time
        last_time_battery_state_changed_ = this->get_clock()->now().seconds();
        timer_ = this->create_wall_timer(chrono::milliseconds(100),bind(&BatteryNode::check_led_state,this));
        RCLCPP_INFO(this->get_logger(),"Battery has been started.");
    }

private:
    void set_led(int64_t led_number, bool state)
    {
        threads_.push_back(thread(bind(&BatteryNode::call_battery_client,this,led_number,state)));
    }

    void call_battery_client(int64_t led_number, bool state)
    {
        battery_client_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
        while (!battery_client_->wait_for_service(chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(),"Waiting for set led service...");
        }
        //init request
        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_number = led_number;
        request->state = state;

        auto future = battery_client_->async_send_request(request);
        try
        {
            auto response = future.get();       //this will block the thread
            RCLCPP_INFO(this->get_logger(),"%d",response->success);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"Service call failed.");
        }
    }

    void check_led_state()
    {
        double clock = this->get_clock()->now().seconds();
        if(battery_state)   // battery is full
        {
            if(clock - last_time_battery_state_changed_ > 4.0){
                RCLCPP_INFO(this->get_logger(),"Battery is empty! Charging now...");
                last_time_battery_state_changed_ = clock;
                battery_state = false;   // change battery full to empty state
                set_led(3,true);
            }
        }
        else        // battery is empty
        {
            if(clock - last_time_battery_state_changed_ > 6.0){
                RCLCPP_INFO(this->get_logger(),"Battery is full! Stop charging now...");
                last_time_battery_state_changed_ = clock;
                battery_state = true;   // change battery empty to full state
                set_led(3,false);
            }
        }
        
    }

    Client<my_robot_interfaces::srv::SetLed>::SharedPtr battery_client_;
    TimerBase::SharedPtr timer_;
    vector<thread> threads_;
    bool battery_state;     //true: fully charge, false: empty
    double last_time_battery_state_changed_;
};

int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = std::make_shared<BatteryNode>(); 
    spin(node);
    shutdown();
    return 0;
}