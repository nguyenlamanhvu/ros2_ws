#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace rclcpp;
using namespace std;

class ResetCounterNode : public Node 
{
public:
    ResetCounterNode() : Node("reset_counter") 
    {
        threads_.push_back(thread(bind(&ResetCounterNode::call_reset_counter,this)));
    }

private:
    void call_reset_counter()
    {
        client_ = this->create_client<example_interfaces::srv::SetBool>("reset_counter");
        while (!client_->wait_for_service(chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(),"Waiting for server number counter.");
        }
        auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
        request->data = true;

        auto future = client_->async_send_request(request);
        try
        {
            auto response = future.get();
            if(response->success == true)
            {
                RCLCPP_INFO(this->get_logger(),"Counter has been reset.");
            } 
            else 
            {
                RCLCPP_INFO(this->get_logger(),"Counter hasn't been reset.");
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"Service call failed.");
        }
        
    }
    Client<example_interfaces::srv::SetBool>::SharedPtr client_;
    vector<thread> threads_; 
};

int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = std::make_shared<ResetCounterNode>(); 
    spin(node);
    shutdown();
    return 0;
}