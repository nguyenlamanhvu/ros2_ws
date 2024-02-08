//NO OOP
/*
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace rclcpp;
using namespace std;

int main(int argc, char **argv)
{
    init(argc, argv);
    if (argc != 3) {
        RCLCPP_ERROR(get_logger("rclcpp"), "usage: add_two_ints_client X Y");
        return 1;
    }
    auto node = make_shared<AddTwoIntsClientNode>("add_two_ints_client"); 
    auto client_ = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");    // return: sharedptr
    while (!client_->wait_for_service(chrono::seconds(1)))
    {
        RCLCPP_WARN(node->get_logger(),"Waiting for server add two ints...");
    }
    //After server had been stared
    //Create SharePtr request by make_shared<>() function
    auto request = make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);

    auto future =  client_->async_send_request(request);
    if(spin_until_future_complete(node,future) == executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(),"%d + %d = %d",request->a,request->b,future.get()->sum);
    }
    else {
        RCLCPP_ERROR(node->get_logger(),"Error while calling service.");
    }
    
    shutdown();
    return 0;
}
*/

//OOP
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace rclcpp;
using namespace std;

class AddTwoIntsClientNode : public Node 
{
public:
    AddTwoIntsClientNode(int64_t number1, int64_t number2) : Node("add_two_ints_client") 
    {
        threads_.push_back(thread(bind(&AddTwoIntsClientNode::call_add_two_ints,this,number1,number2)));
    }

private:
    // this function should be called in different thread because spin(node)
    void call_add_two_ints(int64_t a, int64_t b)        
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while (!client_->wait_for_service(chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(),"Waiting for server add two ints...");
        }
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        auto future = client_->async_send_request(request);     //future: sharedfuture not sharedptr

        try
        {
            auto response = future.get();       //this will block the thread
            RCLCPP_INFO(this->get_logger(),"%d + %d = %d",a,b,response->sum);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"Service call failed.");
        }
    }
    Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_; 
    vector<thread> threads_;   
};

int main(int argc, char **argv)
{
    init(argc, argv);
    if (argc != 3) {
        RCLCPP_ERROR(get_logger("rclcpp"), "usage: add_two_ints_client X Y");
        return 1;
    }
    auto node = std::make_shared<AddTwoIntsClientNode>(atoll(argv[1]),atoll(argv[2])); 
    spin(node);
    shutdown();
    return 0;
}