#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace rclcpp;
using namespace std;
using placeholders::_1;
using placeholders::_2;

class AddTwoIntsServerNode : public Node 
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server") 
    {
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                                                    bind(&AddTwoIntsServerNode::callback_add_two_ints,this,_1,_2));
        RCLCPP_INFO(this->get_logger(),"Add Two Ints Server has been started.");
    }
private:
    //const de cho dia chi cua doi tuong khong bi doi, gia tri van co the bi thay doi
    void callback_add_two_ints(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                               const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(),"%d + %d = %d",request->a,request->b,response->sum);
    }
    Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>(); 
    spin(node);
    shutdown();
    return 0;
}