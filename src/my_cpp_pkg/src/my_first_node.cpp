#include "rclcpp/rclcpp.hpp"

using namespace rclcpp;
using namespace std;

class MyNode: public Node       //Node:base class, MyNode:derived class
{
public:
    MyNode(): Node("cpp_test"),counter(1)      //default constructor
    {
        RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");      //RCLCPP_INFO macro ensures every published message is printed to the console
        //Timer
        timer = this->create_wall_timer(chrono::seconds(1),bind(&MyNode::timerCallback,this));
    }
private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello %d",counter);    
        counter++;
    }
    int counter;
    TimerBase::SharedPtr timer;
};

int main(int argc, char **argv){
    init(argc,argv);        
    auto node = make_shared<MyNode>();     //node: shared pointer
    spin(node);
    shutdown();
    return 0;
}