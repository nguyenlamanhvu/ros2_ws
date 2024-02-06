#include "rclcpp/rclcpp.hpp"

using namespace rclcpp;
using namespace std;

class MyCustomNode : public Node // MODIFY NAME
{
public:
    MyCustomNode() : Node("node_name") // MODIFY NAME
    {
    }

private:
};

int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
    spin(node);
    shutdown();
    return 0;
}