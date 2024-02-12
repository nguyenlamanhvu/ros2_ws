from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("number","my_number")     #tupple

    number_publisher_node = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="my_publisher",
        remappings=[
            remap_number_topic
        ],
        parameters=[                                #array
            {"publish_number": 5},
            {"frequency": 5.0}
        ]
    )

    counter_publisher_node = Node(
        package="my_cpp_pkg",
        executable="number_counter",
        name="my_counter",
        remappings=[
            remap_number_topic,
            ("number_counter","my_counter")
        ] 
    )

    ld.add_action(number_publisher_node)
    ld.add_action(counter_publisher_node)
    
    return ld