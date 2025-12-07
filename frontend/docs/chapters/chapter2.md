---
sidebar_position: 2
---

# Chapter 2: ROS 2 Fundamentals

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

## Key Differences from ROS 1

- **Middleware**: ROS 2 uses DDS (Data Distribution Service) as its middleware
- **Real-time Support**: Better support for real-time systems
- **Multi-robot Systems**: Improved support for multi-robot systems
- **Security**: Built-in security features
- **Quality of Service**: Configurable QoS policies for different communication needs

## Core Concepts

### Nodes
Nodes are processes that perform computation. In ROS 2, nodes are implemented using client libraries such as `rclcpp` for C++ or `rclpy` for Python.

### Topics and Messages
Topics are named buses over which nodes exchange messages. Messages are the data packets sent from publishers to subscribers.

### Services
Services provide a request/response communication pattern, useful for operations that need immediate responses.

### Actions
Actions are a more sophisticated form of communication that handles long-running tasks with feedback and goal management.

## Setting Up a ROS 2 Workspace

```bash
# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Creating a Simple Publisher

Here's an example of a simple publisher in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Simple Subscriber

And here's a corresponding subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Files

Launch files allow you to start multiple nodes at once and configure their parameters:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='talker',
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener',
        ),
    ])
```

## Best Practices

1. **Node Design**: Keep nodes focused on a single responsibility
2. **Parameter Management**: Use parameters for configuration rather than hardcoded values
3. **Logging**: Implement proper logging for debugging and monitoring
4. **Error Handling**: Implement robust error handling and recovery
5. **Resource Management**: Properly manage resources and clean up when shutting down

## Next Steps

In the next chapter, we'll explore robot simulation environments including Gazebo and Unity.