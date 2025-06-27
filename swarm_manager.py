#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from .p2p_network import P2PNetwork
from .task_allocator import TaskAllocator

class SwarmManager(Node):
    def __init__(self):
        super().__init__('swarm_manager')
        
        # Initialize components
        self.p2p_network = P2PNetwork()
        self.task_allocator = TaskAllocator()
        
        # Publishers and Subscribers
        self.action_sub = self.create_subscription(
            String, '/ns/action', self.action_callback, 10)
        self.status_pub = self.create_publisher(String, '/ns/swarm_status', 10)
        
        # Swarm state
        self.agents = {}
        self.tasks = []
        
        self.get_logger().info("Swarm Manager initialized")

    def action_callback(self, msg):
        try:
            action = json.loads(msg.data)
            self.get_logger().info(f"Processing action: {action}")
            
            # Allocate tasks
            allocated = self.task_allocator.allocate(action, self.agents)
            
            # Send tasks to agents
            for agent_id, task in allocated.items():
                self.p2p_network.send_task(agent_id, task)
                
            # Update status
            status = {
                'action': action['action'],
                'allocated_tasks': len(allocated),
                'available_agents': len(self.agents)
            }
            status_msg = String()
            status_msg.data = json.dumps(status)
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing action: {str(e)}")

    def update_agent_status(self, agent_id, status):
        self.agents[agent_id] = status
        self.get_logger().info(f"Agent {agent_id} status updated: {status}")

def main(args=None):
    rclpy.init(args=args)
    node = SwarmManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
