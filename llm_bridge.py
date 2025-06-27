#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from .openai_integration import OpenAIHandler
from .local_llm import LocalLLM

class LLMBridge(Node):
    def __init__(self):
        super().__init__('llm_bridge')
        
        # Publishers and Subscribers
        self.command_sub = self.create_subscription(
            String, '/ns/command', self.command_callback, 10)
        self.action_pub = self.create_publisher(String, '/ns/action', 10)
        self.feedback_pub = self.create_publisher(String, '/ns/feedback', 10)
        
        # LLM Handlers
        self.openai = OpenAIHandler()
        self.local_llm = LocalLLM()
        
        self.get_logger().info("NeuroController LLM Bridge initialized")

    def command_callback(self, msg):
        try:
            command = msg.data
            self.get_logger().info(f"Received command: {command}")
            
            # Determine which LLM to use
            if self._is_complex_command(command):
                response = self.openai.process_command(command)
            else:
                response = self.local_llm.process_command(command)
            
            # Publish action
            action_msg = String()
            action_msg.data = json.dumps(response)
            self.action_pub.publish(action_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing command: {str(e)}")
            feedback_msg = String()
            feedback_msg.data = f"Error: {str(e)}"
            self.feedback_pub.publish(feedback_msg)

    def _is_complex_command(self, command):
        # Simple heuristic to determine command complexity
        complex_keywords = ['сложный', 'авария', 'помощь', 'экстрен', 'найти']
        return any(keyword in command.lower() for keyword in complex_keywords)

def main(args=None):
    rclpy.init(args=args)
    node = LLMBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
