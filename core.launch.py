from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_openai',
            default_value='false',
            description='Whether to use OpenAI API'
        ),
        
        DeclareLaunchArgument(
            'quantized_model_path',
            default_value='./quantized_mistral',
            description='Path to quantized LLM model'
        ),
        
        Node(
            package='ns_os',
            executable='llm_bridge',
            name='neuro_controller',
            parameters=[{
                'use_openai': LaunchConfiguration('use_openai'),
                'quantized_model_path': LaunchConfiguration('quantized_model_path')
            }],
            output='screen'
        ),
        
        Node(
            package='ns_os',
            executable='swarm_manager',
            name='swarm_engine',
            output='screen'
        ),
        
        Node(
            package='ns_os',
            executable='bio_fusion',
            name='bio_fusion',
            output='screen'
        )
    ])
