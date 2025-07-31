#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('human_detection_yolo')
    
    # Launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to YOLO model file'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='YOLO confidence threshold'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic from camera'
    )
    
    detection_topic_arg = DeclareLaunchArgument(
        'detection_topic',
        default_value='/human_detections',
        description='Output detection topic'
    )
    
    # YOLO Detector Node
    yolo_detector_node = Node(
        package='human_detection_yolo',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[
            {
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'image_topic': LaunchConfiguration('image_topic'),
                'detection_topic': LaunchConfiguration('detection_topic')
            }
        ],
        output='screen',
        remappings=[
            # สามารถ remap topics ได้ที่นี่หากต้องการ
        ]
    )
    
    # Human Detection Subscriber Node
    human_subscriber_node = Node(
        package='human_detection_yolo',
        executable='human_subscriber',
        name='human_detection_subscriber',
        parameters=[
            {
                'detection_topic': LaunchConfiguration('detection_topic'),
                'max_warning_level': 10,
                'warning_increment': 1,
                'warning_decrement': 2
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        model_path_arg,
        confidence_threshold_arg,
        image_topic_arg,
        detection_topic_arg,
        yolo_detector_node,
        human_subscriber_node
    ])