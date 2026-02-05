#!/usr/bin/env python3

# Copyright 2025 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the drink order behavior tree example.
    
    This example demonstrates a simple HRI interaction:
    1. Robot asks what the person wants to drink
    2. Robot listens to the response
    3. Robot repeats what it heard
    
    Requires simple_hri TTS and ASR services to be running.
    """
    
    # Behavior Tree node
    drink_order_bt_node = Node(
        package='bt_examples',
        executable='drink_order_bt_example',
        name='drink_order_bt',
        output='screen',
        parameters=[],
        remappings=[]
    )
    
    return LaunchDescription([
        drink_order_bt_node
    ])
