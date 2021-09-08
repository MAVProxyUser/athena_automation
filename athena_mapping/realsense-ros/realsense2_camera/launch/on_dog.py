# Copyright (c) 2018 Intel Corporation
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

"""Launch realsense2_camera node."""
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

configurable_parameters = [{'name': 'serial_no',                    'default': '', 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': '', 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': '', 'description': 'choose device by type'},
                           {'name': 'json_file_path',               'default': '', 'description': 'allows advanced configuration'},    
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'fisheye_width',                'default': '640', 'description': 'fisheye width'},
                           {'name': 'fisheye_height',               'default': '480', 'description': 'fisheye width'},
                           {'name': 'enable_fisheye',               'default': 'false', 'description': 'enable fisheye stream'},
                           {'name': 'depth_width',                  'default': '640', 'description': 'depth image width'},                           
                           {'name': 'depth_height',                 'default': '480', 'description': 'depth image height'},     
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'infra_width',                  'default': '640', 'description': 'infra width'},
                           {'name': 'infra_height',                 'default': '480', 'description': 'infra width'},
                           {'name': 'enable_infra',                 'default': 'false', 'description': ''},  
                           {'name': 'enable_infra1',                'default': 'true', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'true', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'color_width',                  'default': '640', 'description': 'color image width'},                         
                           {'name': 'color_height',                 'default': '480', 'description': 'color image height'},                           
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'fisheye_fps',                  'default': '15.', 'description': ''},
                           {'name': 'depth_fps',                    'default': '15.', 'description': ''},
                           {'name': 'infra_fps',                    'default': '15.', 'description': ''},
                           {'name': 'color_fps',                    'default': '15.', 'description': ''},
                           {'name': 'gyro_fps',                     'default': '200.', 'description': ''},
                           {'name': 'accel_fps',                    'default': '100.', 'description': ''},
                           {'name': 'color_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'confidence_qos',               'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'depth_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'fisheye_qos',                  'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'infra_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'enable_gyro',                  'default': 'true', 'description': ''},                           
                           {'name': 'enable_accel',                 'default': 'true', 'description': ''},         
                           {'name': 'enable_pointcloud',            'default': 'false', 'description': 'enable pointcloud'},
                           {'name': 'pointcloud_texture_stream',    'default': 'RS2_STREAM_COLOR', 'description': 'testure stream for pointcloud'}, 
                           {'name': 'pointcloud_texture_index',     'default': '0', 'description': 'testure stream index for pointcloud'}, 
                           {'name': 'enable_sync',                  'default': 'true', 'description': 'new add'},                           
                           {'name': 'align_depth',                  'default': 'true', 'description': ''},    
                           {'name': 'filters',                      'default': '', 'description': ''},                           
                           {'name': 'clip_distance',                'default': '-2.', 'description': ''},                           
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': ''},                           
                           {'name': 'initial_reset',                'default': 'true', 'description': ''}, 
                           {'name': 'unite_imu_method',             'default': 'copy', 'description': '[copy|linear_interpolation]'},
                           {'name': 'topic_odom_in',                'default': 'odom_in', 'description': 'topic for T265 wheel odometry'},
                           {'name': 'calib_odom_file',              'default': '', 'description': ''}, 
                           {'name': 'allow_no_texture_points',      'default': 'false', 'description': ''},
                           {'name': 'camera',                       'default': 'camera', 'description': ''},
                           {'name': 'tf_prefix',                    'default': 'camera', 'description': ''},
                           {'name': 'external_manager',             'default': 'false', 'description': ''},
                           {'name': 'manager',                      'default': 'realsense2_camera_manager', 'description': ''},
                           {'name': 'publish_tf',                   'default': 'true', 'description': ''},
                           {'name': 'tf_publish_rate',              'default': '10.', 'description': ''},
                           {'name': 'publish_odom_tf',              'default': 'true', 'description': ''},
                           {'name': 'allow_no_texture_points',      'default': 'false', 'description': ''}, 
                           {'name': 'namespace2',                   'default': '/camera', 'description': '/ is needed'},
                           {'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'config_file',                  'default': '', 'description': 'yaml config file'},
                           {'name': 'temporal.holes_fill',          'default': '0', 'description': 'Persistency mode'},
                           {'name': 'stereo_module.emitter_on_off', 'default': 'false', 'description': ''},
                           {'name': 'stereo_module.exposure.1',     'default': '7500', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.gain.1',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.exposure.2',     'default': '1', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.gain.2',         'default': '16', 'description': 'Initial value for hdr_merge filter'},

                           {'name': 'rosbag_filename',              'default': '', 'description': ''},
                           {'name': 'required',                     'default': 'false', 'description': ''}, 
                           {'name': 'enable_fisheye1',              'default': 'false', 'description': ''},
                           {'name': 'enable_fisheye2',              'default': 'false', 'description': ''}, 
                           {'name': 'enable_pose',                  'default': 'false', 'description': ''},

                           {'name': 'base_frame_id',                'default': 'camera_link', 'description': ''}, 
                           {'name': 'depth_frame_id',               'default': 'camera_depth_frame', 'description': ''},
                           {'name': 'infra1_frame_id',              'default': 'camera_infra1_frame', 'description': ''},
                           {'name': 'infra2_frame_id',              'default': 'camera_infra2_frame', 'description': ''},
                           {'name': 'color_frame_id',               'default': 'camera_color_frame', 'description': ''}, 
                           {'name': 'fisheye_frame_id',             'default': 'camera_fisheye_frame', 'description': ''},
                           {'name': 'fisheye1_frame_id',            'default': 'camera_fisheye1_frame', 'description': ''},
                           {'name': 'fisheye2_frame_id',            'default': 'camera_fisheye2_frame', 'description': ''},
                           {'name': 'accel_frame_id',               'default': 'camera_accel_frame', 'description': ''}, 
                           {'name': 'gyro_frame_id',                'default': 'camera_gyro_frame', 'description': ''},
                           {'name': 'pose_frame_id',                'default': 'camera_pose_frame', 'description': ''},

                           {'name': 'depth_optical_frame_id',       'default': 'camera_depth_optical_frame', 'description': ''}, 
                           {'name': 'infra1_optical_frame_id',      'default': 'camera_infra1_optical_frame', 'description': ''},
                           {'name': 'infra2_optical_frame_id',      'default': 'camera_infra2_optical_frame', 'description': ''},
                           {'name': 'color_optical_frame_id',       'default': 'camera_color_optical_frame', 'description': ''},
                           {'name': 'fisheye_optical_frame_id',     'default': 'camera_fisheye_optical_frame', 'description': ''}, 
                           {'name': 'fisheye1_optical_frame_id',    'default': 'camera_fisheye1_optical_frame', 'description': ''},
                           {'name': 'fisheye2_optical_frame_id',    'default': 'camera_fisheye2_optical_frame', 'description': ''},
                           {'name': 'accel_optical_frame_id',       'default': 'camera_accel_optical_frame', 'description': ''},
                           {'name': 'gyro_optical_frame_id',        'default': 'camera_gyro_optical_frame', 'description': ''}, 
                           {'name': 'imu_optical_frame_id',         'default': 'camera_imu_optical_frame', 'description': ''},
                           {'name': 'pose_optical_frame_id',        'default': 'camera_pose_optical_frame', 'description': ''},

                           {'name': 'aligned_depth_to_color_frame_id',     'default': 'camera_aligned_depth_to_color_frame', 'description': ''},
                           {'name': 'aligned_depth_to_infra1_frame_id',    'default': 'camera_aligned_depth_to_infra1_frame', 'description': ''},
                           {'name': 'aligned_depth_to_infra2_frame_id',    'default': 'camera_aligned_depth_to_infra2_frame', 'description': ''},
                           {'name': 'aligned_depth_to_fisheye_frame_id',   'default': 'camera_aligned_depth_to_fisheye_frame', 'description': ''},
                           {'name': 'aligned_depth_to_fisheye1_frame_id',  'default': 'camera_aligned_depth_to_fisheye1_frame', 'description': ''},
                           {'name': 'aligned_depth_to_fisheye2_frame_id',  'default': 'camera_aligned_depth_to_fisheye2_frame', 'description': ''},
                           {'name': 'odom_frame_id',                       'default': 'camera_odom_frame', 'description': ''},
                           ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        # Realsense
        launch_ros.actions.Node(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('config_file'), "' == ''"])),
            package='realsense2_camera', 
            namespace=[namespace, LaunchConfiguration("namespace2")],
            name=LaunchConfiguration("camera_name"),
            executable='realsense2_camera_node',
            parameters=[set_configurable_parameters(configurable_parameters)],
            output='screen',
            emulate_tty=True,
            ),
        launch_ros.actions.Node(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('config_file'), "' != ''"])),
            package='realsense2_camera', 
            namespace=namespace,
            name=LaunchConfiguration("camera_name"),
            executable='realsense2_camera_node',
            parameters=[set_configurable_parameters(configurable_parameters), {LaunchConfiguration("config_file")}],
            output='screen',
            emulate_tty=True,
            ),
    ])
