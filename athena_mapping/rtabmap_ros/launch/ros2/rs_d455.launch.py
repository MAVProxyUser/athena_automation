# Requirements:
#   A realsense D400 series
#   Install realsense2 ros2 package (refactor branch)
# Example:
#   $ ros2 run realsense_node realsense_node
#   $ ros2 launch rtabmap_ros realsense_d400.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch_ros.actions import Node

def generate_launch_description():
    rtabmap_parameters=[{
          'subscribe_depth'                    : True,
          'subscribe_rgb'                      : True,
          'subscribe_rgbd'                     : False,
          'subscribe_stereo'                   : False,
          'subscribe_scan'                     : False,
          'subscribe_scan_cloud'               : False,
          'subscribe_scan_descriptor'          : False,
          'subscribe_user_data'                : False,
          'frame_id'                           : 'base_footprint',
          'map_frame_id'                       : 'map',
          'odom_frame_id'                      : 'odom',
          'publish_tf'                         : True,
          'ground_truth_frame_id'              : '',
          'ground_truth_base_frame_id'         : '',
          'odom_tf_angular_variance'           : 0.04,
          'odom_tf_linear_variance'            : 0.04,
          'odom_sensor_sync'                   : False,
          'wait_for_transform_duration'        : 0.2,
          'database_path'                      : '~/.ros/rtabmap.db',
          'approx_sync'                        : True,
          'config_path'                        : '',
          'queue_size'                         : 2,
          'scan_cloud_max_points'              : 0,
          'landmark_linear_variance'           : 0.0001,
          'landmark_angular_variance'          : 9999.0,
          'Mem/IncrementalMemory'              : 'False',
          'Mem/InitWMWithAllNodes'             : 'False',
          'gen_scan'                           : True,
          'map_always_update'                  : True,
          'Rtabmap/DetectionRate'              : '5',
          'Rtabmap/LoopThr'                    : '0.70',
          'Rtabmap/TimeThr'                    : '800',
          'Rtabmap/StartNewMapOnGoodSignature' : 'True',
          'Mem/ReduceGraph'                    : 'True',
          'Mem/RecentWmRatio'                  : '0.2',
          'Mem/STMSize'                        : '10',
          'Mem/NotLinkedNodesKept'             : 'False',
          'Mem/BadSignaturesIgnored'           : 'True',
          'Reg/Strategy'                       : '2',
          'Optimizer/Strategy'                 : '1',
          'RGBD/OptimizeMaxError'              : '3.00',
          'RGBD/OptimizeFromGraphEnd'          : 'False',
          'RGBD/LocalRadius'                   : '1.5',
          'RGBD/LinearUpdate'                  : '0.3',
          'RGBD/NeighborLinkRefining'          : 'False',
          'RGBD/LocalBundleOnLoopClosure'      : 'True',
          'RGBD/SavedLocalizationIgnored'      : 'True',
          'Kp/BadSignRatio'                    : '0.5',
          'Vis/MaxDepth'                       : '3.5',
          'Vis/MinDepth'                       : '0.3',
          'Kp/DetectorStrategy'                : '6',
          'Vis/GridRows'                       : '3',
          'Vis/GridCols'                       : '4',
          'Vis/MinInliers'                     : '50',
          'Vis/MinInliersDistribution'         : '0.01',
          'Icp/RangeMin'                       : '0.0',
          'Icp/RangeMax'                       : '2.0',
          'Icp/MaxCorrespondenceDistance'      : '0.08',
          'Icp/VoxelSize'                      : '0.02',
          'Icp/CorrespondenceRatio'            : '0.2',
          'Grid/RangeMin'                      : '0.25',
          'Grid/RangeMax'                      : '3.0',
          'Grid/MaxObstacleHeight'             : '0.5',
          'Grid/MaxGroundHeight'               : '0.0',
          'Grid/MinClusterSize'                : '15',
          'Grid/RayTracing'                    : 'True',
          'Grid/MapFrameProjection'            : 'True',
          'GridGlobal/Eroded'                  : 'True',
          'GridGlobal/FootprintRadius'         : '0.15',
          'GridGlobal/OccupancyThr'            : '0.2',
          'GridGlobal/ProbMiss'                : '0.1'}]

    rtabmap_remappings=[
          ('rgb/image',              'camera/color/image_raw'),
          ('depth/image',            'camera/aligned_depth_to_color/image_raw'),
          ('rgb/camera_info',        'camera/color/camera_info'),
          ('rgbd_image',             'rgbd_image_relay'),
          ('left/image_rect',        'camera/infra1/image_rect_raw'),
          ('right/image_rect',       'camera/infra2/image_rect_raw'),
          ('left/camera_info',       'camera/infra1/camera_info'),
          ('right/camera_info',      'camera/infra2/camera_info'),
          ('scan',                   'scan'),
          ('scan_cloud',             'camera/depth/color/points'),
          ('scan_descriptor',        'scan_descriptor'),
          ('user_data',              'user_data'),
          ('user_data_async',        'user_data_async'),
          ('gps/fix',                'gps/fix'),
          ('tag_detections',         'tag_detections'),
          ('odom',                   'odom_chassis'),
          ('imu',                    'imu/data')]

    point_cloud_parameters=[{
          'decimation'  : 4,
          'voxel_size'  : 0.0,
          'approx_sync' : False}]

    point_cloud_remappings=[
          ('rgb/image',        'camera/color/image_raw'),
          ('depth/image',      'camera/aligned_depth_to_color/image_raw'),
          ('rgb/camera_info',  'camera/color/camera_info'),
          ('rgbd_image',       'rgbd_image_relay'),
          ('cloud',            'voxel_cloud' )]

    imu_filter_parameters=[{
          'use_mag':False,
          'publish_tf':False,
          'world_frame':'enu'}]
    imu_filter_remappings=[('imu/data_raw', 'camera/imu')]

    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([
      # Set env var to print messages to stdout immediately
      SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

      Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=namespace, name='base_joint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']),

      Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=namespace,
            name='camera_joint',
            arguments=['0.3252', '0.0475', '-0.0795', '0', '0', '0', 'base_link', 'camera_link']),

      #Node(
      #      package='imu_filter_madgwick',
      #      executable='imu_filter_madgwick_node',
      #      namespace=namespace,
      #      name='imu_filter',
      #      output={},
      #      parameters=imu_filter_parameters,
      #      remappings=imu_filter_remappings),

      Node(
            package='rtabmap_ros',
            executable='rtabmap',
            namespace=namespace,
            name='rtabmap',
            output='screen',
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings,
            # arguments=['-d']
            ),

      #Node(
      #      package='rtabmap_ros',
      #      executable='point_cloud_xyzrgb',
      #      namespace=namespace, output={},
      #      parameters=point_cloud_parameters,
      #      remappings=point_cloud_remappings,
      #      arguments=['rtabmap_ros/point_cloud_xyzrgb']),
    ])
