from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

configurable_parameters = [
        {'name': 'publish_tf',              'default': 'false'},
        {'name': 'use_fej',                 'default': 'true'},
        {'name': 'use_imuavg',              'default': 'true'},
        {'name': 'use_rk4int',              'default': 'true'},
        {'name': 'use_stereo',              'default': 'false'},
        {'name': 'calib_cam_extrinsics',    'default': 'false'},
        {'name': 'calib_cam_intrinsics',    'default': 'false'},
        {'name': 'calib_cam_timeoffset',    'default': 'true'},
        {'name': 'calib_camimu_dt',         'default': '0.0'},
        {'name': 'max_clones',              'default': '11'},
        {'name': 'max_slam',                'default': '50'},
        {'name': 'max_slam_in_update',      'default': '25'},
        {'name': 'max_msckf_in_update',     'default': '40'},
        {'name': 'max_cameras',             'default': '1'},
        {'name': 'dt_slam_delay',           'default': '0.0'},
        {'name': 'init_window_time',        'default': '1.0'},
        {'name': 'init_imu_thresh',         'default': '0.4'},
        {'name': 'gravity',                 'default': '[0.0,0.0,9.81]'},
        {'name': 'feat_rep_msckf',          'default': 'GLOBAL_3D'},
        {'name': 'feat_rep_slam',           'default': 'ANCHORED_FULL_INVERSE_DEPTH'},
        {'name': 'feat_rep_aruco',          'default': 'ANCHORED_FULL_INVERSE_DEPTH'},
        {'name': 'try_zupt',                'default': 'false'},
        {'name': 'zupt_chi2_multipler',     'default': '2'},
        {'name': 'zupt_max_velocity',       'default': '0.5'},
        {'name': 'zupt_noise_multiplier',   'default': '50.0'},
        {'name': 'record_timing_information',   'default': 'false'},
        {'name': 'record_timing_filepath',      'default': ''},
        {'name': 'use_klt',                 'default': 'true' },
        {'name': 'num_pts',                 'default': '300' },
        {'name': 'fast_threshold',          'default': '15' },
        {'name': 'grid_x',                  'default': '4' },
        {'name': 'grid_y',                  'default': '3' },
        {'name': 'min_px_dist',             'default': '5' },
        {'name': 'knn_ratio',               'default': '0.65' },
        {'name': 'downsample_cameras',      'default': 'false' },
        {'name': 'multi_threading',         'default': 'true' },
        {'name': 'use_aruco',               'default': 'false' },
        {'name': 'num_aruco',               'default': '1024' },
        {'name': 'downsize_aruco',          'default': 'true' },
        {'name': 'up_msckf_sigma_px',           'default': '1.0' },
        {'name': 'up_msckf_chi2_multipler',     'default': '1' },
        {'name': 'up_slam_sigma_px',            'default': '1.0' },
        {'name': 'up_slam_chi2_multipler',      'default': '1' },
        {'name': 'up_aruco_sigma_px',           'default': '1.0' },
        {'name': 'up_aruco_chi2_multipler',     'default': '1' },
        {'name': 'gyroscope_noise_density',     'default': '0.016' },
        {'name': 'gyroscope_random_walk',       'default': '0.0022' },
        {'name': 'accelerometer_noise_density', 'default': '0.0028' },
        {'name': 'accelerometer_random_walk',   'default': '0.00086' },
        {'name': 'cam0_wh', 'default': '[640, 480]'},
        {'name': 'cam0_is_fisheye', 'default': 'false' },
        {'name': 'cam0_k',  'default': '[423.738861083984,424.0939331054687,319.178070068359,244.712249755859]'},
        {'name': 'cam0_d',  'default': '[-0.05016623064875603, 0.057663965970277786, -0.0005494342185556889, 0.00038652922376058996]'},
        {'name': 'T_C0toI', 'default': '[1.0, 0.0, 0.0, -0.029, \
                                         0.0, 1.0, 0.0, 0.008, \
                                         0.0, 0.0, 1.0, 0.016, \
                                         0.0, 0.0, 0.0, 1.0]'}]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        launch_ros.actions.Node(
            package='ov_msckf',
            namespace=namespace,
            executable='ros_subscribe_msckf',
            name='ov_msckf',
            remappings=[('imu', 'camera/imu'), ('topic_camera0', 'camera/color/image_raw')],
            parameters = [set_configurable_parameters(configurable_parameters) ],
            output='screen',
            ),
        ])
