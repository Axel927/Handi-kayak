import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Nom du package
    package_name = 'imu_package'

    pkg_path = os.path.join(get_package_share_directory('imu_package'))
    robot_localization_file_path = os.path.join(pkg_path, 'config/ekf.yaml')

    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[robot_localization_file_path]
        )

    #Launch Navsat
    navsat_transform_node = Node(
    package='robot_localization',
    executable='navsat_transform_node',
    name='navsat_transform_node',
    remappings=[
        ('gps/fix','Gps_readings'),
        #('odometry/filtered','telemetry/navsat_transform_odometry_output'),
        ("gps/filtered","telemetry/gnss/filtered"),
        ('imu','Imu_readings'),
    ],
    parameters=[{
        "publish_filtered_gps": True,
        "yaw_offset": 1.5707963,
        "zero_altitude": True,
        "use_odometry_yaw": False,
        "magnetic_declination_radians": 0.0383972435, # https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
        "datum": [52.00000, 4.00000,0.0]
    }],
        )

    # Lancer l'imu
    imu = Node(
        package='imu_package', executable='imu_main',
        )
    # Lancer l'imu
    position_zero = Node(
        package='imu_package', executable='position_zero',
        )

    ekf_listener = Node(
        package = 'imu_package', executable = 'ekf_listener'
    )

    gps = Node(
        package = 'imu_package', executable = 'gps_listener'
    )

    return LaunchDescription([
        start_robot_localization_cmd,
        imu,
        ekf_listener,
        gps,
        #navsat_transform_node
    ])
