from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("orbslam3")
    param_file = os.path.join(pkg_share, "config", "monocular", "monocular_align.yaml")

    use_faulty = LaunchConfiguration('use_faulty')
    image_topic = PythonExpression([
        "'/camera/image_faulty' if '", use_faulty, "'.lower() == 'true' "
        "else '/morai/camera/image_raw'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_faulty',
            default_value='false',
            description='If true, ORB-SLAM subscribes /camera/image_faulty instead of /morai/camera/image_raw',
        ),
        DeclareLaunchArgument(
            'visualization',
            default_value='true',
            description='ORB-SLAM3 Pangolin viewer. Set false for headless '
                        '(offline bag-replay / automated fault experiments); '
                        'unrelated to the fault_panel GUI.',
        ),
        DeclareLaunchArgument(
            'map_based_slam',
            default_value='false',
            description='Relocalize into the System.LoadAtlasFromFile map at '
                        'startup and never create a new map on tracking loss, '
                        'while Local Mapping keeps running. Best accuracy for '
                        'map-based localization; recovers from camera faults '
                        'like localization_mode but without its VO drift.',
        ),
        DeclareLaunchArgument(
            'localization_mode',
            default_value='false',
            description='If true, call System::ActivateLocalizationMode() at '
                        'startup = the viewer "Localization Mode" checkbox '
                        '(localize against System.LoadAtlasFromFile map, no new '
                        'mapping). Use true for headless runs; false = manual '
                        'control via the viewer button.',
        ),
        Node(
            package="orbslam3",
            executable="mono",
            name="orbslam3_mono",
            output="screen",
            # dict AFTER param_file overrides the yaml values.
            parameters=[param_file, {
                'visualization': ParameterValue(
                    LaunchConfiguration('visualization'), value_type=bool),
                'localization_mode': ParameterValue(
                    LaunchConfiguration('localization_mode'), value_type=bool),
                'map_based_slam': ParameterValue(
                    LaunchConfiguration('map_based_slam'), value_type=bool),
            }],
            remappings=[('/morai/camera/image_raw', image_topic)],
        )
    ])