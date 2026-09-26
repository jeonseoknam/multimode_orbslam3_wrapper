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

    # Resolve the vocabulary and the ORB-SLAM3 settings from this package rather
    # than from whatever absolute path monocular_align.yaml happens to carry.
    # Those entries used to point at a developer's other workspace, so a fresh
    # clone silently ran someone else's settings file -- which had
    # System.LoadAtlasFromFile enabled, so "mapping" started with a map already
    # loaded and saved nothing on exit.
    default_vocab = os.path.join(pkg_share, "vocabulary", "ORBvoc.txt")
    default_settings = os.path.join(
        pkg_share, "config", "monocular", "MORAI_1280x720.yaml")

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
            'vocab_path',
            default_value=default_vocab,
            description='ORB vocabulary. Defaults to the copy in this package.',
        ),
        DeclareLaunchArgument(
            'settings_path',
            default_value=default_settings,
            description='ORB-SLAM3 settings YAML. Defaults to this package\'s '
                        'MORAI_1280x720.yaml. This file -- not any launch '
                        'argument -- decides whether an atlas is loaded '
                        '(System.LoadAtlasFromFile) or saved '
                        '(System.SaveAtlasToFile).',
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
                'vocab_path': LaunchConfiguration('vocab_path'),
                'settings_path': LaunchConfiguration('settings_path'),
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