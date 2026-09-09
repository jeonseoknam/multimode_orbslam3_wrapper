// #include <iostream>
// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "monocular-slam-node.hpp"
// #include "System.h"

// int main(int argc, char **argv)
// {
//     if (argc < 3)
//     {
//         std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
//         return 1;
//     }

//     rclcpp::init(argc, argv);

//     bool visualization = true;

//     ORB_SLAM3::System SLAM(
//         argv[1],
//         argv[2],
//         ORB_SLAM3::System::MONOCULAR,
//         visualization
//     );

//     auto node = std::make_shared<MonocularSlamNode>(&SLAM);

//     std::cout << "============================" << std::endl;
//     std::cout << "Monocular ORB-SLAM3 node started" << std::endl;
//     std::cout << "============================" << std::endl;

//     rclcpp::spin(node);

//     // 종료는 딱 한 번만
//     node->ShutdownAndSave();

//     // ROS 종료
//     rclcpp::shutdown();

//     // 마지막에 node 해제
//     node.reset();

//     return 0;
// }

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"
#include "System.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto param_node = std::make_shared<rclcpp::Node>(
        "orbslam3_mono",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    std::string vocab_path;
    std::string settings_path;
    bool visualization = true;
    bool localization_mode = false;
    bool map_based_slam = false;

    param_node->get_parameter_or("vocab_path", vocab_path, std::string(""));
    param_node->get_parameter_or("settings_path", settings_path, std::string(""));
    param_node->get_parameter_or("visualization", visualization, true);
    // Replicate the viewer's "Localization Mode" checkbox without a GUI:
    // with System.LoadAtlasFromFile set, this localizes against the loaded
    // map and stops adding keyframes. Needed for headless bag-replay.
    param_node->get_parameter_or("localization_mode", localization_mode, false);
    // Relocalize into System.LoadAtlasFromFile at startup and never leave it,
    // but keep Local Mapping running. localization_mode (only-tracking) recovers
    // from faults just as well, yet without new map points the tracker falls back
    // to pure visual odometry on weakly-matched stretches and drifts metres.
    param_node->get_parameter_or("map_based_slam", map_based_slam, false);

    if (vocab_path.empty() || settings_path.empty())
    {
        std::cerr << "\n[mono] Required parameters are missing.\n"
                  << "  - vocab_path\n"
                  << "  - settings_path\n";
        rclcpp::shutdown();
        return 1;
    }

    ORB_SLAM3::System SLAM(
        vocab_path,
        settings_path,
        ORB_SLAM3::System::MONOCULAR,
        visualization
    );

    if (localization_mode || map_based_slam) {
        if (localization_mode)
            SLAM.ActivateLocalizationMode();
        // Required on a monocular session that loads an atlas: otherwise the
        // tracker is NOT_INITIALIZED on the first frame, monocular-initialises a
        // fresh map inside the loaded one and global-BAs all 51k of its points
        // before ever emitting a pose (~50 s of silence).
        SLAM.StartLostForRelocalization();
        SLAM.SetNeverCreateNewMap(true);
    }

    auto node = std::make_shared<MonocularSlamNode>(&SLAM);

    std::cout << "============================" << std::endl;
    std::cout << "Monocular ORB-SLAM3 node started" << std::endl;
    std::cout << "vocab_path    : " << vocab_path << std::endl;
    std::cout << "settings_path : " << settings_path << std::endl;
    std::cout << "visualization : " << (visualization ? "true" : "false") << std::endl;
    std::cout << "localization  : " << (localization_mode ? "ON (map-based, no mapping)" : "off (SLAM)") << std::endl;
    std::cout << "map_based_slam: " << (map_based_slam ? "ON (reloc into atlas, mapping kept)" : "off") << std::endl;
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);

    node->ShutdownAndSave();
    rclcpp::shutdown();
    node.reset();

    return 0;
}