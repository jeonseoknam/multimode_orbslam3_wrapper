#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "mono-inertial-node.hpp"
#include "System.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 4)
    {
        std::cerr << std::endl;
        std::cerr << "Usage: ros2 run orbslam3 mono-inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    std::string vocab_file = argv[1];
    std::string settings_file = argv[2];
    std::string do_rectify = argv[3];
    std::string do_equalize = (argc >= 5) ? argv[4] : "false";

    bool visualization = true;

    // 중요:
    // ORB_SLAM3 원본 System.h 에 IMU_MONOCULAR enum 이 있어야 합니다.
    // 만약 로컬 코드에서 이름이 다르면 그 enum 이름에 맞게 바꿔주세요.
    ORB_SLAM3::System SLAM(
        vocab_file,
        settings_file,
        ORB_SLAM3::System::IMU_MONOCULAR,
        visualization
    );

    auto node = std::make_shared<MonoInertialNode>(
        &SLAM,
        settings_file,
        do_rectify,
        do_equalize
    );

    std::cout << "============================" << std::endl;
    std::cout << "ORB-SLAM3 ROS2 Mono-Inertial node started" << std::endl;
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}