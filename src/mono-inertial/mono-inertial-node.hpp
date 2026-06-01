#ifndef __MONO_INERTIAL_NODE_HPP__
#define __MONO_INERTIAL_NODE_HPP__

#include <queue>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "utility.hpp"

using std::string;
using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class MonoInertialNode : public rclcpp::Node
{
public:
    MonoInertialNode(
        ORB_SLAM3::System* pSLAM,
        const string &strSettingsFile,
        const string &strDoRectify,
        const string &strDoEqual
    );

    ~MonoInertialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const ImageMsg::SharedPtr msg);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();

    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImg_;

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;

    std::queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex imuMutex_;

    std::queue<ImageMsg::SharedPtr> imgBuf_;
    std::mutex imgMutex_;

    bool doRectify_;
    bool doEqual_;

    cv::Mat K_, D_, M1_, M2_;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));

    // 경계 IMU 보존용
    ImuMsg::SharedPtr lastUsedImu_;
    bool hasLastUsedImu_ = false;

    // 디버그용
    double lastImageTime_ = -1.0;
};

#endif