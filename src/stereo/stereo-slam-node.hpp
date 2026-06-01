#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <atomic>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class StereoSlamNode : public rclcpp::Node
{
public:
    using ImageMsg = sensor_msgs::msg::Image;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using approximate_sync_policy =
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image>;

    StereoSlamNode(ORB_SLAM3::System* pSLAM,
                   const std::string &strSettingsFile,
                   const std::string &strDoRectify);

    ~StereoSlamNode();

    void ShutdownAndSave();

private:
    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgLeft,
                    const sensor_msgs::msg::Image::SharedPtr msgRight);

    ORB_SLAM3::System* m_SLAM;

    bool doRectify;
    cv::Mat M1l, M2l, M1r, M2r;

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

    rclcpp::Publisher<PoseStampedMsg>::SharedPtr m_pose_publisher;
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr m_pose_aligned_publisher;

    std::mutex m_shutdown_mutex;
    std::atomic<bool> m_is_shutting_down{false};
    bool m_has_shutdown{false};

    std::string m_keyframe_traj_path{"KeyFrameTrajectory.txt"};

    // ORB(local stereo frame) -> LiDAR/GNSS map frame alignment
    // stereo는 metric scale이 있으므로 보통 1.0부터 시작
    bool m_use_alignment = true;
    double m_align_scale = 1.0;

    // 일단 identity로 두고, 나중에 stereo bag으로 다시 맞춘 SE3/Sim3 결과를 넣으세요.
    Eigen::Matrix3d m_align_R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d m_align_t = Eigen::Vector3d::Zero();

    // Left camera extrinsic wrt base_link
    // left camera: (1.56, +0.10, 1.10) 라고 가정
    // mono 때처럼 z를 무시하고 싶으면 z=0.0 유지
    Eigen::Vector3d m_t_base_leftcam_body = Eigen::Vector3d(1.56, 0.10, 0.0);
};

#endif