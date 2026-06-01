#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include <mutex>
#include <atomic>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "utility.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    using ImageMsg       = sensor_msgs::msg::Image;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

    explicit MonocularSlamNode(ORB_SLAM3::System* pSLAM);
    ~MonocularSlamNode() override;

    void ShutdownAndSave();

private:
    void GrabImage(const ImageMsg::SharedPtr msg);

    // optical(raw) -> camera_link
    void OpticalToCameraLink(
        const Eigen::Vector3d& p_opt_raw,
        const Eigen::Quaterniond& q_opt_raw,
        Eigen::Vector3d& p_camlink_raw,
        Eigen::Quaterniond& q_camlink_raw) const;

    // camera_link -> base_link
    void CameraLinkToBaseLink(
        const Eigen::Vector3d& p_camlink_raw,
        const Eigen::Quaterniond& q_camlink_raw,
        Eigen::Vector3d& p_base_raw,
        Eigen::Quaterniond& q_base_raw) const;

    // aligned camera_link -> aligned base_link
    void CameraLinkAlignedToBaseAligned(
        const Eigen::Vector3d& p_camlink_aligned,
        const Eigen::Quaterniond& q_camlink_aligned,
        Eigen::Vector3d& p_base_aligned,
        Eigen::Quaterniond& q_base_aligned) const;

    void LoadAlignmentParameters();

private:
    ORB_SLAM3::System* m_SLAM = nullptr;

    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;

    // raw poses
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr m_pose_camera_raw_publisher;   // /orb_pose_camera_raw (optical)
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr m_pose_camera_link_publisher;  // /orb_pose (camera_link)
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr m_pose_base_raw_publisher;     // /orb_pose_base_raw (base_link)

    // aligned poses
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr m_pose_camera_aligned_publisher; // /orb_pose_camera_aligned
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr m_pose_aligned_publisher;        // /orb_pose_aligned (base aligned)

    std::atomic<bool> m_is_shutting_down{false};
    bool m_has_shutdown = false;
    std::mutex m_shutdown_mutex;

    // ===== Extrinsic =====
    // optical -> camera_link rotation
    Eigen::Matrix3d m_R_optical_camlink = Eigen::Matrix3d::Identity();

    // base_link -> camera_link translation, expressed in base_link frame
    // MORAI screenshot: x=1.56, y=0.0, z=1.10
    Eigen::Vector3d m_t_base_camlink_in_base = Eigen::Vector3d::Zero();

    // camera_link -> base_link rotation
    // MORAI screenshot rotation is all zero, so Identity
    Eigen::Matrix3d m_R_camlink_base = Eigen::Matrix3d::Identity();

    // ===== Alignment params =====
    bool m_use_alignment = false;
    double m_align_scale = 1.0;
    Eigen::Matrix3d m_align_R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d m_align_t = Eigen::Vector3d::Zero();
};

#endif