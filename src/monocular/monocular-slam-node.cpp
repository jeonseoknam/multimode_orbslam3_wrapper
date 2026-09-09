#include "monocular-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
: Node("ORB_SLAM3_ROS2",
       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  m_SLAM(pSLAM)
{
    // =========================================================
    // optical -> camera_link rotation
    //
    // ROS camera optical frame:
    //   x = right, y = down, z = forward
    //
    // ROS camera_link/body frame:
    //   x = forward, y = left, z = up
    // =========================================================
    m_R_optical_camlink <<
         0.0,  0.0, 1.0,
        -1.0,  0.0, 0.0,
         0.0, -1.0, 0.0;

    // =========================================================
    // MORAI screenshot:
    // base_link -> camera_link
    // position = (1.56, 0.0, 1.10)
    // rotation = (0, 0, 0)
    // =========================================================
    m_t_base_camlink_in_base = Eigen::Vector3d(1.56, 0.0, 1.10);
    m_R_camlink_base = Eigen::Matrix3d::Identity();

    LoadAlignmentParameters();

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/morai/camera/image_raw",
        // "/camera/image_faulty",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, _1));

    // raw
    m_pose_camera_raw_publisher =
        this->create_publisher<PoseStampedMsg>("orb_pose_camera_raw", 10);   // optical raw

    m_pose_camera_link_publisher =
        this->create_publisher<PoseStampedMsg>("orb_pose", 10);              // camera_link raw

    m_pose_base_raw_publisher =
        this->create_publisher<PoseStampedMsg>("orb_pose_base_raw", 10);     // base_link raw

    // aligned
    // 주의:
    // 아래 camera_aligned는 이제 "aligned optical"이 아니라
    // "aligned camera_link" 의미로 사용합니다.
    m_pose_camera_aligned_publisher =
        this->create_publisher<PoseStampedMsg>("orb_pose_camera_aligned", 10);

    // 최종 aligned base_link pose
    m_pose_aligned_publisher =
        this->create_publisher<PoseStampedMsg>("orb_pose_aligned", 10);

    m_tracking_state_publisher =
        this->create_publisher<Int32Msg>("orb_tracking_state", 10);
    m_tracking_ok_publisher =
        this->create_publisher<BoolMsg>("orb_tracking_ok", 10);

    RCLCPP_INFO(this->get_logger(), "MonocularSlamNode started");
    RCLCPP_INFO(this->get_logger(), "use_alignment: %s", m_use_alignment ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "align_scale  : %.12f", m_align_scale);
    RCLCPP_INFO(this->get_logger(), "base->camera_link translation: [%.3f, %.3f, %.3f]",
                m_t_base_camlink_in_base.x(),
                m_t_base_camlink_in_base.y(),
                m_t_base_camlink_in_base.z());
}

MonocularSlamNode::~MonocularSlamNode()
{
}

void MonocularSlamNode::LoadAlignmentParameters()
{
    this->get_parameter_or("use_alignment", m_use_alignment, false);
    this->get_parameter_or("align_scale", m_align_scale, 1.0);

    std::vector<double> align_R_vec;
    std::vector<double> align_t_vec;

    this->get_parameter_or(
        "align_R",
        align_R_vec,
        std::vector<double>{1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0});

    this->get_parameter_or(
        "align_t",
        align_t_vec,
        std::vector<double>{0.0, 0.0, 0.0});

    if (align_R_vec.size() != 9) {
        throw std::runtime_error("Parameter 'align_R' must have exactly 9 elements.");
    }
    if (align_t_vec.size() != 3) {
        throw std::runtime_error("Parameter 'align_t' must have exactly 3 elements.");
    }

    m_align_R <<
        align_R_vec[0], align_R_vec[1], align_R_vec[2],
        align_R_vec[3], align_R_vec[4], align_R_vec[5],
        align_R_vec[6], align_R_vec[7], align_R_vec[8];

    m_align_t <<
        align_t_vec[0], align_t_vec[1], align_t_vec[2];
}

void MonocularSlamNode::ShutdownAndSave()
{
    std::lock_guard<std::mutex> lock(m_shutdown_mutex);

    if (m_has_shutdown)
        return;

    m_has_shutdown = true;
    m_is_shutting_down.store(true);

    std::cout << "[MonocularSlamNode] ShutdownAndSave() called" << std::endl;

    m_image_subscriber.reset();

    if (m_SLAM == nullptr)
    {
        std::cout << "[MonocularSlamNode] m_SLAM is nullptr, skip shutdown" << std::endl;
        return;
    }

    try
    {
        std::cout << "[MonocularSlamNode] Calling m_SLAM->Shutdown()" << std::endl;
        m_SLAM->Shutdown();

        std::cout << "[MonocularSlamNode] ShutdownAndSave() finished successfully" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[MonocularSlamNode] Exception during ShutdownAndSave(): "
                  << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "[MonocularSlamNode] Unknown exception during ShutdownAndSave()" << std::endl;
    }
}

void MonocularSlamNode::OpticalToCameraLink(
    const Eigen::Vector3d& p_opt_raw,
    const Eigen::Quaterniond& q_opt_raw,
    Eigen::Vector3d& p_camlink_raw,
    Eigen::Quaterniond& q_camlink_raw) const
{
    Eigen::Matrix3d R_map_opt = q_opt_raw.toRotationMatrix();

    // m_R_optical_camlink is interpreted as:
    //   camera_link <- optical
    //
    // To compose on the right of R_map_opt (= map <- optical),
    // we need:
    //   optical <- camera_link
    // = (camera_link <- optical)^T
    Eigen::Matrix3d R_map_camlink = R_map_opt * m_R_optical_camlink.transpose();

    // optical frame and camera_link share the same sensor origin
    p_camlink_raw = p_opt_raw;

    q_camlink_raw = Eigen::Quaterniond(R_map_camlink);
    q_camlink_raw.normalize();
}

void MonocularSlamNode::CameraLinkToBaseLink(
    const Eigen::Vector3d& p_camlink_raw,
    const Eigen::Quaterniond& q_camlink_raw,
    Eigen::Vector3d& p_base_raw,
    Eigen::Quaterniond& q_base_raw) const
{
    Eigen::Matrix3d R_map_camlink = q_camlink_raw.toRotationMatrix();

    // base->camera_link is known in base frame
    // convert to camera_link->base expressed in camera_link frame
    Eigen::Vector3d t_camlink_base_in_camlink =
        -(m_R_camlink_base.transpose() * m_t_base_camlink_in_base);

    Eigen::Matrix3d R_map_base = R_map_camlink * m_R_camlink_base;

    p_base_raw = p_camlink_raw + R_map_camlink * t_camlink_base_in_camlink;

    q_base_raw = Eigen::Quaterniond(R_map_base);
    q_base_raw.normalize();
}

void MonocularSlamNode::CameraLinkAlignedToBaseAligned(
    const Eigen::Vector3d& p_camlink_aligned,
    const Eigen::Quaterniond& q_camlink_aligned,
    Eigen::Vector3d& p_base_aligned,
    Eigen::Quaterniond& q_base_aligned) const
{
    Eigen::Matrix3d R_map_camlink = q_camlink_aligned.toRotationMatrix();

    Eigen::Vector3d t_camlink_base_in_camlink =
        -(m_R_camlink_base.transpose() * m_t_base_camlink_in_base);

    Eigen::Matrix3d R_map_base = R_map_camlink * m_R_camlink_base;

    p_base_aligned = p_camlink_aligned + R_map_camlink * t_camlink_base_in_camlink;

    q_base_aligned = Eigen::Quaterniond(R_map_base);
    q_base_aligned.normalize();
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    if (m_is_shutting_down.load())
        return;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat gray;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    try
    {
        Sophus::SE3f Tcw =
            m_SLAM->TrackMonocular(gray, Utility::StampToSec(msg->header.stamp));

        const int tracking_state = m_SLAM->GetTrackingState();

        // Publish on every frame, including the frames that produce no pose,
        // so a tracking loss is visible immediately instead of only through
        // the absence of poses.
        Int32Msg state_msg;
        state_msg.data = tracking_state;
        m_tracking_state_publisher->publish(state_msg);

        BoolMsg ok_msg;
        ok_msg.data = (tracking_state == 2);
        m_tracking_ok_publisher->publish(ok_msg);

        if (tracking_state != 2)
            return;

        Sophus::SE3f Twc = Tcw.inverse();

        // ======================================================
        // 1) optical raw pose -> /orb_pose_camera_raw
        // ======================================================
        Eigen::Vector3d p_opt_raw = Twc.translation().cast<double>();
        Eigen::Quaterniond q_opt_raw(Twc.unit_quaternion().cast<double>());
        q_opt_raw.normalize();

        PoseStampedMsg optical_raw_msg;
        optical_raw_msg.header.stamp = msg->header.stamp;
        optical_raw_msg.header.frame_id = "map";

        optical_raw_msg.pose.position.x = p_opt_raw.x();
        optical_raw_msg.pose.position.y = p_opt_raw.y();
        optical_raw_msg.pose.position.z = p_opt_raw.z();

        optical_raw_msg.pose.orientation.x = q_opt_raw.x();
        optical_raw_msg.pose.orientation.y = q_opt_raw.y();
        optical_raw_msg.pose.orientation.z = q_opt_raw.z();
        optical_raw_msg.pose.orientation.w = q_opt_raw.w();

        m_pose_camera_raw_publisher->publish(optical_raw_msg);

        // ======================================================
        // 2) camera_link raw pose -> /orb_pose
        // ======================================================
        Eigen::Vector3d p_camlink_raw;
        Eigen::Quaterniond q_camlink_raw;
        OpticalToCameraLink(p_opt_raw, q_opt_raw, p_camlink_raw, q_camlink_raw);

        PoseStampedMsg camlink_raw_msg;
        camlink_raw_msg.header.stamp = msg->header.stamp;
        camlink_raw_msg.header.frame_id = "map";

        camlink_raw_msg.pose.position.x = p_camlink_raw.x();
        camlink_raw_msg.pose.position.y = p_camlink_raw.y();
        camlink_raw_msg.pose.position.z = p_camlink_raw.z();

        camlink_raw_msg.pose.orientation.x = q_camlink_raw.x();
        camlink_raw_msg.pose.orientation.y = q_camlink_raw.y();
        camlink_raw_msg.pose.orientation.z = q_camlink_raw.z();
        camlink_raw_msg.pose.orientation.w = q_camlink_raw.w();

        m_pose_camera_link_publisher->publish(camlink_raw_msg);

        // ======================================================
        // 3) base_link raw pose -> /orb_pose_base_raw
        // ======================================================
        Eigen::Vector3d p_base_raw;
        Eigen::Quaterniond q_base_raw;
        CameraLinkToBaseLink(p_camlink_raw, q_camlink_raw, p_base_raw, q_base_raw);

        PoseStampedMsg base_raw_msg;
        base_raw_msg.header.stamp = msg->header.stamp;
        base_raw_msg.header.frame_id = "map";

        base_raw_msg.pose.position.x = p_base_raw.x();
        base_raw_msg.pose.position.y = p_base_raw.y();
        base_raw_msg.pose.position.z = p_base_raw.z();

        base_raw_msg.pose.orientation.x = q_base_raw.x();
        base_raw_msg.pose.orientation.y = q_base_raw.y();
        base_raw_msg.pose.orientation.z = q_base_raw.z();
        base_raw_msg.pose.orientation.w = q_base_raw.w();

        m_pose_base_raw_publisher->publish(base_raw_msg);

        // ======================================================
        // 4) aligned branch
        //
        // IMPORTANT:
        // align_orb_to_ndt_sim3.py must be run with:
        //   --orb_topic /orb_pose
        //   --ref_topic /current_pose
        //
        // Therefore, scale/R/t are defined on camera_link raw pose,
        // NOT on optical raw pose and NOT on base_raw pose.
        //
        // So runtime alignment must be applied to p_camlink_raw/q_camlink_raw.
        // ======================================================
        if (m_use_alignment)
        {
            // 4-1) camera_link raw -> camera_link aligned
            Eigen::Vector3d p_camlink_aligned =
                m_align_scale * (m_align_R * p_camlink_raw) + m_align_t;

            Eigen::Matrix3d R_camlink_raw = q_camlink_raw.toRotationMatrix();
            Eigen::Matrix3d R_camlink_aligned = m_align_R * R_camlink_raw;

            Eigen::Quaterniond q_camlink_aligned(R_camlink_aligned);
            q_camlink_aligned.normalize();

            // Publish aligned camera_link pose
            // topic name is kept as orb_pose_camera_aligned for compatibility
            // but semantic meaning is now "aligned camera_link pose"
            PoseStampedMsg camlink_aligned_msg;
            camlink_aligned_msg.header.stamp = msg->header.stamp;
            camlink_aligned_msg.header.frame_id = "map";

            camlink_aligned_msg.pose.position.x = p_camlink_aligned.x();
            camlink_aligned_msg.pose.position.y = p_camlink_aligned.y();
            camlink_aligned_msg.pose.position.z = p_camlink_aligned.z();

            camlink_aligned_msg.pose.orientation.x = q_camlink_aligned.x();
            camlink_aligned_msg.pose.orientation.y = q_camlink_aligned.y();
            camlink_aligned_msg.pose.orientation.z = q_camlink_aligned.z();
            camlink_aligned_msg.pose.orientation.w = q_camlink_aligned.w();

            m_pose_camera_aligned_publisher->publish(camlink_aligned_msg);

            // 4-2) aligned camera_link -> aligned base_link
            Eigen::Vector3d p_base_aligned;
            Eigen::Quaterniond q_base_aligned;
            CameraLinkAlignedToBaseAligned(
                p_camlink_aligned, q_camlink_aligned,
                p_base_aligned, q_base_aligned);

            // 4-3) final aligned base pose
            PoseStampedMsg base_aligned_msg;
            base_aligned_msg.header.stamp = msg->header.stamp;
            base_aligned_msg.header.frame_id = "map";

            base_aligned_msg.pose.position.x = p_base_aligned.x();
            base_aligned_msg.pose.position.y = p_base_aligned.y();
            base_aligned_msg.pose.position.z = p_base_aligned.z();

            base_aligned_msg.pose.orientation.x = q_base_aligned.x();
            base_aligned_msg.pose.orientation.y = q_base_aligned.y();
            base_aligned_msg.pose.orientation.z = q_base_aligned.z();
            base_aligned_msg.pose.orientation.w = q_base_aligned.w();

            // This is the final /orb_pose_aligned_base role
            m_pose_aligned_publisher->publish(base_aligned_msg);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception in TrackMonocular(): %s", e.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception in TrackMonocular()");
    }
}