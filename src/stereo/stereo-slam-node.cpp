#include "stereo-slam-node.hpp"
#include <opencv2/core/core.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM,
                               const std::string &strSettingsFile,
                               const std::string &strDoRectify)
: Node("ORB_SLAM3_ROS2"),
  m_SLAM(pSLAM)
{
    std::stringstream ss(strDoRectify);
    ss >> std::boolalpha >> doRectify;

    if (doRectify) {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;
        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;
        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;
        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() ||
            R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
            std::cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << std::endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0,3).colRange(0,3),
                                    cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0,3).colRange(0,3),
                                    cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
    }

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "/morai/camera/left/image_raw");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "/morai/camera/right/image_raw");

    // policy(10) -> (20) 변경
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
        approximate_sync_policy(20), *left_sub, *right_sub);

    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);

    m_pose_publisher = this->create_publisher<PoseStampedMsg>("orb_pose", 10);
    m_pose_aligned_publisher = this->create_publisher<PoseStampedMsg>("orb_pose_aligned", 10);

    RCLCPP_INFO(this->get_logger(), "StereoSlamNode started");
}

StereoSlamNode::~StereoSlamNode()
{
    // destructor에서는 heavy shutdown 하지 않음
}

void StereoSlamNode::ShutdownAndSave()
{
    std::lock_guard<std::mutex> lock(m_shutdown_mutex);

    if (m_has_shutdown)
        return;

    m_has_shutdown = true;
    m_is_shutting_down.store(true);

    std::cout << "[StereoSlamNode] ShutdownAndSave() called" << std::endl;

    syncApproximate.reset();
    left_sub.reset();
    right_sub.reset();

    if (m_SLAM == nullptr) {
        std::cout << "[StereoSlamNode] m_SLAM is nullptr, skip shutdown" << std::endl;
        return;
    }

    try {
        std::cout << "[StereoSlamNode] Calling m_SLAM->Shutdown()" << std::endl;
        m_SLAM->Shutdown();

        std::cout << "[StereoSlamNode] Saving keyframe trajectory to "
                  << m_keyframe_traj_path << std::endl;
        m_SLAM->SaveKeyFrameTrajectoryTUM(m_keyframe_traj_path);

        std::cout << "[StereoSlamNode] ShutdownAndSave() finished successfully" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "[StereoSlamNode] Exception during ShutdownAndSave(): "
                  << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "[StereoSlamNode] Unknown exception during ShutdownAndSave()" << std::endl;
    }
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft,
                                const ImageMsg::SharedPtr msgRight)
{

    double t_left = Utility::StampToSec(msgLeft->header.stamp);
    double t_right = Utility::StampToSec(msgRight->header.stamp);
    double dt = std::abs(t_left - t_right);



    if (dt > 0.0135) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Dropped stereo pair: dt=%.6f sec", dt);
        return;
    }

    if (m_is_shutting_down.load()) {
        return;
    }

    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    try {
        Sophus::SE3f Tcw;

        if (doRectify) {
            cv::Mat imLeft, imRight;
            cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
            cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
            Tcw = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
        } else {
            Tcw = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image,
                                      Utility::StampToSec(msgLeft->header.stamp));
        }

        if (m_SLAM->GetTrackingState() != 2) {
            return;
        }

        Sophus::SE3f Twc = Tcw.inverse();

        // -----------------------------------
        // 1) raw ORB stereo pose (left camera pose)
        // -----------------------------------
        Eigen::Vector3d p_raw = Twc.translation().cast<double>();
        Eigen::Quaterniond q_raw(Twc.unit_quaternion().cast<double>());

        PoseStampedMsg raw_msg;
        raw_msg.header.stamp = msgLeft->header.stamp;
        raw_msg.header.frame_id = "orb_map_raw";

        raw_msg.pose.position.x = p_raw.x();
        raw_msg.pose.position.y = p_raw.y();
        raw_msg.pose.position.z = p_raw.z();

        raw_msg.pose.orientation.x = q_raw.x();
        raw_msg.pose.orientation.y = q_raw.y();
        raw_msg.pose.orientation.z = q_raw.z();
        raw_msg.pose.orientation.w = q_raw.w();

        m_pose_publisher->publish(raw_msg);

        // -----------------------------------
        // 2) aligned ORB stereo pose -> base_link pose in map
        // -----------------------------------
        if (m_use_alignment)
        {
            // local stereo frame -> aligned map frame
            Eigen::Vector3d p_map_cam = m_align_scale * (m_align_R * p_raw) + m_align_t;

            Eigen::Matrix3d R_raw = q_raw.toRotationMatrix();
            Eigen::Matrix3d R_map_cam = m_align_R * R_raw;

            // optical -> base_link axis conversion
            Eigen::Matrix3d R_cam_base;
            R_cam_base << 0, -1,  0,
                          0,  0, -1,
                          1,  0,  0;

            // left camera extrinsic: base -> left_cam(body)
            Eigen::Vector3d t_base_cam_body = m_t_base_leftcam_body;

            // left camera(body) -> base
            Eigen::Vector3d t_cam_body_base_body = -t_base_cam_body;

            // convert that offset into optical frame
            Eigen::Vector3d t_cam_optical_base = R_cam_base * t_cam_body_base_body;

            // final base pose in map
            Eigen::Matrix3d R_map_base = R_map_cam * R_cam_base;
            Eigen::Vector3d p_map_base = p_map_cam + R_map_cam * t_cam_optical_base;

            Eigen::Quaterniond q_aligned(R_map_base);
            q_aligned.normalize();

            PoseStampedMsg aligned_msg;
            aligned_msg.header.stamp = msgLeft->header.stamp;
            aligned_msg.header.frame_id = "map";

            aligned_msg.pose.position.x = p_map_base.x();
            aligned_msg.pose.position.y = p_map_base.y();
            aligned_msg.pose.position.z = p_map_base.z();

            aligned_msg.pose.orientation.x = q_aligned.x();
            aligned_msg.pose.orientation.y = q_aligned.y();
            aligned_msg.pose.orientation.z = q_aligned.z();
            aligned_msg.pose.orientation.w = q_aligned.w();

            m_pose_aligned_publisher->publish(aligned_msg);
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in TrackStereo(): %s", e.what());
    }
    catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception in TrackStereo()");
    }
}