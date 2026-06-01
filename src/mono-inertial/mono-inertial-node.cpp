#include "mono-inertial-node.hpp"

#include <chrono>
#include <iostream>
#include <cassert>

using std::placeholders::_1;

MonoInertialNode::MonoInertialNode(
    ORB_SLAM3::System *pSLAM,
    const string &strSettingsFile,
    const string &strDoRectify,
    const string &strDoEqual
)
    : Node("orbslam3_mono_inertial"),
      SLAM_(pSLAM)
{
    std::stringstream ss_rec(strDoRectify);
    ss_rec >> std::boolalpha >> doRectify_;

    std::stringstream ss_eq(strDoEqual);
    ss_eq >> std::boolalpha >> doEqual_;

    bClahe_ = doEqual_;

    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            std::cerr << "ERROR: Wrong path to settings file: " << strSettingsFile << std::endl;
            assert(false);
        }

        // ORB-SLAM3 monocular yaml 형식 기준
        // Camera1.fx, Camera1.fy, Camera1.cx, Camera1.cy
        // Camera1.k1, Camera1.k2, Camera1.p1, Camera1.p2, Camera1.k3
        float fx = 0.0f, fy = 0.0f, cx = 0.0f, cy = 0.0f;
        float k1 = 0.0f, k2 = 0.0f, p1 = 0.0f, p2 = 0.0f, k3 = 0.0f;
        int width = 0, height = 0;

        fsSettings["Camera1.fx"] >> fx;
        fsSettings["Camera1.fy"] >> fy;
        fsSettings["Camera1.cx"] >> cx;
        fsSettings["Camera1.cy"] >> cy;

        fsSettings["Camera1.k1"] >> k1;
        fsSettings["Camera1.k2"] >> k2;
        fsSettings["Camera1.p1"] >> p1;
        fsSettings["Camera1.p2"] >> p2;
        fsSettings["Camera1.k3"] >> k3;

        fsSettings["Camera.width"] >> width;
        fsSettings["Camera.height"] >> height;

        if (fx == 0 || fy == 0 || width == 0 || height == 0)
        {
            std::cerr << "ERROR: Monocular calibration parameters are missing in yaml." << std::endl;
            assert(false);
        }

        K_ = (cv::Mat_<float>(3, 3) <<
              fx, 0.0f, cx,
              0.0f, fy, cy,
              0.0f, 0.0f, 1.0f);

        D_ = (cv::Mat_<float>(1, 5) << k1, k2, p1, p2, k3);

        cv::initUndistortRectifyMap(
            K_,
            D_,
            cv::Mat(),
            K_,
            cv::Size(width, height),
            CV_32F,
            M1_,
            M2_
        );
    }

    // 토픽 이름은 필요하면 launch/remap으로 바꾸세요
    subImu_ = this->create_subscription<ImuMsg>(
        "/morai/imu",
        1000,
        std::bind(&MonoInertialNode::GrabImu, this, _1)
    );

    subImg_ = this->create_subscription<ImageMsg>(
        "/morai/camera/image_raw",
        100,
        std::bind(&MonoInertialNode::GrabImage, this, _1)
    );

    syncThread_ = new std::thread(&MonoInertialNode::SyncWithImu, this);
}

MonoInertialNode::~MonoInertialNode()
{
    if (syncThread_)
    {
        syncThread_->join();
        delete syncThread_;
        syncThread_ = nullptr;
    }

    SLAM_->Shutdown();
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    // 필요하면 아래도 추가 가능
    // SLAM_->SaveTrajectoryTUM("FrameTrajectory.txt");
}


void MonoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    auto finite6 =
        std::isfinite(msg->linear_acceleration.x) &&
        std::isfinite(msg->linear_acceleration.y) &&
        std::isfinite(msg->linear_acceleration.z) &&
        std::isfinite(msg->angular_velocity.x) &&
        std::isfinite(msg->angular_velocity.y) &&
        std::isfinite(msg->angular_velocity.z);

    if (!finite6)
    {
        RCLCPP_WARN(this->get_logger(), "Dropping invalid IMU sample (NaN/Inf)");
        return;
    }

    const double t = Utility::StampToSec(msg->header.stamp);

    std::lock_guard<std::mutex> lock(imuMutex_);

    if (!imuBuf_.empty())
    {
        const double t_prev = Utility::StampToSec(imuBuf_.back()->header.stamp);

        // 시간 역행만 드롭
        if (t < t_prev)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Dropping decreasing IMU timestamp: current=%.9f prev=%.9f",
                t, t_prev
            );
            return;
        }

        // 동일 timestamp는 중복으로 보고 그냥 무시
        if (t == t_prev)
        {
            return;
        }
    }

    imuBuf_.push(msg);
}

void MonoInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imgMutex_);

    while (!imgBuf_.empty())
        imgBuf_.pop();

    imgBuf_.push(msg);
}

cv::Mat MonoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }

    cv::Mat img;

    if (msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
        img = cv_ptr->image.clone();
    }
    else if (msg->encoding == sensor_msgs::image_encodings::BGR8)
    {
        cv::cvtColor(cv_ptr->image, img, cv::COLOR_BGR2GRAY);
    }
    else if (msg->encoding == sensor_msgs::image_encodings::RGB8)
    {
        cv::cvtColor(cv_ptr->image, img, cv::COLOR_RGB2GRAY);
    }
    else if (msg->encoding == sensor_msgs::image_encodings::BGRA8)
    {
        cv::cvtColor(cv_ptr->image, img, cv::COLOR_BGRA2GRAY);
    }
    else if (msg->encoding == sensor_msgs::image_encodings::RGBA8)
    {
        cv::cvtColor(cv_ptr->image, img, cv::COLOR_RGBA2GRAY);
    }
    else
    {
        try
        {
            auto mono_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
            img = mono_ptr->image.clone();
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unsupported image encoding: %s",
                msg->encoding.c_str()
            );
            return cv::Mat();
        }
    }

    return img;
}

void MonoInertialNode::SyncWithImu()
{
    while (rclcpp::ok())
    {
        ImageMsg::SharedPtr imgMsg = nullptr;
        double tImg = 0.0;

        // 1) 이미지는 일단 "peek"만 한다. 아직 pop하지 않는다.
        {
            std::lock_guard<std::mutex> lockImg(imgMutex_);
            if (imgBuf_.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            imgMsg = imgBuf_.front();
            tImg = Utility::StampToSec(imgMsg->header.stamp);
        }

        // 이미지 timestamp 역행 방지
        if (lastImageTime_ > 0.0 && tImg <= lastImageTime_)
        {
            std::lock_guard<std::mutex> lockImg(imgMutex_);
            if (!imgBuf_.empty())
                imgBuf_.pop();

            RCLCPP_WARN(
                this->get_logger(),
                "Skipping non-increasing image timestamp: current=%.9f last=%.9f",
                tImg, lastImageTime_
            );
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

        {
            std::lock_guard<std::mutex> lockImu(imuMutex_);

            if (imuBuf_.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            const double tFront = Utility::StampToSec(imuBuf_.front()->header.stamp);
            const double tBack  = Utility::StampToSec(imuBuf_.back()->header.stamp);

            // 현재 이미지보다 더 최신 IMU가 아직 없으면
            // 이미지를 버리지 말고 기다린다.
            if (tBack < tImg)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Waiting for newer IMU. tImg=%.9f, imu_back=%.9f",
                    tImg, tBack
                );
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            // 시작 시점에 현재 이미지보다 오래된 IMU가 아직 없으면 기다린다.
            if (tFront > tImg && !hasLastUsedImu_)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "No IMU sample older than image. tImg=%.9f, imu_front=%.9f",
                    tImg, tFront
                );
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            // 이전 프레임 마지막 IMU를 경계 샘플로 포함
            if (hasLastUsedImu_)
            {
                const double tLast = Utility::StampToSec(lastUsedImu_->header.stamp);
                if (tLast <= tImg)
                {
                    cv::Point3f acc(
                        lastUsedImu_->linear_acceleration.x,
                        lastUsedImu_->linear_acceleration.y,
                        lastUsedImu_->linear_acceleration.z
                    );
                    cv::Point3f gyr(
                        lastUsedImu_->angular_velocity.x,
                        lastUsedImu_->angular_velocity.y,
                        lastUsedImu_->angular_velocity.z
                    );
                    vImuMeas.emplace_back(acc, gyr, tLast);
                }
            }

            // 현재 이미지 시각 이하의 IMU를 모두 수집
            while (!imuBuf_.empty())
            {
                const double t = Utility::StampToSec(imuBuf_.front()->header.stamp);

                if (t <= tImg)
                {
                    auto imuMsg = imuBuf_.front();
                    imuBuf_.pop();

                    cv::Point3f acc(
                        imuMsg->linear_acceleration.x,
                        imuMsg->linear_acceleration.y,
                        imuMsg->linear_acceleration.z
                    );
                    cv::Point3f gyr(
                        imuMsg->angular_velocity.x,
                        imuMsg->angular_velocity.y,
                        imuMsg->angular_velocity.z
                    );

                    vImuMeas.emplace_back(acc, gyr, t);

                    lastUsedImu_ = imuMsg;
                    hasLastUsedImu_ = true;
                }
                else
                {
                    break;
                }
            }
        }

        // IMU가 비어 있으면 이미지를 버리지 말고 한 번 더 기다릴 수도 있지만,
        // 여기서는 해당 이미지를 폐기하고 다음 최신 이미지로 넘어간다.
        if (vImuMeas.empty())
        {
            std::lock_guard<std::mutex> lockImg(imgMutex_);
            if (!imgBuf_.empty())
                imgBuf_.pop();

            RCLCPP_WARN(
                this->get_logger(),
                "Skipping frame: empty IMU vector. tImg=%.9f",
                tImg
            );
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        // 이제 실제 처리 직전에 이미지 pop
        {
            std::lock_guard<std::mutex> lockImg(imgMutex_);
            if (imgBuf_.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            // front가 우리가 peek했던 이미지라고 가정
            imgMsg = imgBuf_.front();
            imgBuf_.pop();
        }

        cv::Mat im = GetImage(imgMsg);
        if (im.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (bClahe_)
            clahe_->apply(im, im);

        if (doRectify_)
            cv::remap(im, im, M1_, M2_, cv::INTER_LINEAR);

        try
        {
            auto Tcw = SLAM_->TrackMonocular(im, tImg, vImuMeas);
            (void)Tcw;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "SLAM processing exception: %s", e.what());
        }

        lastImageTime_ = tImg;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}