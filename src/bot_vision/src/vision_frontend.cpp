#include "bot_vision/vision_frontend.hpp"
#include "bot_vision/distortion_correction.hpp"
#include "bot_vision/triangulation.hpp"
#include "bot_vision/preprocessing.hpp"
#include "bot_vision/multiview_policy.hpp"
#include "bot_vision/feature_tracks.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <chrono>

namespace bot_vision
{
    VisionFrontend::VisionFrontend(const std::string& name)
    : Node(name),
    m_tf_buffer(this->get_clock()),
    m_tf_listener(m_tf_buffer),
    m_rect_maps_ready(false)
    {
        declare_parameter<std::vector<std::string>>("image_topics", {"/left_camera/image_raw", "/right_camera/image_raw"});
        m_img_topics = get_parameter("image_topics").as_string_array();

        m_img_subs.resize(m_img_topics.size());
        m_img_timestamps.m_timestamp_array.assign(
            m_img_topics.size(),
            rclcpp::Time(0, 0, RCL_ROS_TIME)
        );
        m_has_timestamp.assign(m_img_topics.size(), false);
        m_selected_ids.assign(m_img_topics.size(), false);
        cv::Mat empty {};
        m_raw_images.assign(m_img_topics.size(), empty);

        declare_parameter<std::vector<std::string>>("cam_info_topics", {"/left_camera/camera_info", "/right_camera/camera_info"});
        m_cam_info_topics = get_parameter("cam_info_topics").as_string_array();

        m_cam_info_subs.resize(m_cam_info_topics.size());
        m_info_cameras.resize(m_cam_info_subs.size());
        m_camera_ready.assign(m_cam_info_subs.size(), false);
        m_cam_tf_names.resize(m_cam_info_topics.size());

        declare_parameter<std::vector<std::string>>("cam_tf_names", {"left_camera_link_optical", "right_camera_link_optical"});
        m_cam_tf_names = get_parameter("cam_tf_names").as_string_array();

        std::size_t cam_idx {0};
        for(const std::string& cam_info_topic : m_cam_info_topics)
        {
            m_cam_info_subs[cam_idx] = create_subscription<sensor_msgs::msg::CameraInfo>(
                cam_info_topic,
                10,
                [this, cam_idx](const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
                {
                    this->cameraInfoCallback(cam_idx, info_msg);
                }                
            );
            ++cam_idx;
        }

        std::size_t img_sub_idx {0};
        for(const std::string& img_topic : m_img_topics)
        {
            m_img_subs[img_sub_idx] = create_subscription<sensor_msgs::msg::Image>(
                img_topic,
                10,
                [this, img_sub_idx](const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
                {
                    this->imageArrivalCallback(img_sub_idx, img_msg);
                }               
            );
            ++img_sub_idx;
        }

        rclcpp::QoS points_cov_pub_qos(10);
        m_pts_cov_pub = create_publisher<bot_interfaces::msg::PointWithCovarianceArray>(
            "/ua_triangulation/pointswithcovariance",
            points_cov_pub_qos
        );

        rclcpp::QoS points_pub_qos(10);
        m_pts_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/ua_triangulation/points",
            points_pub_qos
        );

        RCLCPP_INFO(
            get_logger(),
            "VisionFrontend initialized img_topics=%zu cam_info_topics=%zu cam_tf_names=%zu",
            m_img_topics.size(),
            m_cam_info_topics.size(),
            m_cam_tf_names.size()
        );

        m_matcher_client = create_client<bot_interfaces::srv::MatchedCorrespondences2d>(
            "/xfeat_lightglue"
        );

    }

    void VisionFrontend::cameraInfoCallback(
        std::size_t cam_idx,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg
    )
    {
        RCLCPP_INFO_ONCE(
            get_logger(),
            "cameraInfoCallback reached for frame=%s",
            info_msg->header.frame_id.c_str()
        );

        m_info_cameras[cam_idx].populateCameraCalibration(info_msg);
        m_info_cameras[cam_idx].setCameraId(cam_idx);

        try
        {
            const auto tf_msg = m_tf_buffer.lookupTransform(
                "base_footprint",
                m_cam_tf_names.at(cam_idx),
                tf2::TimePointZero
            );

            m_info_cameras[cam_idx].setCameraExtrinsics(tf_msg);
            m_camera_ready[cam_idx] = true;

            const auto& cam_info = m_info_cameras[cam_idx];
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "cam[%zu] calibration ready frame=%s",
                cam_idx,
                cam_info.m_frame_id.c_str()
            );
        }
        catch(const tf2::TransformException& ex)
        {
            m_camera_ready[cam_idx] = false;
            RCLCPP_WARN_STREAM(get_logger(), "tf lookup failed for tf frame: "
                                                << m_cam_tf_names.at(cam_idx).c_str()
                                                << " with exception: " << ex.what());
        }
    }

    void VisionFrontend::imageArrivalCallback(
            std::size_t img_sub_idx,
            const sensor_msgs::msg::Image::ConstSharedPtr img_msg
        )
        {
            RCLCPP_INFO_ONCE(
                get_logger(),
                "imageArrivalCallback reached for sub_idx=%zu",
                img_sub_idx
            );

            m_img_timestamps[img_sub_idx] = rclcpp::Time(
                img_msg->header.stamp,
                RCL_ROS_TIME
            );
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg, img_msg->encoding);
            m_raw_images[img_sub_idx] = cv_ptr->image.clone();
            m_has_timestamp[img_sub_idx] = true;

            for(const auto& timestamp : m_has_timestamp)
            {
                if(!timestamp) {return;}
            }

            
            RCLCPP_INFO_ONCE(get_logger(), "All image timestamps received; entering multiview selection");
            m_id_selector.getTimestamps(m_img_timestamps);
            m_selected_ids = m_id_selector.m_result;

            processSelectedViews();
        }

    void VisionFrontend::processSelectedViews()
    {
        RCLCPP_INFO_ONCE(get_logger(), "processSelectedViews reached");

        std::vector<std::size_t> selected_cam_ids {};

        for(std::size_t idx {0}; idx < m_selected_ids.size(); ++idx)
        {
            if(m_selected_ids[idx])
            {
                selected_cam_ids.push_back(idx);
            }
        }

        std::ostringstream selected_ids_stream {};
        double min_selected_time {0.0};
        double max_selected_time {0.0};
        bool has_selected_time {false};

        for(const std::size_t global_id : selected_cam_ids)
        {
            selected_ids_stream << global_id << ' ';
            const double stamp_sec = m_img_timestamps[global_id].seconds();

            if(!has_selected_time)
            {
                min_selected_time = stamp_sec;
                max_selected_time = stamp_sec;
                has_selected_time = true;
            }
            else
            {
                min_selected_time = std::min(min_selected_time, stamp_sec);
                max_selected_time = std::max(max_selected_time, stamp_sec);
            }
        }

        const double max_timestamp_skew = has_selected_time
            ? (max_selected_time - min_selected_time)
            : 0.0;
        const std::string selected_ids_str = selected_ids_stream.str();

        if(selected_cam_ids.size() < 2) 
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Skipping frame: selected_views=%zu ids=[%s] max_timestamp_skew=%.6f s",
                selected_cam_ids.size(),
                selected_ids_str.c_str(),
                max_timestamp_skew
            );
            return;
        }

        for(const std::size_t cam_id : selected_cam_ids)
        {
            if(cam_id >= m_camera_ready.size() || !m_camera_ready[cam_id])
            {
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Skipping frame: selected camera %zu is not calibration/TF ready yet",
                    cam_id
                );
                return;
            }
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Selected views=%zu ids=[%s] max_timestamp_skew=%.6f s",
            selected_cam_ids.size(),
            selected_ids_str.c_str(),
            max_timestamp_skew
        );

        if(m_matcher_request_in_flight)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Skipping frame: matcher request already in flight"
            );
            return;
        }

        auto request = std::make_shared<bot_interfaces::srv::MatchedCorrespondences2d::Request>();
        request->selected_cam_ids.clear();
        request->selected_stamps.clear();

        for(const std::size_t global_id : selected_cam_ids)
        {
            request->selected_cam_ids.push_back(static_cast<std::uint32_t>(global_id));
            request->selected_stamps.push_back(
                static_cast<builtin_interfaces::msg::Time>(m_img_timestamps[global_id])
            );
        }

        request->choose_ref = true;
        request->ref_cam_id = 0;

        constexpr double sigma_x {0.1};
        constexpr double sigma_y {0.1};
        m_pending_selected_cam_ids = selected_cam_ids;
        m_pending_sigma_x = sigma_x;
        m_pending_sigma_y = sigma_y;

        if(!m_matcher_client->wait_for_service(std::chrono::milliseconds(500)))
        {
            RCLCPP_WARN(
                get_logger(),
                "Matcher service /xfeat_lightglue is not available"
            );
            return;
        }

        try
        {
            m_matcher_client->async_send_request(
                request,
                std::bind(
                    &VisionFrontend::matcherResponseCallback,
                    this,
                    std::placeholders::_1
                )
            );
            m_matcher_request_in_flight = true;
        }
        catch(const std::exception& ex)
        {
            m_matcher_request_in_flight = false;
            RCLCPP_ERROR(
                get_logger(),
                "Matcher async_send_request threw exception: %s",
                ex.what()
            );
            return;
        }

        return;
    }

    void VisionFrontend::matcherResponseCallback(
        rclcpp::Client<bot_interfaces::srv::MatchedCorrespondences2d>::SharedFuture future
    )
    {
        m_matcher_request_in_flight = false;
        const auto response = future.get();

        if(!response->success)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Matcher service returned failure: %s",
                response->message.c_str()
            );
            return;
        }

        feature_tracks::CurrentFrameObservation observations {};
        observations.build(
            rclcpp::Time(response->ref_stamp).seconds(),
            response->ref_cam_id,
            m_pending_selected_cam_ids,
            response->matched_pairs,
            m_pending_sigma_x,
            m_pending_sigma_y
        );

        const bool has_empty_camera_observation = std::any_of(
            observations.m_camera_observations.begin(),
            observations.m_camera_observations.end(),
            [](const auto& camera_observation)
            {
                return camera_observation.m_keypoints.empty();
            }
        );

        if(observations.m_camera_observations.empty() || has_empty_camera_observation)
        {
            return;
        }

        std::vector<camera_context::CameraCalibration> selected_cam_info {};
        selected_cam_info.reserve(m_pending_selected_cam_ids.size());

        for(const std::size_t global_id : m_pending_selected_cam_ids)
        {
            if(global_id >= m_info_cameras.size())
            {
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Skipping frame: selected global camera id %zu is out of range for %zu stored calibrations",
                    global_id,
                    m_info_cameras.size()
                );
                return;
            }

            selected_cam_info.push_back(m_info_cameras[global_id]);
        }

        const feature_tracks::CurrentFrameObservation dc_observations =
            distortion_correction::uncertaintyAwareDistortionCorrection(
                selected_cam_info,
                observations
            );

        std::ostringstream dc_counts_stream {};
        for(const auto& camera_observation : dc_observations.m_camera_observations)
        {
            dc_counts_stream << "cam[" << camera_observation.m_camera_id
                             << "]=" << camera_observation.m_keypoints.size() << ' ';
        }

        const std::string dc_counts_str = dc_counts_stream.str();
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Distortion correction: per-camera observations={%s}",
            dc_counts_str.c_str()
        );

        const auto ref_it = std::find(
            m_pending_selected_cam_ids.begin(),
            m_pending_selected_cam_ids.end(),
            static_cast<std::size_t>(response->ref_cam_id)
        );
        if(ref_it == m_pending_selected_cam_ids.end()) { return; }
        const std::size_t ref_local_idx =
            std::distance(m_pending_selected_cam_ids.begin(), ref_it);

        bot_interfaces::msg::PointWithCovarianceArray triangulated_points =
            triangulation::triangulationLOSTU(
                ref_local_idx,
                m_info_cameras,
                dc_observations
            );

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Triangulation output points=%zu",
            triangulated_points.x.size()
        );

        if(triangulated_points.x.empty())
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Skipping publish: triangulation produced zero points"
            );
            return;
        }

        publish3DPoints(std::move(triangulated_points), rclcpp::Time(response->ref_stamp).seconds());
    }

    void VisionFrontend::publish3DPoints(
        bot_interfaces::msg::PointWithCovarianceArray triangulated_points,
        double time
    )
    {
        const rclcpp::Time triangulation_stamp {
            static_cast<rcl_time_point_value_t>(time * 1e9),
            RCL_ROS_TIME
        };

        triangulated_points.header.stamp = triangulation_stamp;
        triangulated_points.header.frame_id = "base_footprint";

        const std::size_t point_count = triangulated_points.x.size();
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Publishing %zu 3D points in frame=%s first_point=(%.3f, %.3f, %.3f)",
            point_count,
            "base_footprint",
            triangulated_points.x.front(),
            triangulated_points.y.front(),
            triangulated_points.z.front()
        );

        m_pts_cov_pub->publish(triangulated_points);

        sensor_msgs::msg::PointCloud2 cloud_msg {};
        cloud_msg.header.stamp = triangulation_stamp;
        cloud_msg.header.frame_id = "base_footprint";

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(point_count);

        sensor_msgs::PointCloud2Iterator<float> x_iter(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> y_iter(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> z_iter(cloud_msg, "z");

        for(std::size_t point_idx {0}; point_idx < point_count; ++point_idx)
        {
            *x_iter = triangulated_points.x[point_idx];
            *y_iter = triangulated_points.y[point_idx];
            *z_iter = triangulated_points.z[point_idx];

            ++x_iter;
            ++y_iter;
            ++z_iter;
        }

        m_pts_pub->publish(cloud_msg);
    }


}   // namespace bot_vision

int main(int argc, char* argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bot_vision::VisionFrontend>("vision_frontend"));
    rclcpp::shutdown();
    return 0;
}
