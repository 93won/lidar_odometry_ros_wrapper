#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Direct include - no more dynamic loading!
#include "../lidar_odometry/src/processing/Estimator.h"
#include "../lidar_odometry/src/database/LidarFrame.h"
#include "../lidar_odometry/src/util/Config.h"

using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;

class LidarOdometryRosWrapper : public rclcpp::Node {
public:
    LidarOdometryRosWrapper() : Node("simple_lidar_odometry") {

        RCLCPP_INFO(this->get_logger(), "=== Simple LiDAR Odometry Started ===");
        
        // Declare parameters
        this->declare_parameter("config_file", "");
        
        // Load config
        load_config();
        
        // Create estimator directly - delay creation to avoid constructor issues
        // estimator_ will be created on first frame
        estimator_ = nullptr;
        
        // Publishers - using lidar_odometry namespace
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);
        features_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("feature_points", 10);
        trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        current_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("current_cloud", 10);
        
        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        
        // Subscriber
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&LidarOdometryRosWrapper::cloud_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Simple LiDAR Odometry Ready!");
    }

private:
    void load_config() {
        // Get config file parameter
        std::string config_file = this->get_parameter("config_file").as_string();
        
        if (config_file.empty()) {
            RCLCPP_WARN(this->get_logger(), "No config file specified, using default parameters");
        }
        
        RCLCPP_INFO(this->get_logger(), "Attempting to load config from: %s", config_file.c_str());
        
        try {
            if (!config_file.empty()) {
                // Use ConfigManager to load YAML configuration (like KittiPlayer)
                lidar_odometry::util::ConfigManager::instance().load_from_file(config_file);
                const auto& system_config = lidar_odometry::util::ConfigManager::instance().get_config();
                
                // Convert SystemConfig to EstimatorConfig (like KittiPlayer does)
                config_ = lidar_odometry::processing::EstimatorConfig{};
                
                // Map system config values to estimator config (exactly like KittiPlayer)
                config_.max_icp_iterations = system_config.max_iterations;
                config_.icp_translation_threshold = system_config.translation_threshold;
                config_.icp_rotation_threshold = system_config.rotation_threshold;
                config_.correspondence_distance = system_config.max_correspondence_distance;
                config_.voxel_size = system_config.voxel_size;
                config_.map_voxel_size = system_config.map_voxel_size;
                config_.max_range = system_config.max_range;
                config_.keyframe_distance_threshold = system_config.keyframe_distance_threshold;
                config_.keyframe_rotation_threshold = system_config.keyframe_rotation_threshold;
                config_.max_solver_iterations = system_config.max_solver_iterations;
                config_.parameter_tolerance = system_config.parameter_tolerance;
                config_.function_tolerance = system_config.function_tolerance;
                config_.max_correspondence_distance = system_config.max_correspondence_distance;
                config_.min_correspondence_points = system_config.min_correspondence_points;
                
                // Map robust estimation settings
                config_.use_adaptive_m_estimator = system_config.use_adaptive_m_estimator;
                config_.loss_type = system_config.loss_type;
                config_.scale_method = system_config.scale_method;
                config_.fixed_scale_factor = system_config.fixed_scale_factor;
                config_.mad_multiplier = system_config.mad_multiplier;
                config_.min_scale_factor = system_config.min_scale_factor;
                config_.max_scale_factor = system_config.max_scale_factor;
                
                // Map PKO/GMM parameters (CRITICAL: was missing in KittiPlayer!)
                config_.num_alpha_segments = system_config.num_alpha_segments;
                config_.truncated_threshold = system_config.truncated_threshold;
                config_.gmm_components = system_config.gmm_components;
                config_.gmm_sample_size = system_config.gmm_sample_size;
                config_.pko_kernel_type = system_config.pko_kernel_type;
                
                RCLCPP_INFO(this->get_logger(), "Successfully loaded config from YAML file");
                RCLCPP_INFO(this->get_logger(), "Scale method: %s, Voxel size: %.1f", 
                           config_.scale_method.c_str(), config_.voxel_size);
            } else {
                throw std::runtime_error("No config file specified");
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML config: %s", e.what());
            RCLCPP_WARN(this->get_logger(), "Using default KITTI config parameters as fallback");
            
            // Fallback to default config
            config_ = lidar_odometry::processing::EstimatorConfig{};
            
            // Set KITTI-suitable parameters as fallback
            config_.max_icp_iterations = 10;
            config_.icp_translation_threshold = 0.005f;
            config_.icp_rotation_threshold = 0.005f;
            config_.correspondence_distance = 1.0f;
            config_.voxel_size = 0.4f;
            config_.map_voxel_size = 0.4f;
            config_.max_range = 40.0f;
            config_.keyframe_distance_threshold = 5.0f;
            config_.keyframe_rotation_threshold = 0.3f;
            config_.max_solver_iterations = 10;
            config_.parameter_tolerance = 1e-6;
            config_.function_tolerance = 1e-6;
            config_.max_correspondence_distance = 1.0f;
            config_.min_correspondence_points = 50;
            
            // Robust estimation settings
            config_.use_adaptive_m_estimator = true;
            config_.loss_type = "huber";
            config_.scale_method = "PKO";
            config_.fixed_scale_factor = 1.0f;
            config_.mad_multiplier = 1.4826f;
            config_.min_scale_factor = 0.1f;
            config_.max_scale_factor = 10.0f;
            
            // PKO/GMM parameters
            config_.num_alpha_segments = 100;
            config_.truncated_threshold = 10.0f;
            config_.gmm_components = 2;
            config_.gmm_sample_size = 100;
            config_.pko_kernel_type = "huber";
        }
    }
    
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        frame_count_++;
        
        // Lazy initialization of estimator
        if (!estimator_) {
            estimator_ = std::make_shared<lidar_odometry::processing::Estimator>(config_);
            RCLCPP_INFO(this->get_logger(), "Estimator initialized on first frame");
        }
        
        // Convert to PCL
        PointCloud::Ptr cloud(new PointCloud());
        pcl::fromROSMsg(*msg, *cloud);
        
        RCLCPP_INFO(this->get_logger(), "Frame #%d - %zu points", frame_count_, cloud->size());
        
        // Create LidarFrame directly
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        auto frame = std::make_shared<lidar_odometry::database::LidarFrame>(
            frame_count_, timestamp, cloud);
        
        // Store current frame for feature publishing
        current_frame_ = frame;
        
        // Process frame
        bool success = estimator_->process_frame(frame);
        
        if (success) {
            // Get current pose
            const auto& pose = estimator_->get_current_pose();
            
            // Publish odometry
            publish_odometry(msg, pose);
            
            // Publish visualization data
            spdlog::info("Publishing visualization data");
            publish_map_points(msg->header);
            publish_current_cloud(msg);
            publish_features(msg->header);
            publish_trajectory(msg->header);
            
            RCLCPP_INFO(this->get_logger(), "Processing successful - pose: [%.3f, %.3f, %.3f]", 
                       pose.translation().x(), pose.translation().y(), pose.translation().z());
        } else {
            // Even if processing failed, publish current cloud for debugging
            publish_current_cloud(msg);
            RCLCPP_WARN(this->get_logger(), "Processing failed");
        }
    }
    
    void publish_odometry(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, 
                         const lidar_odometry::util::SE3f& pose) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = msg->header;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // Position
        const auto& t = pose.translation();
        odom_msg.pose.pose.position.x = t.x();
        odom_msg.pose.pose.position.y = t.y();
        odom_msg.pose.pose.position.z = t.z();
        
        // Orientation
        const auto q = pose.unit_quaternion();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        
        odom_pub_->publish(odom_msg);
        
        // Publish TF
        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom_msg.header;
        tf.child_frame_id = odom_msg.child_frame_id;
        tf.transform.translation.x = t.x();
        tf.transform.translation.y = t.y();
        tf.transform.translation.z = t.z();
        tf.transform.rotation = odom_msg.pose.pose.orientation;
        
        tf_broadcaster_->sendTransform(tf);
        
        // Also publish map -> odom transform (identity for now)
        geometry_msgs::msg::TransformStamped map_to_odom_tf;
        map_to_odom_tf.header.stamp = msg->header.stamp;
        map_to_odom_tf.header.frame_id = "map";
        map_to_odom_tf.child_frame_id = "odom";
        map_to_odom_tf.transform.translation.x = 0.0;
        map_to_odom_tf.transform.translation.y = 0.0;
        map_to_odom_tf.transform.translation.z = 0.0;
        map_to_odom_tf.transform.rotation.w = 1.0;
        map_to_odom_tf.transform.rotation.x = 0.0;
        map_to_odom_tf.transform.rotation.y = 0.0;
        map_to_odom_tf.transform.rotation.z = 0.0;
        
        tf_broadcaster_->sendTransform(map_to_odom_tf);
    }
    
    void publish_map_points(const std_msgs::msg::Header& header) {
        auto map_cloud = estimator_->get_local_map();

        spdlog::info("Publishing map points - count: {}", map_cloud ? map_cloud->size() : 0);
        if (map_cloud && !map_cloud->empty()) {
            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::toROSMsg(*map_cloud, map_msg);
            map_msg.header = header;
            map_msg.header.frame_id = "odom";
            map_pub_->publish(map_msg);
            RCLCPP_INFO(this->get_logger(), "Published map points: %zu", map_cloud->size());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "No map points to publish");
        }
    }
    
    void publish_current_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
        // Republish current cloud with odom frame for visualization
        auto current_msg = *msg;
        current_msg.header.frame_id = "base_link";
        current_cloud_pub_->publish(current_msg);
    }
    
    void publish_features(const std_msgs::msg::Header& header) {
        // Get feature cloud from current frame
        if (!current_frame_) {
            RCLCPP_DEBUG(this->get_logger(), "No current frame available for features");
            return;
        }
        
        auto feature_cloud = current_frame_->get_feature_cloud();
        if (!feature_cloud || feature_cloud->empty()) {
            RCLCPP_DEBUG(this->get_logger(), "No feature points available");
            return;
        }
        
        // Transform features to world coordinates (like PangolinViewer does)
        Eigen::Matrix4f transform_matrix = current_frame_->get_pose().matrix().cast<float>();
        
        PointCloud::Ptr world_features(new PointCloud());
        world_features->reserve(feature_cloud->size());
        
        for (const auto& point : feature_cloud->points) {
            // Transform point to world coordinates
            Eigen::Vector4f local_point(point.x, point.y, point.z, 1.0f);
            Eigen::Vector4f world_point = transform_matrix * local_point;
            
            PointType world_pt;
            world_pt.x = world_point.x();
            world_pt.y = world_point.y(); 
            world_pt.z = world_point.z();
            world_features->points.push_back(world_pt);
        }
        
        world_features->width = world_features->points.size();
        world_features->height = 1;
        world_features->is_dense = false;
        
        // Publish transformed features
        sensor_msgs::msg::PointCloud2 feature_msg;
        pcl::toROSMsg(*world_features, feature_msg);
        feature_msg.header = header;
        feature_msg.header.frame_id = "odom";  // World coordinate frame
        features_pub_->publish(feature_msg);
        
        RCLCPP_INFO(this->get_logger(), "Published feature points: %zu", world_features->size());
    }
    
    void publish_trajectory(const std_msgs::msg::Header& header) {
        // Simple trajectory from current pose
        nav_msgs::msg::Path path;
        path.header = header;
        path.header.frame_id = "odom";
        
        const auto& pose = estimator_->get_current_pose();
        
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        
        const auto& t = pose.translation();
        const auto q = pose.unit_quaternion();
        
        pose_stamped.pose.position.x = t.x();
        pose_stamped.pose.position.y = t.y();
        pose_stamped.pose.position.z = t.z();
        pose_stamped.pose.orientation.w = q.w();
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        
        trajectory_.push_back(pose_stamped);
        
        // Limit trajectory size
        if (trajectory_.size() > 1000) {
            trajectory_.erase(trajectory_.begin());
        }
        
        path.poses = trajectory_;
        trajectory_pub_->publish(path);
    }
    
    // Simple members - no complex threading!
    std::shared_ptr<lidar_odometry::processing::Estimator> estimator_;
    lidar_odometry::processing::EstimatorConfig config_;
    
    // Store current frame for feature publishing
    std::shared_ptr<lidar_odometry::database::LidarFrame> current_frame_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr features_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    int frame_count_ = 0;
    std::vector<geometry_msgs::msg::PoseStamped> trajectory_;
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<LidarOdometryRosWrapper>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
