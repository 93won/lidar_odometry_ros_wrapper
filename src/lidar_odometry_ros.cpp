#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cstring>  // for memcpy
#include <cmath>    // for std::isfinite
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

// Direct include - no more dynamic loading!
#include "../lidar_odometry/src/processing/Estimator.h"
#include "../lidar_odometry/src/database/LidarFrame.h"
#include "../lidar_odometry/src/util/Config.h"

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
        
        // Start processing thread
        processing_thread_ = std::thread(&LidarOdometryRosWrapper::processing_loop, this);
            
        RCLCPP_INFO(this->get_logger(), "Simple LiDAR Odometry Ready!");
    }
    
    ~LidarOdometryRosWrapper() {
        // Stop processing thread
        should_stop_ = true;
        queue_cv_.notify_all();
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }

private:
    /**
     * @brief Convert ROS PointCloud2 message to internal point cloud format
     */
    lidar_odometry::util::PointCloudPtr convert_ros_to_internal(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_cloud) {
        auto internal_cloud = std::make_shared<lidar_odometry::util::PointCloud>();
        
        // Parse the PointCloud2 message fields
        int x_offset = -1, y_offset = -1, z_offset = -1;
        for (const auto& field : ros_cloud->fields) {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
        }
        
        if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid PointCloud2 format: missing x, y, or z fields");
            return internal_cloud;
        }
        
        internal_cloud->reserve(ros_cloud->width * ros_cloud->height);
        
        // Extract points from the data buffer
        const uint8_t* data_ptr = ros_cloud->data.data();
        for (size_t i = 0; i < ros_cloud->width * ros_cloud->height; ++i) {
            const uint8_t* point_data = data_ptr + i * ros_cloud->point_step;
            
            float x, y, z;
            memcpy(&x, point_data + x_offset, sizeof(float));
            memcpy(&y, point_data + y_offset, sizeof(float));
            memcpy(&z, point_data + z_offset, sizeof(float));
            
            // Filter out invalid points
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                internal_cloud->push_back(x, y, z);
            }
        }
        
        return internal_cloud;
    }
    
    /**
     * @brief Convert internal point cloud to ROS PointCloud2 message directly (no PCL)
     */
    sensor_msgs::msg::PointCloud2 convert_internal_to_ros(
        const lidar_odometry::util::PointCloudPtr& internal_cloud,
        const std::string& frame_id = "base_link") {
        
        sensor_msgs::msg::PointCloud2 cloud_msg;
        
        if (!internal_cloud || internal_cloud->empty()) {
            cloud_msg.header.frame_id = frame_id;
            cloud_msg.height = 1;
            cloud_msg.width = 0;
            cloud_msg.is_dense = true;
            return cloud_msg;
        }
        
        // Set up the PointCloud2 message structure
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.height = 1;
        cloud_msg.width = internal_cloud->size();
        cloud_msg.is_dense = true;
        
        // Define fields for x, y, z coordinates
        cloud_msg.fields.resize(3);
        
        cloud_msg.fields[0].name = "x";
        cloud_msg.fields[0].offset = 0;
        cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[0].count = 1;
        
        cloud_msg.fields[1].name = "y";
        cloud_msg.fields[1].offset = 4;
        cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[1].count = 1;
        
        cloud_msg.fields[2].name = "z";
        cloud_msg.fields[2].offset = 8;
        cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[2].count = 1;
        
        cloud_msg.point_step = 12; // 3 * sizeof(float32)
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        
        // Allocate memory for the data
        cloud_msg.data.resize(cloud_msg.row_step);
        
        // Copy the point data
        uint8_t* data_ptr = cloud_msg.data.data();
        for (size_t i = 0; i < internal_cloud->size(); ++i) {
            const auto& point = (*internal_cloud)[i];
            
            // Copy x, y, z coordinates as float32
            memcpy(data_ptr + i * cloud_msg.point_step + 0, &point.x, sizeof(float));
            memcpy(data_ptr + i * cloud_msg.point_step + 4, &point.y, sizeof(float));
            memcpy(data_ptr + i * cloud_msg.point_step + 8, &point.z, sizeof(float));
        }
        
        return cloud_msg;
    }

    void load_config() {
        // Get config file parameter
        std::string config_file = this->get_parameter("config_file").as_string();
        
        if (config_file.empty()) {
            RCLCPP_WARN(this->get_logger(), "No config file specified, using default parameters");
        }
        
        RCLCPP_INFO(this->get_logger(), "Attempting to load config from: %s", config_file.c_str());
        
        try {
            if (!config_file.empty()) {
                // Use ConfigManager to load YAML configuration
                lidar_odometry::util::ConfigManager::instance().load_from_file(config_file);
                config_ = lidar_odometry::util::ConfigManager::instance().get_config();
                
                RCLCPP_INFO(this->get_logger(), "Successfully loaded config from YAML file");
            } else {
                // Use default configuration
                config_ = lidar_odometry::util::SystemConfig{};
                RCLCPP_WARN(this->get_logger(), "Using default config parameters");
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML config: %s", e.what());
            RCLCPP_WARN(this->get_logger(), "Using default config parameters as fallback");
            
            // Fallback to default config
            config_ = lidar_odometry::util::SystemConfig{};
        }
    }
    
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Simply add message to queue - fast callback
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            msg_queue_.push_back(msg);
            
            // No frame dropping - process all frames
            if (msg_queue_.size() > 50) {
                RCLCPP_WARN(this->get_logger(), "Queue size large: %zu - consider optimizing processing", msg_queue_.size());
            }
        }
        
        // Notify processing thread
        queue_cv_.notify_one();
    }
    
    void processing_loop() {
        RCLCPP_INFO(this->get_logger(), "Processing thread started");
        
        while (!should_stop_) {
            sensor_msgs::msg::PointCloud2::SharedPtr msg;
            
            // Wait for new message
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this] { return !msg_queue_.empty() || should_stop_; });
                
                if (should_stop_) break;
                
                msg = msg_queue_.front();
                msg_queue_.pop_front();
            }
            
            // Process the frame (original callback logic)
            process_frame(msg);
        }
        
        RCLCPP_INFO(this->get_logger(), "Processing thread stopped");
    }
    
    void process_frame(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
        frame_count_++;
        
        // Lazy initialization of estimator
        if (!estimator_) {
            estimator_ = std::make_shared<lidar_odometry::processing::Estimator>(config_);
            RCLCPP_INFO(this->get_logger(), "Estimator initialized on first frame");
        }
        
        // Convert ROS message to internal format directly
        auto internal_cloud = convert_ros_to_internal(msg);
        
        RCLCPP_INFO(this->get_logger(), "Frame #%d - %zu points", frame_count_, internal_cloud->size());
        
        // Create LidarFrame with internal cloud format
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        auto frame = std::make_shared<lidar_odometry::database::LidarFrame>(
            frame_count_, timestamp, internal_cloud);
        
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
        
        // Publish TF transforms using rosbag timestamp
        geometry_msgs::msg::TransformStamped odom_to_base_tf;
        odom_to_base_tf.header = odom_msg.header;
        odom_to_base_tf.child_frame_id = odom_msg.child_frame_id;
        odom_to_base_tf.transform.translation.x = t.x();
        odom_to_base_tf.transform.translation.y = t.y();
        odom_to_base_tf.transform.translation.z = t.z();
        odom_to_base_tf.transform.rotation = odom_msg.pose.pose.orientation;
        
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
        
        // Send both transforms at once
        std::vector<geometry_msgs::msg::TransformStamped> transforms = {map_to_odom_tf, odom_to_base_tf};
        tf_broadcaster_->sendTransform(transforms);
    }
    
    void publish_map_points(const std_msgs::msg::Header& header) {
        auto map_cloud = estimator_->get_local_map();

        spdlog::info("Publishing map points - count: {}", map_cloud ? map_cloud->size() : 0);
        if (map_cloud && !map_cloud->empty()) {
            // Convert internal cloud to ROS message directly (no PCL)
            auto map_msg = convert_internal_to_ros(
                std::const_pointer_cast<lidar_odometry::util::PointCloud>(map_cloud), 
                "odom");
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
        
        // Transform features to world coordinates
        Eigen::Matrix4f transform_matrix = current_frame_->get_pose().matrix().cast<float>();
        
        auto world_features = std::make_shared<lidar_odometry::util::PointCloud>();
        world_features->reserve(feature_cloud->size());
        
        for (size_t i = 0; i < feature_cloud->size(); ++i) {
            const auto& point = (*feature_cloud)[i];
            // Transform point to world coordinates
            Eigen::Vector4f local_point(point.x, point.y, point.z, 1.0f);
            Eigen::Vector4f world_point = transform_matrix * local_point;
            
            world_features->push_back(world_point.x(), world_point.y(), world_point.z());
        }
        
        // Convert to ROS message directly (no PCL)
        auto feature_msg = convert_internal_to_ros(world_features, "odom");
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
    
    // Core components
    std::shared_ptr<lidar_odometry::processing::Estimator> estimator_;
    lidar_odometry::util::SystemConfig config_;
    
    // Store current frame for feature publishing
    std::shared_ptr<lidar_odometry::database::LidarFrame> current_frame_;
    
    // ROS components
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr features_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Threading components
    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> msg_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread processing_thread_;
    std::atomic<bool> should_stop_{false};
    
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
