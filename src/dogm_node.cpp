// dogm_ros/src/dogm_node.cpp

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h> // Keep for DOGM markers
#include <string>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits> // Keep for min/max if needed elsewhere

#include <sensor_msgs/PointCloud2.h>
// #include <visualization_msgs/Marker.h> // No longer needed here
#include <pcl_conversions/pcl_conversions.h> // Still needed for conversion

// Include necessary headers from the package
#include "dogm_ros/dynamic_grid_map.h"
#include "dogm_ros/structures.h"

// [REMOVED] interpolateColor function is no longer needed in this file

// Main ROS node class for the DOGM filter
class DogmNode
{
public:
    // Constructor
    DogmNode() : nh_(), pnh_("~")
    {
        loadParams(); // Load parameters from YAML file

        // Initialize the Dynamic Grid Map object
        grid_map_ = std::make_unique<DynamicGridMap>(
            grid_size_, grid_res_, num_particles_,
            process_noise_pos_, process_noise_vel_,
            radar_buffer_size_, min_radar_points_,
            radar_hint_search_radius_,
            use_fsd_, fsd_T_static_, fsd_T_free_,
            use_mc_,
            use_radar_,
            lidar_point_counts_
        );

        // Setup publishers for grid map and DOGM markers
        grid_pub_   = nh_.advertise<nav_msgs::OccupancyGrid>(grid_topic_, 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1); // For DOGM cells/arrows

        // Setup subscriber for LiDAR scan data
        scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(lidar_topic_, 1, &DogmNode::scanCb, this);

        // Setup subscriber for Radar data if enabled
        if (use_radar_) {
            // [REMOVED] radar_viz_pub_ is removed
            radar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(radar_topic_, 1, &DogmNode::radarCb, this);
            ROS_INFO("Radar fusion is ENABLED.");
        } else {
            ROS_INFO("Radar fusion is DISABLED.");
        }

        // Setup subscriber for Odometry data if ego-motion compensation is enabled
        if (use_ego_comp_) {
            odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &DogmNode::odomCb, this);
        }

        // Setup timer for the main filter update loop
        const double hz = (filter_hz_ > 0.5 ? filter_hz_ : 10.0);
        timer_ = nh_.createTimer(ros::Duration(1.0 / hz), &DogmNode::updateLoop, this);
        ROS_INFO_STREAM("DOGM Node started at " << hz << " Hz.");

        // [REMOVED] Calculation of radar_viz_lifetime_ is removed
    }

private:
    // Callback for LiDAR scan messages
    void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        last_scan_ = msg;
        has_scan_  = true;
    }

    // Callback for Odometry messages
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        if (!msg) return;
        ego_vx_ = msg->twist.twist.linear.x;
        ego_vy_ = msg->twist.twist.linear.y;
    }

     // Callback for Radar PointCloud2 messages
     // [MODIFIED] Removed visualization logic, kept PCL conversion and flag setting
     void radarCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
     {
        // Ignore if radar fusion is disabled
        if (!use_radar_) return;

        // Convert ROS message to PCL point cloud
        last_radar_cloud_.reset(new pcl::PointCloud<mmWaveCloudType>());
        pcl::fromROSMsg(*msg, *last_radar_cloud_);

        // Set flag indicating whether new radar data is available and non-empty
        has_radar_ = !last_radar_cloud_->points.empty();

        // [REMOVED] All visualization marker creation and publishing code is removed
        // [REMOVED] Min/Max velocity tracking and printing is removed (can be done in viz node if needed)
    }

    // Main filter update loop triggered by the timer
    void updateLoop(const ros::TimerEvent&)
    {
        // Only run if a new LiDAR scan has been received
        if (!has_scan_) return;

        // Calculate time delta since last update
        ros::Time now = ros::Time::now();
        double dt = (now - last_update_time_).toSec();
        if (dt <= 0.0) return;
        last_update_time_ = now;

        // 1. Generate Measurement Grid from LiDAR & Process Radar Hints
        grid_map_->generateMeasurementGrid(last_scan_, (use_radar_ && has_radar_) ? last_radar_cloud_ : nullptr);
        has_radar_ = false; // Consume the radar data for this cycle

        // Get reference to the particle filter
        auto& pf = grid_map_->getParticleFilter();

        // 2. Predict particle states
        pf.predict(dt, persistence_prob_, velocity_damping_threshold_, velocity_damping_factor_, max_velocity_);

        // 3. Sort particles by grid cell index
        pf.sortParticlesByGridCell(*grid_map_);

        // 4. Update particle weights
        pf.updateWeights(grid_map_->getMeasurementGrid(),
                         grid_map_->getGrid(),
                         use_radar_ ? radar_noise_stddev_ : 1000.0);

        // 5. Update grid cell occupancy
        grid_map_->updateOccupancy(birth_prob_);

        // 6. Generate new particles
        auto new_borns = grid_map_->generateNewParticles(
            newborn_vel_stddev_,
            min_dynamic_birth_ratio_, max_dynamic_birth_ratio_,
            max_radar_speed_for_scaling_,
            dynamic_newborn_vel_stddev_
        );

        // 7. Resample particles
        pf.resample(new_borns);

        // 8. Calculate velocity statistics and classify cells
        grid_map_->calculateVelocityStatistics(
            static_velocity_threshold_, max_vel_for_scaling_,
            use_ego_comp_, ego_vx_, ego_vy_
        );

        // 9. Publish results (OccupancyGrid and DOGM MarkerArray)
        nav_msgs::OccupancyGrid grid_msg;
        grid_map_->toOccupancyGridMsg(grid_msg, base_frame_);
        grid_pub_.publish(grid_msg);

        visualization_msgs::MarkerArray marker_msg;
        grid_map_->toMarkerArrayMsg(marker_msg, base_frame_, show_velocity_arrows_);
        marker_pub_.publish(marker_msg);

        // Reset scan flag
        has_scan_ = false;
    }

    // Load parameters from the ROS parameter server
    void loadParams()
    {
        // Load parameters relevant to DOGM core logic
        pnh_.param("use_radar", use_radar_, true);
        pnh_.param("lidar_topic", lidar_topic_, std::string("/scan"));
        pnh_.param("radar_topic", radar_topic_, std::string("/ti_mmwave/radar_scan_pcl_0")); // Still needed for subscription
        pnh_.param("grid_topic", grid_topic_, std::string("/dogm/grid"));
        pnh_.param("marker_topic", marker_topic_, std::string("/dogm/markers")); // For DOGM markers
        pnh_.param("base_frame", base_frame_, std::string("base_link"));
        pnh_.param("grid_size", grid_size_, 3.0);
        pnh_.param("grid_resolution", grid_res_, 0.15);
        pnh_.param("filter_update_rate", filter_hz_, 10.0);
        pnh_.param("lidar_point_counts", lidar_point_counts_, 3);
        pnh_.param("num_particles", num_particles_, 20000);
        pnh_.param("process_noise_pos", process_noise_pos_, 0.03);
        pnh_.param("process_noise_vel", process_noise_vel_, 0.1);
        pnh_.param("max_velocity", max_velocity_, 1.5);
        pnh_.param("persistence_prob", persistence_prob_, 0.99);
        pnh_.param("birth_prob", birth_prob_, 0.02);
        pnh_.param("newborn_vel_stddev", newborn_vel_stddev_, 0.3);
        pnh_.param("dynamic_newborn_vel_stddev", dynamic_newborn_vel_stddev_, 1.5);
        pnh_.param("velocity_damping_threshold", velocity_damping_threshold_, 0.6);
        pnh_.param("velocity_damping_factor", velocity_damping_factor_, 0.2);
        pnh_.param("static_velocity_threshold", static_velocity_threshold_, 0.3);
        pnh_.param("radar_buffer_size", radar_buffer_size_, 5);
        pnh_.param("min_radar_points", min_radar_points_, 2);
        pnh_.param("radar_hint_search_radius", radar_hint_search_radius_, 2);
        pnh_.param("min_dynamic_birth_ratio", min_dynamic_birth_ratio_, 0.1);
        pnh_.param("max_dynamic_birth_ratio", max_dynamic_birth_ratio_, 0.9);
        pnh_.param("max_radar_speed_for_scaling", max_radar_speed_for_scaling_, 1.5);
        pnh_.param("radar_noise_stddev", radar_noise_stddev_, 0.25);
        pnh_.param("use_false_static_detection", use_fsd_, true);
        pnh_.param("fsd_T_static", fsd_T_static_, 4);
        pnh_.param("fsd_T_free", fsd_T_free_, 10);
        pnh_.param("use_measurement_correction", use_mc_, true);
        pnh_.param("max_velocity_for_scaling", max_vel_for_scaling_, 2.0);
        pnh_.param("show_velocity_arrows", show_velocity_arrows_, true);
        pnh_.param("use_ego_comp", use_ego_comp_, true);
        pnh_.param("odom_topic", odom_topic_, std::string("/odom"));

        // [REMOVED] Parameters related only to radar visualization are removed
    }

private:
    // ROS NodeHandles
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Subscribers and Publishers
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber radar_sub_; // Keep subscriber to receive data
    ros::Publisher  grid_pub_;  // DOGM grid
    ros::Publisher  marker_pub_; // DOGM markers (cells/arrows)
    // [REMOVED] ros::Publisher radar_viz_pub_;
    ros::Timer      timer_;

    // DOGM Core object
    std::unique_ptr<DynamicGridMap> grid_map_;

    // Parameters loaded from YAML (Visualization params removed)
    bool use_radar_;
    std::string lidar_topic_, radar_topic_, grid_topic_, marker_topic_, base_frame_, odom_topic_;
    double grid_size_, grid_res_, filter_hz_, persistence_prob_, birth_prob_;
    int    num_particles_;
    int    lidar_point_counts_;
    double process_noise_pos_, process_noise_vel_, max_velocity_, newborn_vel_stddev_;
    double min_dynamic_birth_ratio_, max_dynamic_birth_ratio_, max_radar_speed_for_scaling_;
    double dynamic_newborn_vel_stddev_;
    double velocity_damping_threshold_, velocity_damping_factor_;
    double static_velocity_threshold_, max_vel_for_scaling_;
    bool show_velocity_arrows_, use_ego_comp_;
    int radar_buffer_size_;
    int min_radar_points_;
    int radar_hint_search_radius_;
    double radar_noise_stddev_;
    bool   use_fsd_, use_mc_;
    int    fsd_T_static_, fsd_T_free_;
    // [REMOVED] double radar_viz_lifetime_;

    // State variables
    sensor_msgs::LaserScan::ConstPtr last_scan_;
    pcl::PointCloud<mmWaveCloudType>::Ptr last_radar_cloud_; // Still needed to pass to grid_map_
    bool has_scan_{false};
    bool has_radar_{false}; // Still needed to know if data is available
    ros::Time last_update_time_;
    double ego_vx_{0.0}, ego_vy_{0.0};
};

// Main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dogm_node");
    DogmNode node;
    ros::spin();
    return 0;
}