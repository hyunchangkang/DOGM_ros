// dogm_ros/src/dogm_node.cpp

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits> // Added for numeric_limits

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>

#include "dogm_ros/dynamic_grid_map.h"
#include "dogm_ros/structures.h"

// 색상 보간 함수 (기존과 동일)
// ... (interpolateColor function remains the same) ...
void interpolateColor(double velocity, double& r, double& g, double& b) {
    const double green[3] = {124/255.0, 252/255.0, 154/255.0};
    const double purple[3] = {138/255.0, 43/255.0, 226/255.0};
    const double orange[3] = {255/255.0, 165/255.0, 0/255.0};
    velocity = std::abs(velocity);
    if (velocity < 0.5) { double scale = velocity / 0.5; r = (1.0 - scale) * green[0] + scale * purple[0]; g = (1.0 - scale) * green[1] + scale * purple[1]; b = (1.0 - scale) * green[2] + scale * purple[2]; }
    else if (velocity < 1.5) { double scale = (velocity - 0.5) / 1.0; r = (1.0 - scale) * purple[0] + scale * orange[0]; g = (1.0 - scale) * purple[1] + scale * orange[1]; b = (1.0 - scale) * purple[2] + scale * orange[2]; }
    else { r = orange[0]; g = orange[1]; b = orange[2]; }
}


class DogmNode {
public:
    DogmNode() : nh_(), pnh_("~") {
        loadParams();

        grid_map_ = std::make_unique<DynamicGridMap>(grid_size_, grid_res_, num_particles_,
                                                     process_noise_pos_, process_noise_vel_,
                                                     radar_buffer_size_, min_radar_points_,
                                                     radar_hint_search_radius_,
                                                     use_fsd_, fsd_T_static_, fsd_T_free_,
                                                     use_mc_,
                                                     use_radar_);
        
        grid_pub_   = nh_.advertise<nav_msgs::OccupancyGrid>(grid_topic_, 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);
        
        scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(lidar_topic_, 1, &DogmNode::scanCb, this);

        if (use_radar_) {
            radar_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("/dogm/radar_velocity_viz", 1);
            radar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(radar_topic_, 1, &DogmNode::radarCb, this);
            ROS_INFO("Radar fusion is ENABLED.");
        } else {
            ROS_INFO("Radar fusion is DISABLED.");
        }

        if (use_ego_comp_) {
            odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &DogmNode::odomCb, this);
        }
        
        const double hz = (filter_hz_ > 0.5 ? filter_hz_ : 20.0);
        timer_ = nh_.createTimer(ros::Duration(1.0 / hz), &DogmNode::updateLoop, this);
        ROS_INFO_STREAM("DOGM Node started.");
    }

private:
    void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg) { last_scan_ = msg; has_scan_  = true; }
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) { if (!msg) return; ego_vx_ = msg->twist.twist.linear.x; ego_vy_ = msg->twist.twist.linear.y; }

     // [MODIFIED] Added min/max velocity tracking and printing
     void radarCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (!use_radar_) return;

        last_radar_cloud_.reset(new pcl::PointCloud<mmWaveCloudType>());
        pcl::fromROSMsg(*msg, *last_radar_cloud_);
        has_radar_ = !last_radar_cloud_->points.empty();

        if (!has_radar_) return;

        // --- [NEW] Min/Max Velocity Tracking ---
        float min_vel = std::numeric_limits<float>::max();
        float max_vel = std::numeric_limits<float>::lowest();
        // --- [END NEW] ---

        visualization_msgs::Marker points_viz;
        points_viz.header.frame_id = base_frame_; points_viz.header.stamp = ros::Time::now();
        points_viz.ns = "radar_velocity_status"; points_viz.id = 0;
        points_viz.type = visualization_msgs::Marker::POINTS; points_viz.action = visualization_msgs::Marker::ADD;
        points_viz.pose.orientation.w = 1.0; points_viz.scale.x = 0.04; points_viz.scale.y = 0.04;
        points_viz.lifetime = ros::Duration(0.1);

        for (const auto& pt : last_radar_cloud_->points) {
            geometry_msgs::Point p; p.x = pt.x; p.y = pt.y; p.z = 0.1; points_viz.points.push_back(p);
            std_msgs::ColorRGBA color; color.a = 1.0; double r, g, b; interpolateColor(pt.velocity, r, g, b);
            color.r = r; color.g = g; color.b = b; points_viz.colors.push_back(color);

            // --- [NEW] Update Min/Max ---
            if (pt.velocity < min_vel) min_vel = pt.velocity;
            if (pt.velocity > max_vel) max_vel = pt.velocity;
            // --- [END NEW] ---
        }
        radar_viz_pub_.publish(points_viz);

        // --- [NEW] Print Min/Max Velocity ---
        ROS_INFO("Radar Scan Received: Min Velocity = %.2f m/s, Max Velocity = %.2f m/s", min_vel, max_vel);
        // --- [END NEW] ---
    }


    void updateLoop(const ros::TimerEvent&) {
        if (!has_scan_) return;

        ros::Time now = ros::Time::now();
        double dt = (now - last_update_time_).toSec();
        if (dt <= 0.0) return;
        last_update_time_ = now;
        
        grid_map_->generateMeasurementGrid(last_scan_, (use_radar_ && has_radar_) ? last_radar_cloud_ : nullptr);
        has_radar_ = false;

        auto& pf = grid_map_->getParticleFilter();
        pf.predict(dt, persistence_prob_, velocity_damping_threshold_, velocity_damping_factor_, max_velocity_);
        pf.sortParticlesByGridCell(*grid_map_);
        
        // Pass huge stddev if radar is off to effectively disable radar likelihood
        pf.updateWeights(grid_map_->getMeasurementGrid(),
                         grid_map_->getGrid(),
                         use_radar_ ? radar_noise_stddev_ : 1000.0); 
                         
        grid_map_->updateOccupancy(birth_prob_);
        auto new_borns = grid_map_->generateNewParticles(newborn_vel_stddev_,
                                                         min_dynamic_birth_ratio_,
                                                         max_dynamic_birth_ratio_,
                                                         max_radar_speed_for_scaling_,
                                                         dynamic_newborn_vel_stddev_);
        pf.resample(new_borns);
        grid_map_->calculateVelocityStatistics(static_velocity_threshold_,
                                               max_vel_for_scaling_,
                                               use_ego_comp_,
                                               ego_vx_, ego_vy_);
        nav_msgs::OccupancyGrid grid_msg;
        grid_map_->toOccupancyGridMsg(grid_msg, base_frame_);
        grid_pub_.publish(grid_msg);
        visualization_msgs::MarkerArray marker_msg;
        grid_map_->toMarkerArrayMsg(marker_msg, base_frame_, show_velocity_arrows_);
        marker_pub_.publish(marker_msg);
        
        has_scan_ = false;
    }

    // loadParams (remains the same as the previous correct version)
    void loadParams() {
        pnh_.param("use_radar", use_radar_, true);
        pnh_.param("lidar_topic", lidar_topic_, std::string("/scan"));
        pnh_.param("radar_topic", radar_topic_, std::string("/ti_mmwave/radar_scan_pcl_0"));
        pnh_.param("grid_topic", grid_topic_, std::string("/dogm/grid"));
        pnh_.param("marker_topic", marker_topic_, std::string("/dogm/markers"));
        pnh_.param("base_frame", base_frame_, std::string("base_link"));
        pnh_.param("grid_size", grid_size_, 3.0);
        pnh_.param("grid_resolution", grid_res_, 0.15);
        pnh_.param("filter_update_rate", filter_hz_, 20.0);
        pnh_.param("persistence_prob", persistence_prob_, 0.99);
        pnh_.param("birth_prob", birth_prob_, 0.03);
        pnh_.param("num_particles", num_particles_, 5000);
        pnh_.param("process_noise_pos", process_noise_pos_, 0.05);
        pnh_.param("process_noise_vel", process_noise_vel_, 2.5);
        pnh_.param("max_velocity", max_velocity_, 2.0);
        pnh_.param("newborn_vel_stddev", newborn_vel_stddev_, 0.5);
        pnh_.param("dynamic_newborn_vel_stddev", dynamic_newborn_vel_stddev_, 4.0);
        pnh_.param("min_dynamic_birth_ratio", min_dynamic_birth_ratio_, 0.1);
        pnh_.param("max_dynamic_birth_ratio", max_dynamic_birth_ratio_, 0.9);
        pnh_.param("max_radar_speed_for_scaling", max_radar_speed_for_scaling_, 1.5);
        pnh_.param("velocity_damping_threshold", velocity_damping_threshold_, 0.2);
        pnh_.param("velocity_damping_factor", velocity_damping_factor_, 0.8);
        pnh_.param("static_velocity_threshold", static_velocity_threshold_, 0.6);
        pnh_.param("max_velocity_for_scaling", max_vel_for_scaling_, 0.5);
        pnh_.param("show_velocity_arrows", show_velocity_arrows_, true);
        pnh_.param("use_ego_comp", use_ego_comp_, true);
        pnh_.param("odom_topic", odom_topic_, std::string("/odom"));
        pnh_.param("radar_buffer_size", radar_buffer_size_, 5);
        pnh_.param("min_radar_points", min_radar_points_, 2);
        pnh_.param("radar_hint_search_radius", radar_hint_search_radius_, 1);
        pnh_.param("radar_noise_stddev", radar_noise_stddev_, 0.25);
        pnh_.param("use_false_static_detection", use_fsd_, true);
        pnh_.param("fsd_T_static", fsd_T_static_, 4);
        pnh_.param("fsd_T_free", fsd_T_free_, 10);
        pnh_.param("use_measurement_correction", use_mc_, true);
    }


private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber scan_sub_, odom_sub_, radar_sub_;
    ros::Publisher  grid_pub_, marker_pub_, radar_viz_pub_;
    ros::Timer      timer_;
    std::unique_ptr<DynamicGridMap> grid_map_;

    // Params
    bool use_radar_;
    std::string lidar_topic_, radar_topic_, grid_topic_, marker_topic_, base_frame_, odom_topic_;
    double grid_size_, grid_res_, filter_hz_, persistence_prob_, birth_prob_;
    int    num_particles_;
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

    // State
    sensor_msgs::LaserScan::ConstPtr last_scan_;
    pcl::PointCloud<mmWaveCloudType>::Ptr last_radar_cloud_;
    bool has_scan_{false};
    bool has_radar_{false};
    ros::Time last_update_time_;
    double ego_vx_{0.0}, ego_vy_{0.0};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dogm_node");
    DogmNode node;
    ros::spin();
    return 0;
}