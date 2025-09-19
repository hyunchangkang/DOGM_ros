#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <memory>
#include <algorithm>

#include "dogm_ros/dynamic_grid_map.h"

class DogmNode {
public:
    DogmNode() : nh_(), pnh_("~") {
        loadParams();

        grid_map_ = std::make_unique<DynamicGridMap>(
            grid_size_, grid_res_, num_particles_,
            process_noise_pos_, process_noise_vel_
        );

        grid_pub_   = nh_.advertise<nav_msgs::OccupancyGrid>(grid_topic_, 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);

        scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(lidar_topic_, 1, &DogmNode::scanCb, this);
        if (use_ego_comp_) {
            odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &DogmNode::odomCb, this);
        }

        const double hz = (filter_hz_ > 0.5 ? filter_hz_ : 20.0);
        timer_ = nh_.createTimer(ros::Duration(1.0 / hz), &DogmNode::updateLoop, this);

        has_scan_ = false;
        last_update_time_ = ros::Time::now();
        ROS_INFO_STREAM("dogm_node started. lidar_topic=" << lidar_topic_
                        << ", grid=" << grid_size_ << "m @" << grid_res_ << "m"
                        << ", ego_comp=" << (use_ego_comp_ ? "on" : "off"));
    }

private:
    void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
        last_scan_ = msg;
        has_scan_  = true;
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!msg) return;
        ego_vx_ = msg->twist.twist.linear.x;
        ego_vy_ = msg->twist.twist.linear.y;
    }

    void updateLoop(const ros::TimerEvent&) {
        if (!has_scan_) return;

        ros::Time now = ros::Time::now();
        double dt = (now - last_update_time_).toSec();
        if (dt <= 0.0) return;
        last_update_time_ = now;

        auto& pf = grid_map_->getParticleFilter();

        grid_map_->generateMeasurementGrid(last_scan_);

        // [수정] predict 함수에 max_velocity_ 파라미터 전달
        pf.predict(dt, persistence_prob_,
                   velocity_damping_threshold_, velocity_damping_factor_,
                   max_velocity_);

        pf.sortParticlesByGridCell(*grid_map_);

        pf.updateWeights(grid_map_->getMeasurementGrid());

        grid_map_->updateOccupancy(birth_prob_);

        auto new_borns = grid_map_->generateNewParticles(newborn_vel_stddev_,
                                                         dynamic_birth_ratio_,
                                                         dynamic_newborn_vel_stddev_);

        pf.resample(new_borns);

        grid_map_->calculateVelocityStatistics(static_velocity_threshold_,
                                               mahalanobis_threshold_,
                                               max_variance_threshold_,
                                               max_vel_for_scaling_,
                                               use_ego_comp_,
                                               ego_vx_, ego_vy_);

        nav_msgs::OccupancyGrid grid_msg;
        grid_map_->toOccupancyGridMsg(grid_msg, base_frame_);
        grid_pub_.publish(grid_msg);

        visualization_msgs::MarkerArray marker_msg;
        grid_map_->toMarkerArrayMsg(marker_msg, base_frame_, show_velocity_arrows_);
        marker_pub_.publish(marker_msg);
    }

    void loadParams() {
        pnh_.param("lidar_topic", lidar_topic_, std::string("/scan"));
        pnh_.param("grid_topic", grid_topic_, std::string("/dogm/grid"));
        pnh_.param("marker_topic", marker_topic_, std::string("/dogm/markers"));
        pnh_.param("base_frame", base_frame_, std::string("base_link"));

        pnh_.param("grid_size", grid_size_, 3.0);
        pnh_.param("grid_resolution", grid_res_, 0.2);
        pnh_.param("filter_update_rate", filter_hz_, 20.0);

        pnh_.param("persistence_prob", persistence_prob_, 0.99);
        pnh_.param("birth_prob", birth_prob_, 0.02);

        pnh_.param("num_particles", num_particles_, 200000);
        pnh_.param("process_noise_pos", process_noise_pos_, 0.05);
        pnh_.param("process_noise_vel", process_noise_vel_, 0.50);
        
        // [추가] max_velocity 파라미터 로딩
        pnh_.param("max_velocity", max_velocity_, 2.0);

        pnh_.param("newborn_vel_stddev", newborn_vel_stddev_, 0.5);

        pnh_.param("dynamic_birth_ratio", dynamic_birth_ratio_, 0.3);
        pnh_.param("dynamic_newborn_vel_stddev", dynamic_newborn_vel_stddev_, 3.0);
        pnh_.param("velocity_damping_threshold", velocity_damping_threshold_, 0.6);
        pnh_.param("velocity_damping_factor", velocity_damping_factor_, 0.85);

        pnh_.param("static_velocity_threshold", static_velocity_threshold_, 0.6);
        pnh_.param("mahalanobis_threshold", mahalanobis_threshold_, 3.0);
        pnh_.param("static_velocity_threshold", static_velocity_threshold_, 0.6);
        pnh_.param("max_variance_threshold", max_variance_threshold_, 0.4);
        pnh_.param("max_variance_threshold", max_variance_threshold_, 0.4);
        pnh_.param("max_velocity_for_scaling", max_vel_for_scaling_, 1.4);

        pnh_.param("show_velocity_arrows", show_velocity_arrows_, true);
        pnh_.param("use_ego_comp", use_ego_comp_, true);
        pnh_.param("odom_topic", odom_topic_, std::string("/odom"));
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber scan_sub_, odom_sub_;
    ros::Publisher  grid_pub_, marker_pub_;
    ros::Timer      timer_;

    std::unique_ptr<DynamicGridMap> grid_map_;

    // Params
    std::string lidar_topic_, grid_topic_, marker_topic_, base_frame_, odom_topic_;
    double grid_size_{3.0};
    double grid_res_{0.2};
    double filter_hz_{20.0};
    double persistence_prob_{0.99};
    double birth_prob_{0.02};

    int    num_particles_{200000};
    double process_noise_pos_{0.05};
    double process_noise_vel_{0.50};
    
    // [추가] max_velocity 멤버 변수
    double max_velocity_{2.0};

    double newborn_vel_stddev_{0.5};
    double dynamic_birth_ratio_{0.3};
    double dynamic_newborn_vel_stddev_{3.0};
    double velocity_damping_threshold_{0.6};
    double velocity_damping_factor_{0.85};

    double static_velocity_threshold_{0.6};
    double mahalanobis_threshold_{3.0};
    double max_variance_threshold_{0.4};
    double max_vel_for_scaling_{1.4};

    bool show_velocity_arrows_{true};
    bool use_ego_comp_{true};

    // State
    sensor_msgs::LaserScan::ConstPtr last_scan_;
    bool has_scan_{false};
    ros::Time last_update_time_;
    double ego_vx_{0.0}, ego_vy_{0.0};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dogm_node");
    DogmNode node;
    ros::spin();
    return 0;
}