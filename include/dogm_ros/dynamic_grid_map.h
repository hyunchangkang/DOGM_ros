#ifndef DYNAMIC_GRID_MAP_H
#define DYNAMIC_GRID_MAP_H

#include <vector>
#include <memory>
#include <string>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>

#include "dogm_ros/structures.h"
#include "dogm_ros/particle_filter.h"

// Class responsible for managing the dynamic grid map state and updates
class DynamicGridMap {
public:
    // Constructor
    DynamicGridMap(double grid_size, double resolution, int num_particles,
                   double process_noise_pos, double process_noise_vel,
                   int radar_buffer_size, int min_radar_points,
                   int radar_hint_search_radius,
                   bool use_fsd, int fsd_T_static, int fsd_T_free,
                   bool use_mc,
                   bool use_radar,
                   int lidar_hit_point,
                   double lidar_noise_stddev,
                   double mode_cluster_velocity_thresh,
                   double particle_static_vel_thresh, // new
                   double radar_static_vel_thresh);    // new
    ~DynamicGridMap() = default; // Default destructor

    // Generates the measurement grid based on sensor inputs
    void generateMeasurementGrid(const sensor_msgs::LaserScan::ConstPtr& scan,
                                 const pcl::PointCloud<mmWaveCloudType>::ConstPtr& radar_cloud);

    // Updates cell occupancy probabilities using Dempster-Shafer
    void updateOccupancy(double birth_prob);

    // Generates new particles for cells with high birth probability
    // Signature reverted to include max_radar_speed_for_scaling
    std::vector<Particle> generateNewParticles(double newborn_vel_stddev,
                                               double min_dynamic_birth_ratio,
                                               double max_dynamic_birth_ratio,
                                               double max_radar_speed_for_scaling,
                                               double dynamic_newborn_vel_stddev);

    // Calculates velocity statistics from particles and classifies cells
    void calculateVelocityStatistics(double max_vel_for_scaling,
                                     bool   use_ego_comp,
                                     double ego_vx, double ego_vy);

    // Converts the grid map state to a ROS OccupancyGrid message
    void toOccupancyGridMsg(nav_msgs::OccupancyGrid& msg, const std::string& frame_id) const;

    // Converts the grid map state and velocities to a ROS MarkerArray message for visualization
    void toMarkerArrayMsg(visualization_msgs::MarkerArray& arr,
                          const std::string& frame_id,
                          bool show_velocity_arrows) const;

    // Getters for internal state
    ParticleFilter& getParticleFilter() { return *particle_filter_; }
    const std::vector<MeasurementCell>& getMeasurementGrid() const { return measurement_grid_; }
    const std::vector<GridCell>& getGrid() const { return grid_; } // Needed by updateWeights

    // Coordinate conversion utilities
    bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& wx, double& wy) const;
    int gridToIndex(int gx, int gy) const;
    void indexToGrid(int idx, int& gx, int& gy) const;
    bool isInside(int gx, int gy) const;

    // Public helper for particle filter (for L_Radar smoothing)
    bool getSmoothedRadarVrHint(int center_gx, int center_gy, double& smoothed_vr_hint) const;


private:
    // Grid map properties
    double grid_size_;
    double resolution_;
    int grid_width_;
    int grid_height_;
    double origin_x_;
    double origin_y_;

    // Grid data storage
    std::vector<GridCell> grid_;
    std::vector<MeasurementCell> measurement_grid_;

    // Particle filter instance
    std::unique_ptr<ParticleFilter> particle_filter_;

    // Random number generator
    std::mt19937 random_generator_;

    // Radar processing parameters
    int radar_buffer_size_;
    int min_radar_points_;
    int radar_hint_search_radius_;

    // Safety Net Parameters
    bool use_fsd_;
    int fsd_T_static_;
    int fsd_T_free_;
    bool use_mc_;

    // General flags
    bool use_radar_;

    // LiDAR processing parameters
    int    lidar_hit_point_;
    double lidar_noise_stddev_;

    // Velocity clustering
    double mode_cluster_velocity_thresh_sq_;
    
    // Dual Thresholds
    double particle_static_vel_thresh_;
    double radar_static_vel_thresh_;
};

#endif // DYNAMIC_GRID_MAP_H