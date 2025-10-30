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
    // [MODIFIED] Added lidar_point_counts parameter
    DynamicGridMap(double grid_size, double resolution, int num_particles,
                   double process_noise_pos, double process_noise_vel,
                   int radar_buffer_size, int min_radar_points,
                   int radar_hint_search_radius,
                   bool use_fsd, int fsd_T_static, int fsd_T_free,
                   bool use_mc,
                   bool use_radar,
                   int lidar_point_counts); // Added lidar_point_counts
    ~DynamicGridMap() = default; // Default destructor

    // Generates the measurement grid based on sensor inputs
    void generateMeasurementGrid(const sensor_msgs::LaserScan::ConstPtr& scan,
                                 const pcl::PointCloud<mmWaveCloudType>::ConstPtr& radar_cloud);

    // Updates cell occupancy probabilities using Dempster-Shafer
    void updateOccupancy(double birth_prob);

    // Generates new particles for cells with high birth probability
    std::vector<Particle> generateNewParticles(double newborn_vel_stddev,
                                               double min_dynamic_birth_ratio,
                                               double max_dynamic_birth_ratio,
                                               double max_radar_speed_for_scaling,
                                               double dynamic_newborn_vel_stddev);

    // Calculates velocity statistics from particles and classifies cells
    void calculateVelocityStatistics(double static_vel_thresh,
                                     double max_vel_for_scaling,
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
    bool isInside(int gx, int gy) const;

private:
    // Calculates a spatially smoothed 1D radar velocity hint
    bool getSmoothedRadarVrHint(int center_gx, int center_gy, double& smoothed_vr_hint) const;

    // Grid map properties
    double grid_size_;        // Size of one side of the square grid (meters)
    double resolution_;       // Size of one cell (meters/cell)
    int grid_width_;          // Number of cells along width
    int grid_height_;         // Number of cells along height
    double origin_x_;         // X coordinate of the grid origin (bottom-left corner)
    double origin_y_;         // Y coordinate of the grid origin (bottom-left corner)

    // Grid data storage
    std::vector<GridCell> grid_;             // Stores the state of each cell (DS masses, velocity, etc.)
    std::vector<MeasurementCell> measurement_grid_; // Stores the latest sensor measurements (m_occ_z, m_free_z)

    // Particle filter instance
    std::unique_ptr<ParticleFilter> particle_filter_;

    // Random number generator
    std::mt19937 random_generator_;

    // Radar processing parameters
    int radar_buffer_size_;      // How many frames to keep radar points in buffer
    int min_radar_points_;       // Minimum radar points required for a reliable hint
    int radar_hint_search_radius_; // Neighborhood size for smoothing radar hint

    // Safety Net Parameters
    bool use_fsd_;            // Flag to enable/disable False Static Detection
    int fsd_T_static_;        // Threshold for static frames in FSD
    int fsd_T_free_;          // Threshold for preceding free frames in FSD
    bool use_mc_;             // Flag to enable/disable Measurement Correction

    // General flags
    bool use_radar_;          // Flag indicating if radar fusion is active

    // [NEW] LiDAR processing parameter
    int lidar_point_counts_; // Minimum LiDAR hits required to consider a cell occupied
};

#endif // DYNAMIC_GRID_MAP_H