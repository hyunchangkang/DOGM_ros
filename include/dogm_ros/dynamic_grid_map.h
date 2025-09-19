#ifndef DYNAMIC_GRID_MAP_H
#define DYNAMIC_GRID_MAP_H

#include <vector>
#include <memory>
#include <string>
#include <random> // [추가] std::mt19937 사용을 위해 추가
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

#include "structures.h"
#include "particle_filter.h"

class DynamicGridMap {
public:
    DynamicGridMap(double grid_size, double resolution, int num_particles,
                   double process_noise_pos, double process_noise_vel);
    ~DynamicGridMap() = default;

    void generateMeasurementGrid(const sensor_msgs::LaserScan::ConstPtr& scan);
    void updateOccupancy(double birth_prob);

    std::vector<Particle> generateNewParticles(double newborn_vel_stddev,
                                               double dynamic_birth_ratio,
                                               double dynamic_newborn_vel_stddev);

    // [수정] cpp 파일의 구현에 맞춰 사용하지 않는 파라미터 이름을 주석 처리하여 명확화
    void calculateVelocityStatistics(double static_vel_thresh,
                                     double /*mahalanobis_thresh*/,
                                     double /*max_variance_thresh*/,
                                     double max_vel_for_scaling,
                                     bool   use_ego_comp,
                                     double ego_vx, double ego_vy);

    void toOccupancyGridMsg(nav_msgs::OccupancyGrid& msg, const std::string& frame_id) const;
    
    // [수정] 파라미터 이름을 cpp 파일과 일치 (msg -> arr, show_arrows -> show_velocity_arrows)
    void toMarkerArrayMsg(visualization_msgs::MarkerArray& arr, 
                          const std::string& frame_id, 
                          bool show_velocity_arrows) const;

    ParticleFilter& getParticleFilter() { return *particle_filter_; }

    // [수정] 파라미터 이름을 cpp 파일과 일치시켜 가독성 향상
    bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& wx, double& wy) const;
    int gridToIndex(int gx, int gy) const;
    bool isInside(int gx, int gy) const;

    const std::vector<MeasurementCell>& getMeasurementGrid() const { return measurement_grid_; }


private:
    double grid_size_;
    double resolution_;
    int grid_width_;
    int grid_height_;
    double origin_x_;
    double origin_y_;
    std::vector<GridCell> grid_;
    std::vector<MeasurementCell> measurement_grid_;
    std::unique_ptr<ParticleFilter> particle_filter_;
    std::mt19937 random_generator_; // cpp 파일에 있으므로 헤더에도 선언 필요
};

#endif // DYNAMIC_GRID_MAP_H