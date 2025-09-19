#ifndef DYNAMIC_GRID_MAP_H
#define DYNAMIC_GRID_MAP_H

#include <vector>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

#include "structures.h"
#include "particle_filter.h"

class DynamicGridMap {
public:
    DynamicGridMap(double grid_size, double resolution);
    ~DynamicGridMap() = default;

    void generateMeasurementGrid(const sensor_msgs::LaserScan::ConstPtr& scan);
    void updateOccupancy(double birth_prob);

    // [수정] 이중 가설 생성을 위한 파라미터를 인자로 받도록 변경
    std::vector<Particle> generateNewParticles(double newborn_vel_stddev,
                                               double dynamic_birth_ratio,
                                               double dynamic_newborn_vel_stddev);

    void calculateVelocityStatistics(double static_vel_thresh, double mahalanobis_thresh, double max_variance_thresh, double max_vel_for_scaling, bool use_ego_comp, double ego_vx, double ego_vy);

    void toOccupancyGridMsg(nav_msgs::OccupancyGrid& msg, const std::string& frame_id) const;
    void toMarkerArrayMsg(visualization_msgs::MarkerArray& msg, const std::string& frame_id, bool show_arrows) const;

    ParticleFilter& getParticleFilter() { return *particle_filter_; }

    bool worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const;
    void gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y) const;

    int gridToIndex(int grid_x, int grid_y) const;
    bool isInside(int grid_x, int grid_y) const;

    // [추가] ParticleFilter에서 셀 정보에 접근하기 위한 Getter 함수들
    int getGridCellCount() const { return grid_width_ * grid_height_; }
    const MeasurementCell& getMeasurementCell(int index) const { return measurement_grid_[index]; }


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
    std::mt19937 random_generator_;
};

#endif // DYNAMIC_GRID_MAP_H