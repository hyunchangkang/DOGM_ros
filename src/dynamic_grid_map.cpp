#include "dogm_ros/dynamic_grid_map.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <random>

// Constructor (동일)
DynamicGridMap::DynamicGridMap(double grid_size, double resolution, int num_particles,
                               double process_noise_pos, double process_noise_vel,
                               int radar_buffer_size, int min_radar_points,
                               int radar_hint_search_radius,
                               bool use_fsd, int fsd_T_static, int fsd_T_free,
                               bool use_mc,
                               bool use_radar,
                               int lidar_hit_point,
                               double lidar_noise_stddev,
                               double mode_cluster_velocity_thresh,
                               double particle_static_vel_thresh,
                               double radar_static_vel_thresh)
    : grid_size_(grid_size),
      resolution_(resolution),
      radar_buffer_size_(radar_buffer_size),
      min_radar_points_(min_radar_points),
      radar_hint_search_radius_(radar_hint_search_radius),
      use_fsd_(use_fsd),
      fsd_T_static_(fsd_T_static),
      fsd_T_free_(fsd_T_free),
      use_mc_(use_mc),
      use_radar_(use_radar),
      lidar_hit_point_(lidar_hit_point),
      lidar_noise_stddev_(lidar_noise_stddev),
      mode_cluster_velocity_thresh_sq_(mode_cluster_velocity_thresh * mode_cluster_velocity_thresh),
      particle_static_vel_thresh_(particle_static_vel_thresh),
      radar_static_vel_thresh_(radar_static_vel_thresh),
      random_generator_(std::mt19937(std::random_device()()))
{
    grid_width_  = static_cast<int>(std::round(grid_size_ / resolution_));
    grid_height_ = grid_width_;
    origin_x_ = 0.0;
    origin_y_ = -grid_size_ / 2.0;
    grid_.assign(grid_width_ * grid_height_, GridCell{});
    measurement_grid_.assign(grid_width_ * grid_height_, MeasurementCell{});
    particle_filter_ = std::make_unique<ParticleFilter>(
        num_particles, process_noise_pos, process_noise_vel
    );
}

// Grid conversion functions (동일)
bool DynamicGridMap::isInside(int gx, int gy) const { return (gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_); }
int DynamicGridMap::gridToIndex(int gx, int gy) const { return gy * grid_width_ + gx; }
void DynamicGridMap::indexToGrid(int idx, int& gx, int& gy) const { gy = idx / grid_width_; gx = idx % grid_width_; }
bool DynamicGridMap::worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    return isInside(gx, gy);
}
void DynamicGridMap::gridToWorld(int gx, int gy, double& wx, double& wy) const {
    wx = origin_x_ + (static_cast<double>(gx) + 0.5) * resolution_;
    wy = origin_y_ + (static_cast<double>(gy) + 0.5) * resolution_;
}

// generateMeasurementGrid (수정됨)
void DynamicGridMap::generateMeasurementGrid(const sensor_msgs::LaserScan::ConstPtr& scan, const pcl::PointCloud<mmWaveCloudType>::ConstPtr& radar_cloud) {
    // [초기화] m_occ_z도 0으로 초기화
    for (auto& m : measurement_grid_) { 
        m.m_free_z = 0.0; 
        m.m_occ_z = 0.0; 
        m.has_lidar_model = false; 
    }
    
    // Radar Processing (동일)
    for (auto& c : grid_) {
        for (auto& rp : c.radar_points_buffer) { rp.age++; }
        c.radar_points_buffer.erase(std::remove_if(c.radar_points_buffer.begin(), c.radar_points_buffer.end(),
                           [this](const RadarPoint& rp) { return rp.age > this->radar_buffer_size_; }), c.radar_points_buffer.end());

        if (c.radar_points_buffer.size() >= min_radar_points_) {
            double sum_weighted_vr = 0.0, sum_weighted_x = 0.0, sum_weighted_y = 0.0, sum_weights = 0.0;
            for (const auto& rp : c.radar_points_buffer) {
                double weight = 1.0 - (static_cast<double>(rp.age) / (radar_buffer_size_ + 1.0));
                sum_weighted_vr += rp.radial_velocity * weight;
                sum_weighted_x += rp.x * weight;
                sum_weighted_y += rp.y * weight;
                sum_weights += weight;
            }
            if (sum_weights > 1e-9) {
                c.radar_vr_hint = sum_weighted_vr / sum_weights;
                c.radar_theta_hint = std::atan2(sum_weighted_y / sum_weights, sum_weighted_x / sum_weights);
                c.radar_cos_theta = std::cos(c.radar_theta_hint);
                c.radar_sin_theta = std::sin(c.radar_theta_hint);
                c.has_reliable_radar = true;
            } else { c.has_reliable_radar = false; c.radar_vr_hint = 0.0; }
        } else { c.has_reliable_radar = false; c.radar_vr_hint = 0.0; }
    }

    if (use_radar_ && radar_cloud) {
        for (const auto& pt : radar_cloud->points) {
            int gx, gy;
            if (worldToGrid(pt.x, pt.y, gx, gy)) {
                int idx = gridToIndex(gx, gy);
                RadarPoint new_rp; new_rp.radial_velocity = pt.velocity; new_rp.x = pt.x; new_rp.y = pt.y; new_rp.age = 0;
                grid_[idx].radar_points_buffer.push_back(new_rp);
            }
        }
    }

    std::vector<std::vector<std::pair<double, double>>> cell_hits(grid_width_ * grid_height_);
    if (!scan) return;
    const double angle_min = static_cast<double>(scan->angle_min);
    const double angle_inc = static_cast<double>(scan->angle_increment);
    const double range_max = static_cast<double>(scan->range_max);
    
    // LiDAR Ray-casting (Free Space) - [ISM 적용]
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double r  = static_cast<double>(scan->ranges[i]);
        if (!std::isfinite(r) || r<0.01) {
            r = range_max;
        }
        const double th = angle_min + angle_inc * static_cast<double>(i);
        const double step  = resolution_ * 0.9;
        const double limit = std::min(r, range_max);
        
        // [수정] Free Space에 고정 확률(P_FREE) 할당
        const double P_FREE = 0.4; 

        for (double rr = 0.0; rr < limit; rr += step) {
            const double wx = rr * std::cos(th);
            const double wy = rr * std::sin(th);
            int gx, gy;
            if (!worldToGrid(wx, wy, gx, gy)) break;
            
            measurement_grid_[gridToIndex(gx, gy)].m_free_z = P_FREE;
        }
        
        if (r < range_max) {
            const double wx = r * std::cos(th);
            const double wy = r * std::sin(th);
            int gx, gy;
            if (worldToGrid(wx, wy, gx, gy)) cell_hits[gridToIndex(gx, gy)].push_back({wx, wy});
        }
    }

    const double lidar_variance_reg = lidar_noise_stddev_ * lidar_noise_stddev_;
    
    // [핵심 수정] Occupancy & Gaussian Model 통합 로직
    for (int idx = 0; idx < static_cast<int>(grid_.size()); ++idx) {
        auto& meas_cell = measurement_grid_[idx];
        const auto& hits = cell_hits[idx];
        
        // [통합] lidar_hit_point_ (예: 5) 미만이면 전부 노이즈로 간주하고 무시
        if (hits.size() < static_cast<size_t>(lidar_hit_point_)) {
            // 초기값이 0.0이므로 별도 대입 불필요. 
            // 1~4개 찍힌 애들은 여기서 걸러짐 -> 파티클 생성 안 됨.
            continue; 
        }
        
        // [기준 충족] 점유 확률 할당 (ISM)
        meas_cell.m_occ_z = 0.7; // 70% 확신 (필요시 hits.size()에 비례하여 조정 가능)
        meas_cell.m_free_z = 0.0; // 점유되었으므로 Free 확률 제거

        // [기준 충족] 가우시안 모델 생성 (기존 로직)
        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& p : hits) { sum_x += p.first; sum_y += p.second; }
        meas_cell.mean_x = sum_x / hits.size(); meas_cell.mean_y = sum_y / hits.size();
        double sum_xx = 0.0, sum_xy = 0.0, sum_yy = 0.0;
        for (const auto& p : hits) {
            double dx = p.first - meas_cell.mean_x; double dy = p.second - meas_cell.mean_y;
            sum_xx += dx * dx; sum_xy += dx * dy; sum_yy += dy * dy;
        }
        double n_minus_1 = (hits.size() > 1) ? (hits.size() - 1) : 1.0;
        double cov_xx = (sum_xx / n_minus_1) + lidar_variance_reg;
        double cov_xy = (sum_xy / n_minus_1);
        double cov_yy = (sum_yy / n_minus_1) + lidar_variance_reg;
        double det = cov_xx * cov_yy - cov_xy * cov_xy;
        if (std::abs(det) < 1e-9) continue;
        double inv_det = 1.0 / det;
        meas_cell.inv_cov_xx =  cov_yy * inv_det; meas_cell.inv_cov_xy = -cov_xy * inv_det; meas_cell.inv_cov_yy =  cov_xx * inv_det;
        meas_cell.has_lidar_model = true;
    }
}

// updateOccupancy (수정됨)
void DynamicGridMap::updateOccupancy(double birth_prob) {
    for (auto& c : grid_) {
        c.m_occ  = std::max(0.0, c.m_occ  * 0.55);
        c.m_free = std::max(0.0, c.m_free * 0.55);
        c.rho_b  = std::max(0.0, c.rho_b  * 0.55);
        c.rho_p  = std::max(0.0, c.rho_p  * 0.55);
    }
    for (int idx = 0; idx < static_cast<int>(grid_.size()); ++idx) {
        auto& cell = grid_[idx];
        const auto& meas = measurement_grid_[idx];
        
        // [수정] ISM 확률 사용
        double m_occ_z_proxy = meas.m_occ_z; 

        double m_occ_pred = std::min(1.0, std::max(0.0, cell.m_occ));
        double m_free_pred= std::min(1.0, std::max(0.0, cell.m_free));
        double m_unk_pred = std::max(0.0, 1.0 - m_occ_pred - m_free_pred);
        
        double K = m_occ_pred * meas.m_free_z + m_free_pred * m_occ_z_proxy;
        double norm = 1.0 / std::max(1e-9, (1.0 - K));
        
        double m_occ_upd = norm * (m_occ_pred * (1.0 - meas.m_free_z) + m_unk_pred * m_occ_z_proxy);
        double m_free_upd= norm * (m_free_pred * (1.0 - m_occ_z_proxy) + m_unk_pred * meas.m_free_z);
        
        m_occ_upd = std::min(1.0, std::max(0.0, m_occ_upd));
        m_free_upd= std::min(1.0, std::max(0.0, m_free_upd));
        
        double term = m_occ_pred + birth_prob * m_unk_pred;
        cell.rho_b = (term > 1e-9) ? (m_occ_upd * birth_prob * m_unk_pred) / term : 0.0;
        cell.rho_p = std::max(0.0, m_occ_upd - cell.rho_b);
        cell.m_occ  = m_occ_upd; cell.m_free = m_free_upd;
    }
}

// 나머지 함수들은 기존과 동일 (getSmoothedRadarVrHint, generateNewParticles, calculateVelocityStatistics, etc.)
// ... (생략 없이 이전 turn과 동일하게 유지) ...
// 편의상 생략했습니다. 이 파일의 나머지 부분은 이전 답변의 코드를 그대로 사용하시면 됩니다.

bool DynamicGridMap::getSmoothedRadarVrHint(int center_gx, int center_gy, double& smoothed_vr_hint) const {
    double sum_weighted_vr = 0.0, sum_weights = 0.0;
    for (int dy = -radar_hint_search_radius_; dy <= radar_hint_search_radius_; ++dy) {
        for (int dx = -radar_hint_search_radius_; dx <= radar_hint_search_radius_; ++dx) {
            int gx = center_gx + dx; int gy = center_gy + dy;
            if (isInside(gx, gy)) {
                const auto& cell = grid_[gridToIndex(gx, gy)];
                if (cell.has_reliable_radar) {
                    int layer = std::max(std::abs(dx), std::abs(dy));
                    double weight = (layer == 0) ? 1.0 : ( (layer <= radar_hint_search_radius_) ? 0.5 : 0.0 );
                    sum_weighted_vr += cell.radar_vr_hint * weight;
                    sum_weights += weight;
                }
            }
        }
    }
    if (sum_weights > 1e-9) { smoothed_vr_hint = sum_weighted_vr / sum_weights; return true; }
    return false;
}

std::vector<Particle> DynamicGridMap::generateNewParticles(double newborn_vel_stddev,
                                               double min_dynamic_birth_ratio,
                                               double max_dynamic_birth_ratio,
                                               double max_radar_speed_for_scaling,
                                               double dynamic_newborn_vel_stddev,
                                               const EgoCalibration& ego_calib)
{
    std::vector<Particle> new_particles;
    
    double static_vx, static_vy;
    ego_calib.getStaticParticleVelocity(static_vx, static_vy);
    std::normal_distribution<double> static_vel_dist_x(static_vx, newborn_vel_stddev);
    std::normal_distribution<double> static_vel_dist_y(static_vy, newborn_vel_stddev);
    
    std::normal_distribution<double> dynamic_noise_dist(0.0, newborn_vel_stddev);
    std::normal_distribution<double> dynamic_fallback_dist(0.0, dynamic_newborn_vel_stddev);

    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            int idx = gridToIndex(x, y);
            const auto& cell = grid_[idx];

            if (cell.rho_b > 0.5 && cell.m_occ > 0.6) {
                int num_to_birth = static_cast<int>(std::ceil(cell.rho_b * 4.0));
                
                double smoothed_vr = 0.0;
                bool has_radar = use_radar_ && getSmoothedRadarVrHint(x, y, smoothed_vr);
                double current_dynamic_ratio = min_dynamic_birth_ratio;

                if (has_radar) {
                    double smoothed_speed = std::abs(smoothed_vr);
                    double scale = std::min(1.0, smoothed_speed / std::max(1e-6, max_radar_speed_for_scaling));
                    current_dynamic_ratio = min_dynamic_birth_ratio + (max_dynamic_birth_ratio - min_dynamic_birth_ratio) * scale;
                } else if (!use_radar_) {
                    current_dynamic_ratio = max_dynamic_birth_ratio;
                }

                int num_dynamic = static_cast<int>(num_to_birth * current_dynamic_ratio);
                int num_static = num_to_birth - num_dynamic;

                for (int i = 0; i < num_static; ++i) {
                    Particle p;
                    gridToWorld(x, y, p.x, p.y);
                    p.vx = static_vel_dist_x(random_generator_);
                    p.vy = static_vel_dist_y(random_generator_);
                    p.weight = cell.rho_b / static_cast<double>(num_to_birth);
                    p.grid_cell_idx = idx;
                    p.age = 0;
                    new_particles.push_back(p);
                }
                
                double cell_mean_speed = std::sqrt(cell.mean_vx * cell.mean_vx + cell.mean_vy * cell.mean_vy);
                
                for (int i = 0; i < num_dynamic; ++i) {
                    Particle p;
                    gridToWorld(x, y, p.x, p.y);
                    
                    if (cell.is_dynamic && cell_mean_speed > 0.1) {
                         p.vx = cell.mean_vx + dynamic_noise_dist(random_generator_);
                         p.vy = cell.mean_vy + dynamic_noise_dist(random_generator_);
                    }
                    else {
                        p.vx = dynamic_fallback_dist(random_generator_) + static_vx; 
                        p.vy = dynamic_fallback_dist(random_generator_) + static_vy; 
                    }
                    
                    p.weight = cell.rho_b / static_cast<double>(num_to_birth);
                    p.grid_cell_idx = idx;
                    p.age = 0;
                    new_particles.push_back(p);
                }
            }
        }
    }
    return new_particles;
}

void DynamicGridMap::calculateVelocityStatistics(double max_vel_for_scaling,
                                                 bool   use_ego_comp,
                                                 const EgoCalibration& ego_calib)
{
    for (auto& cell : grid_) {
        cell.is_dynamic = false; 
        cell.dyn_streak = 0; cell.stat_streak = 0;
        cell.dynamic_score *= 0.85; 
        if (cell.dynamic_score < 0.01) cell.dynamic_score = 0.0;
        if (cell.m_free > 0.8) cell.free_streak = std::min<std::uint8_t>(255, cell.free_streak + 1);
        else if (cell.m_occ > 0.6) cell.free_streak = 0;
    }

    auto& parts = particle_filter_->getParticles();
    const int need_on_frames = 2; const int need_off_frames = 4;

    auto flush_cell = [&](int cell_idx, int start, int end) {
        if (cell_idx < 0 || cell_idx >= static_cast<int>(grid_.size())) return;
        auto& c = grid_[cell_idx];

        if (end - start <= 2) {
            c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            if (c.stat_streak >= need_off_frames) c.is_dynamic = false;
            c.dyn_streak = 0; return;
        }

        double max_weight = -1.0; int winner_idx = start;
        for (int j = start; j < end; ++j) {
            if (parts[j].weight > max_weight) { max_weight = parts[j].weight; winner_idx = j; }
        }
        const double winner_vx = parts[winner_idx].vx; const double winner_vy = parts[winner_idx].vy;
        double mode_w_sum = 0.0, mode_vx_sum = 0.0, mode_vy_sum = 0.0;
        const double VELOCITY_THRESHOLD_SQ = mode_cluster_velocity_thresh_sq_; 

        for (int j = start; j < end; ++j) {
            const double dvx = parts[j].vx - winner_vx; const double dvy = parts[j].vy - winner_vy;
            if ((dvx * dvx + dvy * dvy) < VELOCITY_THRESHOLD_SQ) {
                mode_w_sum += parts[j].weight;
                mode_vx_sum += parts[j].weight * parts[j].vx;
                mode_vy_sum += parts[j].weight * parts[j].vy;
            }
        }

        if (mode_w_sum <= 1e-9) {
            c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            if (c.stat_streak >= need_off_frames) c.is_dynamic = false;
            c.dyn_streak = 0; return;
        }

        c.mean_vx = mode_vx_sum / mode_w_sum;
        c.mean_vy = mode_vy_sum / mode_w_sum;

        double speed_p = std::sqrt(c.mean_vx * c.mean_vx + c.mean_vy * c.mean_vy);
        if (use_ego_comp) { 
            speed_p = ego_calib.getAbsoluteSpeed(c.mean_vx, c.mean_vy);
        }

        double m_unk = std::max(0.0, 1.0 - c.m_occ - c.m_free);
        const bool is_occupied = (c.m_occ >= c.m_free && c.m_occ >= m_unk);
        const bool has_speed_from_particles = (speed_p > particle_static_vel_thresh_);

        double smoothed_vr_hint = 0.0;
        int gx, gy; indexToGrid(cell_idx, gx, gy);
        bool has_reliable_smoothed_radar = use_radar_ && getSmoothedRadarVrHint(gx, gy, smoothed_vr_hint);
        
        double speed_r = 0.0;
        if (has_reliable_smoothed_radar) {
            double cell_wx, cell_wy;
            gridToWorld(gx, gy, cell_wx, cell_wy);
            double azimuth = std::atan2(cell_wy, cell_wx);
            double abs_radar_vel = ego_calib.getAbsoluteRadialVelocity(smoothed_vr_hint, azimuth);
            speed_r = std::abs(abs_radar_vel);
        }
        
        const bool has_speed_from_radar = has_reliable_smoothed_radar && (speed_r > radar_static_vel_thresh_);
        const bool is_reliably_static_by_radar = has_reliable_smoothed_radar && (speed_r < particle_static_vel_thresh_);
        
        bool dyn_candidate = false;
        if (use_radar_) {
            dyn_candidate = is_occupied && !is_reliably_static_by_radar && (has_speed_from_particles || has_speed_from_radar);
            if (use_mc_ && is_occupied && !dyn_candidate && has_speed_from_particles && is_reliably_static_by_radar) dyn_candidate = true;
            bool is_currently_static = is_occupied && !dyn_candidate;
            if (use_fsd_ && is_currently_static && c.stat_streak >= fsd_T_static_ && c.free_streak >= fsd_T_free_) { 
                dyn_candidate = true; c.free_streak = 0; 
            }
        } else {
            dyn_candidate = is_occupied && has_speed_from_particles;
        }

        if (dyn_candidate) {
            int streak_increase = (has_speed_from_radar) ? 2 : 1;
            c.dyn_streak  = std::min<std::uint8_t>(255, c.dyn_streak + streak_increase);
            c.stat_streak = 0;
        } else {
            c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            c.dyn_streak  = 0;
        }

        if (!c.is_dynamic && c.dyn_streak >= need_on_frames) c.is_dynamic = true;
        if ( c.is_dynamic && c.stat_streak >= need_off_frames) c.is_dynamic = false;

        const double target = c.is_dynamic ? std::min(1.0, speed_p / std::max(1e-6, max_vel_for_scaling)) : 0.0;
        const double alpha  = 0.6;
        c.dynamic_score = alpha * target + (1.0 - alpha) * c.dynamic_score;
    }; 

    int current_idx = -1; int first_i = 0;
    for (int i = 0; i <= static_cast<int>(parts.size()); ++i) {
        bool last = (i == static_cast<int>(parts.size()));
        int idx   = last ? -1 : parts[i].grid_cell_idx;
        if (last || idx != current_idx) {
            flush_cell(current_idx, first_i, i);
            if (last) break;
            current_idx = idx;
            first_i = i;
        }
    }
}

void DynamicGridMap::toOccupancyGridMsg(nav_msgs::OccupancyGrid& msg, const std::string& frame_id) const
{
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.info.resolution = resolution_;
    msg.info.width  = grid_width_;
    msg.info.height = grid_height_;
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(grid_width_ * grid_height_, -1);
    for (size_t i = 0; i < grid_.size(); ++i) {
        const auto& c = grid_[i];
        double m_unk = std::max(0.0, 1.0 - c.m_occ - c.m_free);
        
        if (c.m_occ >= c.m_free && c.m_occ >= m_unk) {
            msg.data[i] = static_cast<int8_t>(std::min(100.0, c.m_occ * 100.0));
        } else if (c.m_free >= m_unk) {
            msg.data[i] = 0;
        } else {
            msg.data[i] = -1;
        }
    }
}

void DynamicGridMap::toMarkerArrayMsg(visualization_msgs::MarkerArray& arr,
                                      const std::string& frame_id,
                                      bool show_velocity_arrows,
                                      const EgoCalibration& ego_calib) const
{
    arr.markers.clear();
    visualization_msgs::Marker cubes;
    cubes.header.stamp = ros::Time::now(); cubes.header.frame_id = frame_id; cubes.ns = "dogm_cells"; cubes.id = 0;
    cubes.type = visualization_msgs::Marker::CUBE_LIST; cubes.action = visualization_msgs::Marker::ADD;
    cubes.pose.orientation.w = 1.0; cubes.scale.x = resolution_; cubes.scale.y = resolution_; cubes.scale.z = 0.02;
    cubes.lifetime = ros::Duration(1);
    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            const auto& c = grid_[gridToIndex(x, y)];
            geometry_msgs::Point p; p.x = origin_x_ + (x + 0.5) * resolution_; p.y = origin_y_ + (y + 0.5) * resolution_; p.z = -0.02;
            std_msgs::ColorRGBA col; col.a = 0.2;
            double m_unk = std::max(0.0, 1.0 - c.m_occ - c.m_free);

            if (c.m_occ >= c.m_free && c.m_occ >= m_unk) {
                if (c.is_dynamic) { col.r = 1.0f; col.g = 0.0f; col.b = 0.0f; } 
                else              { col.r = 0.0f; col.g = 0.0f; col.b = 1.0f; } 
                col.a = 0.2 + 0.4 * std::min(1.0, c.m_occ); 
            } 
            else if (c.m_free >= m_unk) { 
                col.r = col.g = col.b = 1.0f; col.a = 0.5f; 
            } 
            else { 
                col.r = col.g = col.b = 0.5f; col.a = 0.5f; 
            }
            cubes.points.push_back(p); cubes.colors.push_back(col);
        }
    }
    arr.markers.push_back(cubes);

    if (show_velocity_arrows) {
        visualization_msgs::Marker arrows;
        arrows.header.stamp = ros::Time::now(); arrows.header.frame_id = frame_id; arrows.ns = "dogm_vel"; arrows.id = 1;
        arrows.type = visualization_msgs::Marker::ARROW; arrows.action = visualization_msgs::Marker::ADD;
        arrows.scale.x = 0.02; arrows.scale.y = 0.04; arrows.scale.z = 0.04;
        arrows.color.r = 1.0; arrows.color.g = 0.0; arrows.color.b = 0.0; arrows.color.a = 1.0;
        arrows.lifetime = ros::Duration(0.2);
        int arrow_id = 10;
        for (int y = 0; y < grid_height_; ++y) {
            for (int x = 0; x < grid_width_; ++x) {
                const auto& c = grid_[gridToIndex(x, y)];
                if (!c.is_dynamic || c.m_occ < 0.6) continue;
                geometry_msgs::Point p0, p1;
                p0.x = origin_x_ + (x + 0.5) * resolution_;
                p0.y = origin_y_ + (y + 0.5) * resolution_;
                p0.z = 0.00;
                
                double abs_vx, abs_vy;
                ego_calib.getAbsoluteVelocity(c.mean_vx, c.mean_vy, abs_vx, abs_vy);

                const double scale = 0.4;
                p1.x = p0.x + scale * abs_vx; 
                p1.y = p0.y + scale * abs_vy;
                p1.z = 0.00;
                visualization_msgs::Marker a = arrows;
                a.id = arrow_id++; a.points.clear(); a.points.push_back(p0); a.points.push_back(p1);
                arr.markers.push_back(a);
            }
        }
    }
}

void DynamicGridMap::shiftGrid(double dx, double dy) {
    int shift_x = static_cast<int>(std::round(dx / resolution_));
    int shift_y = static_cast<int>(std::round(dy / resolution_));

    if (shift_x == 0 && shift_y == 0) return;

    std::vector<GridCell> new_grid(grid_.size());
    std::vector<MeasurementCell> new_meas(measurement_grid_.size());

    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            int old_x = x + shift_x;
            int old_y = y + shift_y;

            if (isInside(old_x, old_y)) {
                int new_idx = gridToIndex(x, y);
                int old_idx = gridToIndex(old_x, old_y);
                new_grid[new_idx] = grid_[old_idx];
                new_meas[new_idx] = measurement_grid_[old_idx];
                for(auto& p : new_grid[new_idx].radar_points_buffer) {
                    p.x -= dx; p.y -= dy;
                }
            }
        }
    }
    grid_ = new_grid;
    measurement_grid_ = new_meas;
}