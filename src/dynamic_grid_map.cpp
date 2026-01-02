#include "dogm_ros/dynamic_grid_map.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <random>
#include <queue> 
#include <map>

#define ONE_ARROW       1
#define BOUNDING_BOX    1  // 1: 바운딩 박스 표시, 0: 바운딩 박스 숨김

DynamicGridMap::DynamicGridMap(double grid_size, double resolution, int num_particles,
                               double process_noise_pos, double process_noise_vel,
                               int radar_buffer_size, int min_radar_points,
                               int radar_hint_search_radius,
                               bool use_fsd, int fsd_T_static, int fsd_T_free,
                               bool use_mc,
                               bool use_radar,
                               int lidar_hit_point,
                               double lidar_noise_stddev,
                               // [Params]
                               double particle_vector_vel_thresh,
                               double particle_vector_ang_thresh,
                               double particle_static_vel_thresh,
                               double radar_static_vel_thresh,
                               bool cluster_mode)
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
      // [Thresholds]
      particle_vector_vel_thresh_(particle_vector_vel_thresh),
      particle_vector_ang_thresh_(particle_vector_ang_thresh),
      particle_static_vel_thresh_(particle_static_vel_thresh),
      radar_static_vel_thresh_(radar_static_vel_thresh),
      cluster_mode_(cluster_mode),
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

// ... (Helper functions - 기존과 동일) ...
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

void DynamicGridMap::generateMeasurementGrid(const sensor_msgs::LaserScan::ConstPtr& scan, 
                                             const std::vector<RadarDataPacket>& radar_packets) {
    for (auto& m : measurement_grid_) { m.m_free_z = 0.0; m.m_occ_z = 0.0; m.has_lidar_model = false; }
    for (auto& c : grid_) { c.radar_hints.clear(); }

    for (auto& c : grid_) {
        for (auto& rp : c.radar_buffer_1) { rp.age++; }
        c.radar_buffer_1.erase(std::remove_if(c.radar_buffer_1.begin(), c.radar_buffer_1.end(),
                           [this](const RadarPoint& rp) { return rp.age > this->radar_buffer_size_; }), c.radar_buffer_1.end());
        for (auto& rp : c.radar_buffer_2) { rp.age++; }
        c.radar_buffer_2.erase(std::remove_if(c.radar_buffer_2.begin(), c.radar_buffer_2.end(),
                           [this](const RadarPoint& rp) { return rp.age > this->radar_buffer_size_; }), c.radar_buffer_2.end());
    }

    if (use_radar_) {
        for (size_t i = 0; i < radar_packets.size(); ++i) {
            const auto& packet = radar_packets[i];
            if (!packet.cloud) continue;
            for (const auto& pt : packet.cloud->points) {
                int gx, gy;
                if (worldToGrid(pt.x, pt.y, gx, gy)) {
                    int idx = gridToIndex(gx, gy);
                    RadarPoint new_rp; 
                    new_rp.radial_velocity = pt.velocity; 
                    new_rp.x = pt.x; new_rp.y = pt.y; new_rp.age = 0;
                    if (i == 0) grid_[idx].radar_buffer_1.push_back(new_rp);
                    else if (i == 1) grid_[idx].radar_buffer_2.push_back(new_rp);
                }
            }
        }
    }

    auto processBuffer = [&](const std::vector<RadarPoint>& buffer, double sx, double sy, double syaw, int min_pts) -> RadarHint 
    {
        RadarHint hint;
        if (buffer.size() < static_cast<size_t>(min_pts)) return hint;

        double sum_vr = 0.0, sum_x = 0.0, sum_y = 0.0;
        for (const auto& rp : buffer) { 
            sum_vr += rp.radial_velocity; 
            sum_x += rp.x; 
            sum_y += rp.y; 
        }

        // 1. 관측 데이터들의 평균(무게중심) 계산
        hint.vr = sum_vr / buffer.size(); 
        double mean_x = sum_x / buffer.size(); 
        double mean_y = sum_y / buffer.size();
        
        hint.sensor_x = sx; 
        hint.sensor_y = sy;
        hint.sensor_yaw = syaw;
        hint.valid = true;

        // 2. [수정] 센서 방향 벡터 사전 계산 (센서 정면 = 로컬 X축)
        // 센서가 바라보는 방향의 단위 벡터
        hint.sensor_cos_yaw = std::cos(syaw);
        hint.sensor_sin_yaw = std::sin(syaw);

        return hint;
    };

    for (int idx = 0; idx < grid_.size(); ++idx) {
        auto& c = grid_[idx];
        if (radar_packets.size() > 0) {
            RadarHint h1 = processBuffer(c.radar_buffer_1, radar_packets[0].sensor_x, radar_packets[0].sensor_y, 
                                         radar_packets[0].sensor_yaw, min_radar_points_);
            if (h1.valid) c.radar_hints.push_back(h1);
        }
        if (radar_packets.size() > 1) {
            RadarHint h2 = processBuffer(c.radar_buffer_2, radar_packets[1].sensor_x, radar_packets[1].sensor_y,
                                         radar_packets[1].sensor_yaw, min_radar_points_);
            if (h2.valid) c.radar_hints.push_back(h2);
        }
    }
    
    // Lidar Processing (Simplified for brevity as it's unchanged)
    std::vector<std::vector<std::pair<double, double>>> cell_hits(grid_width_ * grid_height_);
    if (!scan) return;
    const double angle_min = static_cast<double>(scan->angle_min);
    const double angle_inc = static_cast<double>(scan->angle_increment);
    const double range_max = static_cast<double>(scan->range_max);
    
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double r  = static_cast<double>(scan->ranges[i]);
        if (!std::isfinite(r) || r<0.01) { r = range_max; }
        const double th = angle_min + angle_inc * static_cast<double>(i);
        const double step  = resolution_ * 0.9;
        const double limit = std::min(r, range_max);
        const double P_FREE = 0.4; 

        for (double rr = 0.0; rr < limit; rr += step) {
            const double wx = rr * std::cos(th); const double wy = rr * std::sin(th);
            int gx, gy;
            if (!worldToGrid(wx, wy, gx, gy)) break;
            measurement_grid_[gridToIndex(gx, gy)].m_free_z = P_FREE;
        }
        if (r < range_max) {
            const double wx = r * std::cos(th); const double wy = r * std::sin(th);
            int gx, gy;
            if (worldToGrid(wx, wy, gx, gy)) cell_hits[gridToIndex(gx, gy)].push_back({wx, wy});
        }
    }
    
    const double lidar_variance_reg = lidar_noise_stddev_ * lidar_noise_stddev_;
    for (int idx = 0; idx < static_cast<int>(grid_.size()); ++idx) {
        auto& meas_cell = measurement_grid_[idx];
        const auto& hits = cell_hits[idx];
        if (hits.size() < static_cast<size_t>(lidar_hit_point_)) continue; 
        meas_cell.m_occ_z = 0.7; meas_cell.m_free_z = 0.0; 
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
        meas_cell.inv_cov_xx = cov_yy * inv_det; meas_cell.inv_cov_xy = -cov_xy * inv_det; meas_cell.inv_cov_yy = cov_xx * inv_det;
        meas_cell.has_lidar_model = true;
    }
}

void DynamicGridMap::updateOccupancy(double birth_prob) {
    // ... (Occupancy Update logic unchanged) ...
    for (auto& c : grid_) {
        c.m_occ  = std::max(0.0, c.m_occ  * 0.55);
        c.m_free = std::max(0.0, c.m_free * 0.55);
        c.rho_b  = std::max(0.0, c.rho_b  * 0.55);
        c.rho_p  = std::max(0.0, c.rho_p  * 0.55);
    }
    for (int idx = 0; idx < static_cast<int>(grid_.size()); ++idx) {
        auto& cell = grid_[idx];
        const auto& meas = measurement_grid_[idx];
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

void DynamicGridMap::calculateVelocityStatistics(double max_vel_for_scaling,
                                                 bool   use_ego_comp,
                                                 const EgoCalibration& ego_calib)
{
    // 1. Initialization
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

        // --- [Cell-Level Winner Mode] ---
        // 1. Find Winner in Cell
        double max_weight = -1.0;
        int winner_idx = start;
        for (int j = start; j < end; ++j) {
            if (parts[j].weight > max_weight) {
                max_weight = parts[j].weight;
                winner_idx = j;
            }
        }
        
        // 2. Sector Gating (Vel 0.1, Ang 60.0)
        double mode_vx_sum = 0.0;
        double mode_vy_sum = 0.0;
        double mode_w_sum = 0.0;

        for (int j = start; j < end; ++j) {
            if (particle_filter_->checkSectorMatch(parts[winner_idx], parts[j], 
                                                   particle_vector_vel_thresh_, 
                                                   particle_vector_ang_thresh_)) 
            {
                mode_vx_sum += parts[j].vx * parts[j].weight;
                mode_vy_sum += parts[j].vy * parts[j].weight;
                mode_w_sum  += parts[j].weight;
            }
        }

        if (mode_w_sum <= 1e-9) {
            c.stat_streak = std::min<std::uint8_t>(255, c.stat_streak + 1);
            if (c.stat_streak >= need_off_frames) c.is_dynamic = false;
            c.dyn_streak = 0; return;
        }

        c.mean_vx = mode_vx_sum / mode_w_sum;
        c.mean_vy = mode_vy_sum / mode_w_sum;
        // --- [End Cell-Level] ---

        double speed_p = std::sqrt(c.mean_vx * c.mean_vx + c.mean_vy * c.mean_vy);
        if (use_ego_comp) { 
            speed_p = ego_calib.getAbsoluteSpeed(c.mean_vx, c.mean_vy);
        }

        double m_unk = std::max(0.0, 1.0 - c.m_occ - c.m_free);
        const bool is_occupied = (c.m_occ >= c.m_free && c.m_occ >= m_unk);
        const bool has_speed_from_particles = (speed_p > particle_static_vel_thresh_);

        double max_comp_speed = 0.0;
        int gx, gy; indexToGrid(cell_idx, gx, gy);

        if (use_radar_) {
             for (int dy = -radar_hint_search_radius_; dy <= radar_hint_search_radius_; ++dy) {
                for (int dx = -radar_hint_search_radius_; dx <= radar_hint_search_radius_; ++dx) {
                    int nx = gx + dx; int ny = gy + dy;
                    if (isInside(nx, ny)) {
                        const auto& neighbor = grid_[gridToIndex(nx, ny)];
                        double cell_wx, cell_wy;
                        gridToWorld(nx, ny, cell_wx, cell_wy);

                        for (const auto& hint : neighbor.radar_hints) {
                            double azimuth = std::atan2(cell_wy - hint.sensor_y, cell_wx - hint.sensor_x);
                            double abs_vr = std::abs(ego_calib.getAbsoluteRadialVelocity(hint.vr, azimuth));
                            if (abs_vr > max_comp_speed) {
                                max_comp_speed = abs_vr;
                            }
                        }
                    }
                }
            }
        }
        
        const bool has_speed_from_radar = (max_comp_speed > radar_static_vel_thresh_);
        bool dyn_candidate = false;
        if (use_radar_) {
            dyn_candidate = is_occupied && (has_speed_from_particles || has_speed_from_radar);
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

    // [New] Execute Cluster-based Processing if enabled
    if (cluster_mode_) {
        processDynamicClustersAvg(ego_calib);
    }
}

void DynamicGridMap::processDynamicClustersAvg(const EgoCalibration& ego_calib) {
    // Simple Average-based Clustering
    // 클러스터 내 모든 셀의 속도를 단순 평균내서 적용
    
    std::vector<bool> visited(grid_.size(), false);
    const int cluster_search_radius = 2;

    for (int i = 0; i < grid_.size(); ++i) {
        if (visited[i] || !grid_[i].is_dynamic) continue;
        
        std::vector<int> cluster_indices;
        std::queue<int> q;
        q.push(i);
        visited[i] = true;
        
        // BFS 클러스터링
        while (!q.empty()) {
            int curr = q.front(); q.pop();
            cluster_indices.push_back(curr);
            int gx, gy; indexToGrid(curr, gx, gy);
            
            for (int dy = -cluster_search_radius; dy <= cluster_search_radius; ++dy) {
                for (int dx = -cluster_search_radius; dx <= cluster_search_radius; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = gx + dx; int ny = gy + dy;
                    if (isInside(nx, ny)) {
                        int nidx = gridToIndex(nx, ny);
                        if (!visited[nidx] && grid_[nidx].is_dynamic) {
                            visited[nidx] = true;
                            q.push(nidx);
                        }
                    }
                }
            }
        }
        
        if (cluster_indices.empty()) continue;

        // 클러스터 내 모든 셀의 속도 평균 계산
        double avg_vx = 0.0;
        double avg_vy = 0.0;
        int count = 0;
        
        for (int c_idx : cluster_indices) {
            avg_vx += grid_[c_idx].mean_vx;
            avg_vy += grid_[c_idx].mean_vy;
            count++;
        }
        
        if (count > 0) {
            avg_vx /= count;
            avg_vy /= count;
            
            // 평균 속도를 클러스터 내 모든 셀에 적용
            for (int c_idx : cluster_indices) {
                grid_[c_idx].mean_vx = avg_vx;
                grid_[c_idx].mean_vy = avg_vy;
            }
        }
    }
}

#if 0
void DynamicGridMap::processDynamicClusters(const EgoCalibration& ego_calib) {
    auto& parts = particle_filter_->getParticles();
    
    // 1. [Efficiency] 셀별 파티클 인덱스 맵핑 (O(N))
    // 전체 파티클을 매번 탐색하면 연산량이 많으므로, 각 셀별 파티클 구간을 미리 저장
    std::vector<std::pair<int, int>> cell_particle_ranges(grid_.size(), {-1, -1});
    int current_cell = -1;
    int start_p = 0;
    
    // parts 벡터는 grid_cell_idx 순으로 정렬되어 있다고 가정
    for (int i = 0; i <= static_cast<int>(parts.size()); ++i) {
        bool last = (i == static_cast<int>(parts.size()));
        int c_idx = last ? -1 : parts[i].grid_cell_idx;
        if (last || c_idx != current_cell) {
            if (current_cell >= 0 && current_cell < static_cast<int>(grid_.size())) {
                cell_particle_ranges[current_cell] = {start_p, i};
            }
            current_cell = c_idx;
            start_p = i;
        }
    }

    // 2. BFS Clustering (2칸 반경으로 병합 - 두 발을 하나로 묶기 위함)
    std::vector<bool> visited(grid_.size(), false);
    
    // [설정] 클러스터링 탐색 반경: 2칸 (Chebyshev distance 2)
    const int cluster_search_radius = 2; 

    for (int i = 0; i < grid_.size(); ++i) {
        // 이미 처리했거나 동적이 아닌 셀은 패스
        if (visited[i] || !grid_[i].is_dynamic) continue;
        
        std::vector<int> cluster_indices;
        std::queue<int> q;
        q.push(i);
        visited[i] = true;
        
        // --- BFS 탐색 시작 ---
        while (!q.empty()) {
            int curr = q.front(); q.pop();
            cluster_indices.push_back(curr);
            int gx, gy; indexToGrid(curr, gx, gy);
            
            // 2칸 반경 이웃 탐색
            for (int dy = -cluster_search_radius; dy <= cluster_search_radius; ++dy) {
                for (int dx = -cluster_search_radius; dx <= cluster_search_radius; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = gx + dx; int ny = gy + dy;
                    if (isInside(nx, ny)) {
                        int nidx = gridToIndex(nx, ny);
                        // 방문 안 했고, 동적 셀(is_dynamic)인 경우만 병합
                        if (!visited[nidx] && grid_[nidx].is_dynamic) {
                            visited[nidx] = true;
                            q.push(nidx);
                        }
                    }
                }
            }
        }
        
        if (cluster_indices.empty()) continue;

        // 3. [Consensus Step 1] 각 셀의 대표(Local Candidates) 선출
        // 클러스터 내의 각 셀에서 가장 가중치가 높은 파티클을 후보로 뽑습니다.
        std::vector<const Particle*> candidates;
        candidates.reserve(cluster_indices.size());

        for (int c_idx : cluster_indices) {
            std::pair<int, int> range = cell_particle_ranges[c_idx];
            if (range.first == -1) continue;

            double local_max_weight = -1.0;
            int local_winner_idx = -1;

            // 해당 셀 안에서 1등 찾기
            for (int p = range.first; p < range.second; ++p) {
                if (parts[p].weight > local_max_weight) {
                    local_max_weight = parts[p].weight;
                    local_winner_idx = p;
                }
            }
            if (local_winner_idx != -1) {
                candidates.push_back(&parts[local_winner_idx]);
            }
        }

        if (candidates.empty()) continue;

        // 4. [Consensus Step 2] 상호 검증 투표 (Voting)
        // 후보들끼리 서로 비교하여 가장 많은 지지(가중치 합)를 받는 파티클을 선정합니다.
        const Particle* best_consensus_particle = candidates[0];
        double max_consensus_score = -1.0;

        for (const auto* cand : candidates) {
            double current_score = 0.0;
            
            for (const auto* voter : candidates) {
                // [검증] yaml 파일의 설정값(0.3m/s, 5.0도)을 사용하여 유사도 판단
                if (particle_filter_->checkSectorMatch(*cand, *voter, 
                                                       particle_vector_vel_thresh_, 
                                                       particle_vector_ang_thresh_)) 
                {
                    // 지지도는 상대방의 Weight만큼 합산 (신뢰도 기반 투표)
                    current_score += voter->weight;
                }
            }

            if (current_score > max_consensus_score) {
                max_consensus_score = current_score;
                best_consensus_particle = cand;
            }
        }

        // 5. [Final Step] 리더 기준 최종 가중 평균 (Re-Gating)
        // 선정된 '진짜 대장'을 기준으로 클러스터 내 모든 파티클을 다시 필터링합니다.
        const Particle& winner = *best_consensus_particle;
        double cluster_vx_sum = 0.0;
        double cluster_vy_sum = 0.0;
        double cluster_w_sum = 0.0;

        for (int c_idx : cluster_indices) {
            std::pair<int, int> range = cell_particle_ranges[c_idx];
            if (range.first == -1) continue;

            for (int p = range.first; p < range.second; ++p) {
                // 리더와 방향이 맞는 파티클들만 최종 합산 (노이즈 제거)
                // 여기서도 yaml 파일의 설정값을 사용합니다.
                if (particle_filter_->checkSectorMatch(winner, parts[p], 
                                                       particle_vector_vel_thresh_, 
                                                       particle_vector_ang_thresh_)) 
                {
                    cluster_vx_sum += parts[p].vx * parts[p].weight;
                    cluster_vy_sum += parts[p].vy * parts[p].weight;
                    cluster_w_sum  += parts[p].weight;
                }
            }
        }

        // 6. 결과 적용
        if (cluster_w_sum > 1e-9) {
            double final_vx = cluster_vx_sum / cluster_w_sum;
            double final_vy = cluster_vy_sum / cluster_w_sum;

            // 계산된 "정제된 속도"를 클러스터 내 모든 셀에 일괄 적용
            for (int c_idx : cluster_indices) {
                grid_[c_idx].mean_vx = final_vx;
                grid_[c_idx].mean_vy = final_vy;
            }
        }
    }
}
#endif

// void DynamicGridMap::processDynamicClusters(const EgoCalibration& ego_calib) {
//     auto& parts = particle_filter_->getParticles();
    
//     // 1. [Efficiency] 파티클 검색 최적화를 위한 맵핑 (Cell Index -> Particle Range)
//     // 전체 파티클을 매번 탐색하면 연산량이 많으므로, 각 셀별 파티클 구간을 미리 저장
//     std::vector<std::pair<int, int>> cell_particle_ranges(grid_.size(), {-1, -1});
//     int current_cell = -1;
//     int start_p = 0;
    
//     // parts 벡터는 grid_cell_idx 순으로 정렬되어 있다고 가정
//     for (int i = 0; i <= static_cast<int>(parts.size()); ++i) {
//         bool last = (i == static_cast<int>(parts.size()));
//         int c_idx = last ? -1 : parts[i].grid_cell_idx;
//         if (last || c_idx != current_cell) {
//             if (current_cell >= 0 && current_cell < static_cast<int>(grid_.size())) {
//                 cell_particle_ranges[current_cell] = {start_p, i};
//             }
//             current_cell = c_idx;
//             start_p = i;
//         }
//     }

//     // 2. BFS Clustering (2칸 반경으로 병합 - 두 발을 하나로 묶기 위함)
//     std::vector<bool> visited(grid_.size(), false);
    
//     // [수정 포인트 1] 클러스터링 반경을 2칸(Chebyshev distance 2)으로 설정
//     const int cluster_search_radius = 2; 

//     for (int i = 0; i < grid_.size(); ++i) {
//         // 이미 처리했거나 동적이 아닌 셀은 패스
//         if (visited[i] || !grid_[i].is_dynamic) continue;
        
//         std::vector<int> cluster_indices;
//         std::queue<int> q;
//         q.push(i);
//         visited[i] = true;
        
//         // --- BFS 탐색 시작 ---
//         while (!q.empty()) {
//             int curr = q.front(); q.pop();
//             cluster_indices.push_back(curr);
//             int gx, gy; indexToGrid(curr, gx, gy);
            
//             // 2칸 반경 이웃 탐색
//             for (int dy = -cluster_search_radius; dy <= cluster_search_radius; ++dy) {
//                 for (int dx = -cluster_search_radius; dx <= cluster_search_radius; ++dx) {
//                     if (dx == 0 && dy == 0) continue;
//                     int nx = gx + dx; int ny = gy + dy;
//                     if (isInside(nx, ny)) {
//                         int nidx = gridToIndex(nx, ny);
//                         // 방문 안 했고, 동적 셀(is_dynamic)인 경우만 병합
//                         if (!visited[nidx] && grid_[nidx].is_dynamic) {
//                             visited[nidx] = true;
//                             q.push(nidx);
//                         }
//                     }
//                 }
//             }
//         }
        
//         if (cluster_indices.empty()) continue;

//         // 3. [수정 포인트 2] "Global Winner" 찾기 (클러스터 전체 파티클 대상)
//         double global_max_weight = -1.0;
//         int global_winner_idx = -1;

//         // 클러스터에 속한 모든 셀을 뒤져서 최고 가중치 파티클을 찾음
//         for (int c_idx : cluster_indices) {
//             std::pair<int, int> range = cell_particle_ranges[c_idx];
//             if (range.first == -1) continue; // 파티클 없는 셀은 패스

//             for (int p = range.first; p < range.second; ++p) {
//                 if (parts[p].weight > global_max_weight) {
//                     global_max_weight = parts[p].weight;
//                     global_winner_idx = p;
//                 }
//             }
//         }

//         // 유효한 파티클이 하나도 없으면 스킵
//         if (global_winner_idx == -1) continue; 

//         // 4. [수정 포인트 3] Global Winner 기준으로 가중 평균 (Re-Gating)
//         const Particle& winner = parts[global_winner_idx];
//         double cluster_vx_sum = 0.0;
//         double cluster_vy_sum = 0.0;
//         double cluster_w_sum = 0.0;

//         for (int c_idx : cluster_indices) {
//             std::pair<int, int> range = cell_particle_ranges[c_idx];
//             if (range.first == -1) continue;

//             for (int p = range.first; p < range.second; ++p) {
//                 // 검증: 이 파티클이 Winner(대장)와 속도/방향이 비슷한가?
//                 // 비슷하지 않다면(노이즈라면) 과감히 합산에서 제외됨 (Internal Noise Filtering 효과)
//                 if (particle_filter_->checkSectorMatch(winner, parts[p], 
//                                                        particle_vector_vel_thresh_, 
//                                                        particle_vector_ang_thresh_)) 
//                 {
//                     cluster_vx_sum += parts[p].vx * parts[p].weight;
//                     cluster_vy_sum += parts[p].vy * parts[p].weight;
//                     cluster_w_sum  += parts[p].weight;
//                 }
//             }
//         }

//         // 5. 최종 결과 적용
//         if (cluster_w_sum > 1e-9) {
//             double final_vx = cluster_vx_sum / cluster_w_sum;
//             double final_vy = cluster_vy_sum / cluster_w_sum;

//             // 계산된 "정제된 속도"를 클러스터 내 모든 셀에 일괄 적용 (Object Consistency)
//             for (int c_idx : cluster_indices) {
//                 grid_[c_idx].mean_vx = final_vx;
//                 grid_[c_idx].mean_vy = final_vy;
//             }
//         }
//     }
// }


std::vector<Particle> DynamicGridMap::generateNewParticles(double newborn_vel_stddev,
                                               double min_dynamic_birth_ratio,
                                               double max_dynamic_birth_ratio,
                                               double max_radar_speed_for_scaling,
                                               double dynamic_newborn_vel_stddev,
                                               const EgoCalibration& ego_calib)
{
    std::vector<Particle> new_particles;
    
    // 1. Ego Motion 보정용 정적 속도 가져오기
    double static_vx, static_vy;
    ego_calib.getStaticParticleVelocity(static_vx, static_vy);
    
    // 2. 속도 분포 설정 (기존 로직 유지)
    std::normal_distribution<double> static_vel_dist_x(static_vx, newborn_vel_stddev);
    std::normal_distribution<double> static_vel_dist_y(static_vy, newborn_vel_stddev);
    // 동적 파티클은 방향성 없이 랜덤하게 퍼짐 (Zero Mean + Large Stddev)
    std::normal_distribution<double> dynamic_fallback_dist(0.0, dynamic_newborn_vel_stddev);

    // [수정 핵심] 위치 랜덤 분포 추가 (Uniform Distribution)
    // 파티클 생성 위치를 셀 중심에서 셀 크기(resolution) 전체 영역으로 확장
    // 범위: [-resolution_/2.0, +resolution_/2.0]
    std::uniform_real_distribution<double> pos_dist(-resolution_ / 2.0, resolution_ / 2.0);

    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            int idx = gridToIndex(x, y);
            const auto& cell = grid_[idx];

            // 탄생(Birth) 조건: 확률 질량(rho_b)이 충분하고 점유된 셀일 때
            if (cell.rho_b > 0.5 && cell.m_occ > 0.6) {
                // [NEW] 이미 동적으로 추적 중인 셀이면 새 파티클 생성 안 함
                // 이유: is_dynamic = true는 이미 파티클이 해당 영역을 추적 중이라는 의미
                if (cell.is_dynamic) {
                    continue;  // 건너뛰기
                }
                
                int num_to_birth = static_cast<int>(std::ceil(cell.rho_b * 4.0));
                
                // [Radar Hint 로직] - 여기서는 동적 파티클 비율 계산에만 사용 (기존 유지)
                double max_comp_speed = 0.0;
                if (use_radar_) {
                    for (int dy = -radar_hint_search_radius_; dy <= radar_hint_search_radius_; ++dy) {
                        for (int dx = -radar_hint_search_radius_; dx <= radar_hint_search_radius_; ++dx) {
                            int nx = x + dx; int ny = y + dy;
                            if (isInside(nx, ny)) {
                                const auto& neighbor = grid_[gridToIndex(nx, ny)];
                                double cell_wx, cell_wy;
                                gridToWorld(nx, ny, cell_wx, cell_wy);
                                for (const auto& hint : neighbor.radar_hints) {
                                    double azimuth = std::atan2(cell_wy - hint.sensor_y, cell_wx - hint.sensor_x);
                                    double abs_vr = std::abs(ego_calib.getAbsoluteRadialVelocity(hint.vr, azimuth));
                                    if (abs_vr > max_comp_speed) max_comp_speed = abs_vr;
                                }
                            }
                        }
                    }
                }

                // 동적 파티클 비율 결정
                double current_dynamic_ratio = min_dynamic_birth_ratio;
                if (max_comp_speed > 1e-6) {
                    double scale = std::min(1.0, max_comp_speed / std::max(1e-6, max_radar_speed_for_scaling));
                    current_dynamic_ratio = min_dynamic_birth_ratio + (max_dynamic_birth_ratio - min_dynamic_birth_ratio) * scale;
                }

                int num_dynamic = static_cast<int>(num_to_birth * current_dynamic_ratio);
                int num_static = num_to_birth - num_dynamic;

                // 3. 정적 파티클 생성 (Static Particles)
                for (int i = 0; i < num_static; ++i) {
                    Particle p;
                    gridToWorld(x, y, p.x, p.y); // 셀 중심 좌표 할당
                    
                    // [적용] 위치 랜덤 노이즈 추가 -> 셀 전체 영역에 분포
                    p.x += pos_dist(random_generator_);
                    p.y += pos_dist(random_generator_);
                    
                    p.vx = static_vel_dist_x(random_generator_);
                    p.vy = static_vel_dist_y(random_generator_);
                    p.weight = cell.rho_b / static_cast<double>(num_to_birth);
                    p.grid_cell_idx = idx; p.age = 0;
                    new_particles.push_back(p);
                }

                // 4. 동적 파티클 생성 (Dynamic Particles)
                for (int i = 0; i < num_dynamic; ++i) {
                    Particle p;
                    gridToWorld(x, y, p.x, p.y); // 셀 중심 좌표 할당
                    
                    // [적용] 위치 랜덤 노이즈 추가 -> 셀 전체 영역에 분포 (탈출 확률 확보)
                    p.x += pos_dist(random_generator_);
                    p.y += pos_dist(random_generator_);

                    // 속도는 기존과 동일하게 랜덤 분포 사용 (방향성 없음)
                    p.vx = dynamic_fallback_dist(random_generator_) + static_vx; 
                    p.vy = dynamic_fallback_dist(random_generator_) + static_vy; 
                    
                    p.weight = cell.rho_b / static_cast<double>(num_to_birth);
                    p.grid_cell_idx = idx; p.age = 0;
                    new_particles.push_back(p);
                }
            }
        }
    }
    return new_particles;
}

void DynamicGridMap::toOccupancyGridMsg(nav_msgs::OccupancyGrid& msg, const std::string& frame_id) const {
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
#if !ONE_ARROW
void DynamicGridMap::toMarkerArrayMsg(visualization_msgs::MarkerArray& arr,
                                      const std::string& frame_id,
                                      bool show_velocity_arrows,
                                      const EgoCalibration& ego_calib) const {
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
            } else if (c.m_free >= m_unk) { col.r = col.g = col.b = 1.0f; col.a = 0.5f; } 
            else { col.r = col.g = col.b = 0.5f; col.a = 0.5f; }
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
                p0.x = origin_x_ + (x + 0.5) * resolution_; p0.y = origin_y_ + (y + 0.5) * resolution_; p0.z = 0.00;
                double abs_vx, abs_vy;
                ego_calib.getAbsoluteVelocity(c.mean_vx, c.mean_vy, abs_vx, abs_vy);
                const double scale = 0.4;
                p1.x = p0.x + scale * abs_vx; p1.y = p0.y + scale * abs_vy; p1.z = 0.00;
                visualization_msgs::Marker a = arrows;
                a.id = arrow_id++; a.points.clear(); a.points.push_back(p0); a.points.push_back(p1);
                arr.markers.push_back(a);
            }
        }
    }
}
#endif

#if ONE_ARROW
void DynamicGridMap::toMarkerArrayMsg(visualization_msgs::MarkerArray& arr,
                                      const std::string& frame_id,
                                      bool show_velocity_arrows,
                                      const EgoCalibration& ego_calib) const {
    arr.markers.clear();
    
    // 1. 점유 그리드 (Cube List) 시각화 - 기존 로직 유지
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
            } else if (c.m_free >= m_unk) { col.r = col.g = col.b = 1.0f; col.a = 0.5f; } 
            else { col.r = col.g = col.b = 0.5f; col.a = 0.5f; }
            cubes.points.push_back(p); cubes.colors.push_back(col);
        }
    }
    arr.markers.push_back(cubes);

    // 2. 속도 벡터(Arrow) 시각화
    if (show_velocity_arrows) {
        visualization_msgs::Marker arrow_template;
        arrow_template.header.stamp = ros::Time::now(); 
        arrow_template.header.frame_id = frame_id; 
        arrow_template.ns = "dogm_vel"; 
        arrow_template.type = visualization_msgs::Marker::ARROW; 
        arrow_template.action = visualization_msgs::Marker::ADD;
        arrow_template.scale.x = 0.04; // Shaft diameter (ONE_ARROW=0과 동일)
        arrow_template.scale.y = 0.06;  // Head diameter (ONE_ARROW=0과 동일)
        arrow_template.scale.z = 0.06;  // Head length (ONE_ARROW=0과 동일)
        arrow_template.color.r = 1.0; arrow_template.color.g = 1.0; arrow_template.color.b = 0.0; arrow_template.color.a = 1.0;
        arrow_template.lifetime = ros::Duration(0.4);

        int arrow_id = 10;
        
        // --- [Case A] Cluster Mode ON: 클러스터 중심에 하나만 그리기 + 바운딩 박스 표시 ---
        if (cluster_mode_) {
            std::vector<bool> visited(grid_.size(), false);
            const int viz_cluster_radius = 2; // 시각화용 그룹핑 (로직과 동일하게 2칸)
            #if BOUNDING_BOX
            int bbox_id = 5000; // 바운딩 박스 마커 ID 시작점
            #endif

            for (int i = 0; i < grid_.size(); ++i) {
                // 아직 방문 안 했고, 동적이며, 점유 확률이 높은 셀만 시작점으로
                if (visited[i] || !grid_[i].is_dynamic || grid_[i].m_occ < 0.6) continue;

                // BFS로 클러스터 묶기
                std::vector<int> cluster_indices;
                std::queue<int> q;
                q.push(i);
                visited[i] = true;

                double sum_wx = 0.0;
                double sum_wy = 0.0;
                int count = 0;
                
                #if BOUNDING_BOX
                // 바운딩 박스 계산용 변수
                double min_wx = 1e9, max_wx = -1e9;
                double min_wy = 1e9, max_wy = -1e9;
                #endif

                // 클러스터의 대표 속도 (모두 동일하게 설정되어 있으므로 첫 번째 것 사용)
                double cluster_mean_vx = grid_[i].mean_vx;
                double cluster_mean_vy = grid_[i].mean_vy;

                while (!q.empty()) {
                    int curr = q.front(); q.pop();
                    cluster_indices.push_back(curr);
                    
                    // Centroid 계산을 위한 좌표 합산
                    int cx, cy; indexToGrid(curr, cx, cy);
                    double wx, wy; gridToWorld(cx, cy, wx, wy);
                    sum_wx += wx;
                    sum_wy += wy;
                    count++;
                    
                    #if BOUNDING_BOX
                    // 바운딩 박스 min/max 업데이트
                    min_wx = std::min(min_wx, wx);
                    max_wx = std::max(max_wx, wx);
                    min_wy = std::min(min_wy, wy);
                    max_wy = std::max(max_wy, wy);
                    #endif

                    for (int dy = -viz_cluster_radius; dy <= viz_cluster_radius; ++dy) {
                        for (int dx = -viz_cluster_radius; dx <= viz_cluster_radius; ++dx) {
                            if (dx == 0 && dy == 0) continue;
                            int nx = cx + dx; int ny = cy + dy;
                            if (isInside(nx, ny)) {
                                int nidx = gridToIndex(nx, ny);
                                // 동적이고 점유된 셀만 연결
                                if (!visited[nidx] && grid_[nidx].is_dynamic && grid_[nidx].m_occ >= 0.6) {
                                    visited[nidx] = true;
                                    q.push(nidx);
                                }
                            }
                        }
                    }
                }

                // 클러스터 하나 완성 -> 빨간색 원 + 화살표 생성
                if (count > 0) {
                    // 클러스터 중심 계산
                    double centroid_x = sum_wx / count;
                    double centroid_y = sum_wy / count;
                    
                    #if BOUNDING_BOX
                    // 1. 클러스터 중심에 빨간색 원 표시
                    visualization_msgs::Marker circle;
                    circle.header.stamp = ros::Time::now();
                    circle.header.frame_id = frame_id;
                    circle.ns = "cluster_circle";
                    circle.id = bbox_id++;
                    circle.type = visualization_msgs::Marker::CYLINDER;
                    circle.action = visualization_msgs::Marker::ADD;
                    
                    // 원의 중심 위치
                    circle.pose.position.x = centroid_x;
                    circle.pose.position.y = centroid_y;
                    circle.pose.position.z = 0.01; // 바닥에 가깝게
                    circle.pose.orientation.w = 1.0;
                    
                    // 클러스터 크기 계산 (바운딩 박스 대각선의 절반)
                    double width = max_wx - min_wx + resolution_;
                    double height = max_wy - min_wy + resolution_;
                    double radius = std::sqrt(width * width + height * height) / 2.0;
                    
                    circle.scale.x = radius * 2.0; // 지름
                    circle.scale.y = radius * 2.0; // 지름
                    circle.scale.z = 0.02; // 원의 두께 (얇게)
                    
                    circle.color.r = 1.0;  // 빨간색
                    circle.color.g = 0.0;
                    circle.color.b = 0.0;
                    circle.color.a = 0.3;  // 반투명
                    circle.lifetime = ros::Duration(0.2);
                    
                    arr.markers.push_back(circle);
                    #endif

                    // 2. 클러스터 중심에 속도 화살표 생성 (빨간색 유지)
                    geometry_msgs::Point p0, p1;
                    p0.x = centroid_x; 
                    p0.y = centroid_y; 
                    p0.z = 0.01; // 빨간색 원(z=0.01) 위에 표시

                    double abs_vx, abs_vy;
                    ego_calib.getAbsoluteVelocity(cluster_mean_vx, cluster_mean_vy, abs_vx, abs_vy);
                    
                    // 속도 크기가 너무 작으면 화살표 생략 (깔끔한 시각화)
                    double speed = std::sqrt(abs_vx*abs_vx + abs_vy*abs_vy);
                    if (speed < 0.1) continue;

                    const double scale = 0.8; // ONE_ARROW=0과 동일 (0.4)
                    p1.x = p0.x + scale * abs_vx; 
                    p1.y = p0.y + scale * abs_vy; 
                    p1.z = 0.02; // 빨간색 원(z=0.01) 위에 표시

                    visualization_msgs::Marker a = arrow_template;
                    a.id = arrow_id++;
                    a.points.push_back(p0); 
                    a.points.push_back(p1);
                    arr.markers.push_back(a);
                }
            }
        } 
        // --- [Case B] Cluster Mode OFF: 개별 셀마다 그리기 (기존 방식) ---
        else {
            for (int y = 0; y < grid_height_; ++y) {
                for (int x = 0; x < grid_width_; ++x) {
                    const auto& c = grid_[gridToIndex(x, y)];
                    if (!c.is_dynamic || c.m_occ < 0.6) continue;

                    geometry_msgs::Point p0, p1;
                    p0.x = origin_x_ + (x + 0.5) * resolution_; 
                    p0.y = origin_y_ + (y + 0.5) * resolution_; 
                    p0.z = 0.0;

                    double abs_vx, abs_vy;
                    ego_calib.getAbsoluteVelocity(c.mean_vx, c.mean_vy, abs_vx, abs_vy);
                    
                    const double scale = 0.4;
                    p1.x = p0.x + scale * abs_vx; 
                    p1.y = p0.y + scale * abs_vy; 
                    p1.z = 0.0;

                    visualization_msgs::Marker a = arrow_template;
                    a.id = arrow_id++;
                    a.points.push_back(p0); 
                    a.points.push_back(p1);
                    arr.markers.push_back(a);
                }
            }
        }
    }
}
#endif

void DynamicGridMap::allParticlesToMarkerMsg(visualization_msgs::Marker& marker, const std::string& frame_id) const {
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = "all_particles";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    // 화살표 선 두께 (0.005m = 5mm)
    marker.scale.x = 0.005; 

    // 초록색 설정 (요청사항)
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.6f;

    marker.lifetime = ros::Duration(0.1);

    const auto& particles = particle_filter_->getParticles();
    const double shaft_scale = 0.15; // 몸통 길이 배율
    const double head_len = 0.04;    // 화살표 머리 날개 길이
    const double cos30 = 0.866, sin30 = 0.5; // 30도 회전용 상수

    // 화살표 하나당 3개의 선(점 6개)이 필요함
    marker.points.reserve(particles.size() * 6);

    for (const auto& p : particles) {
        double vx = p.vx;
        double vy = p.vy;
        double speed = std::sqrt(vx * vx + vy * vy);

        // 1. 몸통 (Shaft) 시작점과 끝점 계산
        geometry_msgs::Point p_start, p_end;
        p_start.x = p.x;
        p_start.y = p.y;
        p_start.z = 0.1; // 격자 위로 약간 띄움

        p_end.x = p.x + vx * shaft_scale;
        p_end.y = p.y + vy * shaft_scale;
        p_end.z = 0.1;
        
        marker.points.push_back(p_start);
        marker.points.push_back(p_end);

        // 속도가 너무 작으면 머리는 그리지 않음
        if (speed > 0.1) {
            // 역방향 단위 벡터 계산
            double ux = -vx / speed * head_len;
            double uy = -vy / speed * head_len;

            // 2. 머리 날개 1 (Left Wing)
            geometry_msgs::Point h1;
            h1.x = p_end.x + (ux * cos30 - uy * sin30);
            h1.y = p_end.y + (ux * sin30 + uy * cos30);
            h1.z = 0.1;
            marker.points.push_back(p_end);
            marker.points.push_back(h1);

            // 3. 머리 날개 2 (Right Wing)
            geometry_msgs::Point h2;
            h2.x = p_end.x + (ux * cos30 + uy * sin30);
            h2.y = p_end.y + (-ux * sin30 + uy * cos30);
            h2.z = 0.1;
            marker.points.push_back(p_end);
            marker.points.push_back(h2);
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
            int old_x = x + shift_x; int old_y = y + shift_y;
            if (isInside(old_x, old_y)) {
                int new_idx = gridToIndex(x, y); int old_idx = gridToIndex(old_x, old_y);
                new_grid[new_idx] = grid_[old_idx];
                new_meas[new_idx] = measurement_grid_[old_idx];
                for(auto& p : new_grid[new_idx].radar_buffer_1) { p.x -= dx; p.y -= dy; }
                for(auto& p : new_grid[new_idx].radar_buffer_2) { p.x -= dx; p.y -= dy; }
            }
        }
    }
    grid_ = new_grid;
    measurement_grid_ = new_meas;
}