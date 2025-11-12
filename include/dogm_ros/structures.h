#ifndef DOGM_ROS_STRUCTURES_H
#define DOGM_ROS_STRUCTURES_H

#include <cstdint>
#include <vector> // [MODIFIED] Include vector
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct mmWaveCloudType
{
    PCL_ADD_POINT4D;
    union
    {
        struct
        {
            float intensity; // SNR 값 저장용
            float velocity;  // Raw Radial Velocity (vr)
        };
        float data_c[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(mmWaveCloudType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, velocity, velocity)
)

struct Particle {
    double x{0.0}, y{0.0};
    double vx{0.0}, vy{0.0};
    double weight{0.0};
    int    grid_cell_idx{-1};
    int    age{0}; // 파티클의 나이 (프레임 단위)
};

/**
 * @struct RadarPoint
 * @brief [NEW] Stores raw 1D velocity (vr) and direction (x, y)
 */
struct RadarPoint
{
    double radial_velocity; // Raw vr (+/-)
    double x;               // Position for direction calculation
    double y;               // Position for direction calculation
    int age;                // 포인트가 추가된 후 지난 프레임 수
};

struct GridCell {
    // DS-PHD/MIB 질량
    double m_occ{0.0};
    double m_free{0.0};
    double rho_b{0.0};
    double rho_p{0.0};

    // 속도 통계
    double mean_vx{0.0}, mean_vy{0.0};
    double var_vx{0.0},  var_vy{0.0}, covar_vxy{0.0};

    // 동적 판정 & 시각화
    bool   is_dynamic{false};
    double dynamic_score{0.0};
    double mahalanobis_dist{0.0}; // (Original DOGM, can be repurposed)

    // 히스테리시스용 연속 프레임 카운트
    std::uint8_t dyn_streak{0};
    std::uint8_t stat_streak{0};
    
    // --- [MODIFIED] Radar 1D Hint ---
    double radar_vr_hint{0.0};      // Time-filtered 1D range_rate (+/-) (단일 셀의 시간 평균)
    double radar_theta_hint{0.0};   // Time-filtered 1D direction (angle)
    
    // [OPTIMIZATION] Pre-computed trigonometric values
    double radar_cos_theta{1.0};    // cos(radar_theta_hint)
    double radar_sin_theta{0.0};    // sin(radar_theta_hint)
    // ---------------------------------

    bool has_reliable_radar{false}; // True if buffer has enough points
    std::vector<RadarPoint> radar_points_buffer; // [NEW] Buffer for temporal averaging

    // [NEW] For False Static Detection
    std::uint8_t free_streak{0}; 
};

struct MeasurementCell {
    // [REMOVED] m_occ_z is replaced by the continuous model below
    // double m_occ_z{0.0};  
    
    double m_free_z{0.0}; // 측정 free 질량 (Still used)

    // --- [NEW] LiDAR Likelihood Model Parameters ---
    // (μ, Σ⁻¹) calculated in generateMeasurementGrid
    bool   has_lidar_model{false}; // True if this cell has a valid model
    
    // Mean (μ)
    double mean_x{0.0};
    double mean_y{0.0};

    // Pre-calculated Inverse Covariance Matrix (Σ⁻¹)
    double inv_cov_xx{0.0};
    double inv_cov_xy{0.0};
    double inv_cov_yy{0.0};
    // --- [END NEW] ---
};

#endif // DOGM_ROS_STRUCTURES_H