#ifndef DOGM_ROS_STRUCTURES_H
#define DOGM_ROS_STRUCTURES_H

#include <cstdint>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct mmWaveCloudType
{
    PCL_ADD_POINT4D;
    union
    {
        struct
        {
            float intensity; 
            float velocity;  
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
    int    age{0}; 
};

// [수정] RadarPoint에 센서 정보를 포함하지 않고 Buffer 별로 관리
struct RadarPoint
{
    double radial_velocity; 
    double x;               
    double y;               
    int age;                
};

// [신규] 정제된 레이다 힌트 구조체
struct RadarHint {
    double vr;          // 평균 도플러 속도
    double sensor_x;    // 해당 힌트를 만든 센서의 위치 (Azimuth 계산용)
    double sensor_y;    
    bool valid{false};
};

struct GridCell {
    double m_occ{0.0};
    double m_free{0.0};
    double rho_b{0.0};
    double rho_p{0.0};

    double mean_vx{0.0}, mean_vy{0.0};
    double var_vx{0.0},  var_vy{0.0}, covar_vxy{0.0};

    bool   is_dynamic{false};
    double dynamic_score{0.0};
    double mahalanobis_dist{0.0}; 

    std::uint8_t dyn_streak{0};
    std::uint8_t stat_streak{0};
    
    // [수정] 단일 힌트 변수 삭제 -> 힌트 벡터 사용
    // double radar_vr_hint{0.0}; (삭제)
    // double radar_theta_hint{0.0}; (삭제)
    
    // [신규] 여러 센서의 힌트를 저장하는 벡터
    std::vector<RadarHint> radar_hints;

    // [수정] 센서별 버퍼 분리 (최대 2개 가정)
    // buffer_1: Radar 1 (Right), buffer_2: Radar 2 (Left)
    std::vector<RadarPoint> radar_buffer_1; 
    std::vector<RadarPoint> radar_buffer_2; 

    std::uint8_t free_streak{0}; 
};

struct MeasurementCell {
    double m_occ_z{0.0};  
    double m_free_z{0.0}; 

    bool   has_lidar_model{false}; 
    double mean_x{0.0};
    double mean_y{0.0};
    double inv_cov_xx{0.0};
    double inv_cov_xy{0.0};
    double inv_cov_yy{0.0};
};

#endif // DOGM_ROS_STRUCTURES_H