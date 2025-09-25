#ifndef DOGM_ROS_STRUCTURES_H
#define DOGM_ROS_STRUCTURES_H

#include <cstdint>
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

struct GridCell {
    double m_occ{0.0};
    double m_free{0.0};
    double rho_b{0.0};
    double rho_p{0.0};
    double mean_vx{0.0}, mean_vy{0.0};
    double var_vx{0.0},  var_vy{0.0}, covar_vxy{0.0};
    bool   is_dynamic{false};
    double dynamic_score{0.0};
    std::uint8_t dyn_streak{0};
    std::uint8_t stat_streak{0};
    bool   has_radar_hint{false};
    double radar_vx{0.0};
    double radar_vy{0.0};
};

struct MeasurementCell {
    double m_occ_z{0.0};
    double m_free_z{0.0};
};

#endif // DOGM_ROS_STRUCTURES_H