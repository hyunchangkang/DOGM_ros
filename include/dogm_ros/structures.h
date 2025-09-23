/////////제미나이가 바꿔준 버전////////////////

// #ifndef STRUCTURES_H
// #define STRUCTURES_H

// #include <vector>
// #include <cmath>

// // ===================================================================================
// // 1단계 수정: 데이터 구조 재설계
// // - CUDA 오픈소스와 논문을 기반으로, 안정적인 계산에 필요한 모든 변수를 포함하도록
// //   Particle, GridCell, MeasurementCell 구조체를 재정의합니다.
// // ===================================================================================


// /**
//  * @struct Particle
//  * @brief 파티클 필터의 각 입자를 나타냅니다. 위치, 속도, 가중치 가설을 가집니다.
//  */
// struct Particle {
//     // 파티클의 상태 변수
//     double x, y;      // 위치 (m)
//     double vx, vy;    // 속도 (m/s)
//     double weight;    // 파티클 가중치

//     /**
//      * @brief [핵심] 이 파티클이 속한 그리드 셀의 1차원 인덱스입니다.
//      * 이 변수는 모든 파티클을 셀별로 빠르게 정렬(sort)하는 데 사용됩니다.
//      */
//     int grid_cell_idx;

//     // 생성자
//     Particle() : x(0), y(0), vx(0), vy(0), weight(0), grid_cell_idx(-1) {}
// };


// /**
//  * @struct GridCell
//  * @brief 그리드 맵의 각 셀이 가지는 정보입니다.
//  */
// struct GridCell {
//     // --- 뎀스터-셰퍼(Dempster-Shafer) 질량 ---
//     double m_occ;      // 점유(Occupied)에 대한 믿음 질량
//     double m_free;     // 비점유(Free)에 대한 믿음 질량
//     // '알 수 없음(Unknown)' 질량은 (1.0 - m_occ - m_free)로 계산됩니다.

//     // --- PHD 필터 질량 (논문의 핵심) ---
//     /**
//      * @brief [핵심] 지속(persistent) 파티클에 의해 유지되는 점유 질량 (rho_p)
//      * 이전에 존재하던 장애물이 계속 존재할 것이라는 믿음을 나타냅니다.
//      * 이 값은 기존 파티클들의 가중치를 업데이트하는 데 사용됩니다.
//      */
//     double rho_p;

//     /**
//      * @brief [핵심] 신규 생성(new-born) 파티클에 할당될 점유 질량 (rho_b)
//      * 새로운 장애물이 나타났을 것이라는 믿음을 나타냅니다.
//      * 이 값에 비례하여 새로운 파티클이 생성됩니다.
//      */
//     double rho_b;

//     // --- 속도 통계 ---
//     double mean_vx, mean_vy;    // 평균 속도
//     double var_vx, var_vy;      // 속도 분산
//     double covar_vxy;           // 속도 공분산

//     // --- 동적/정적 분류 ---
//     bool is_dynamic;            // 최종 분류 결과 (주로 시각화 및 화살표 표시에 사용)
//     double dynamic_score;       // 0~1 사이의 동적 확률 점수 (색상 매핑에 사용)

//     // 생성자: 모든 값을 0 또는 false로 초기화
//     GridCell() : m_occ(0), m_free(0), rho_p(0), rho_b(0),
//                  mean_vx(0), mean_vy(0), var_vx(0), var_vy(0), covar_vxy(0),
//                  is_dynamic(false), dynamic_score(0.0) {}
// };


// /**
//  * @struct MeasurementCell
//  * @brief LiDAR 스캔으로부터 생성된 측정 그리드의 각 셀 정보입니다.
//  */
// struct MeasurementCell {
//     double m_occ_z;    // 측정된 점유 질량
//     double m_free_z;   // 측정된 비점유 질량

//     MeasurementCell() : m_occ_z(0), m_free_z(0) {}
// };

// #endif // STRUCTURES_H



// /////////GPT가 바꿔준 버전////////////////

// #ifndef DOGM_ROS_STRUCTURES_H
// #define DOGM_ROS_STRUCTURES_H

// #include <cstdint>

// struct Particle {
//     double x{0.0}, y{0.0};     // [m]
//     double vx{0.0}, vy{0.0};   // [m/s]
//     double weight{0.0};
//     int    grid_cell_idx{-1};
// };

// struct GridCell {
//     // DS-PHD/MIB 질량
//     double m_occ{0.0};
//     double m_free{0.0};
//     double rho_b{0.0};
//     double rho_p{0.0};

//     // 속도 통계
//     double mean_vx{0.0}, mean_vy{0.0};
//     double var_vx{0.0},  var_vy{0.0}, covar_vxy{0.0};

//     // 동적 판정 & 시각화
//     bool   is_dynamic{false};
//     double dynamic_score{0.0};     // 0~1
//     double mahalanobis_dist{0.0};  // 디버깅

//     // 히스테리시스용 연속 프레임 카운트
//     std::uint8_t dyn_streak{0};   // 동적 후보 연속 프레임 수
//     std::uint8_t stat_streak{0};  // 정적 후보 연속 프레임 수
// };

// struct MeasurementCell {
//     double m_occ_z{0.0};  // 측정 점유 질량
//     double m_free_z{0.0}; // 측정 free 질량
// };

// #endif // DOGM_ROS_STRUCTURES_H

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
            float intensity; // SNR 값 저장용
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
    int    age{0}; // [추가] 파티클의 나이 (프레임 단위)
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
    double mahalanobis_dist{0.0};

    // 히스테리시스용 연속 프레임 카운트
    std::uint8_t dyn_streak{0};
    std::uint8_t stat_streak{0};
    // Radar 힌트 저장을 위한 변수 추가
    bool   is_dynamic_by_radar{false};
    double radar_vx{0.0};
    double radar_vy{0.0};
};

struct MeasurementCell {
    double m_occ_z{0.0};  // 측정 점유 질량
    double m_free_z{0.0}; // 측정 free 질량
};

#endif // DOGM_ROS_STRUCTURES_H