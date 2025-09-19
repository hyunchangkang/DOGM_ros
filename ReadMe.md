# ROS 기반 동적 점유 격자 지도 (DOGM) 구현 프로젝트

## 1. 개요

이 ROS 패키지는 [D. Nuss, et al. (2018)[cite_start], "A random finite set approach for dynamic occupancy grid maps with real-time application"](https://journals.sagepub.com/doi/10.1177/0278364918775523) 논문에서 제안된 **DS-PHD/MIB 필터**를 CPU 기반 환경에서 실시간으로 구현하는 것을 목표로 합니다[cite: 3].

2D LiDAR 센서(`sensor_msgs/LaserScan`) 데이터를 입력받아, 주변 환경을 동적 정보가 포함된 점유 격자 지도로 생성합니다. 각 격자 셀은 단순한 점유/비점유 상태를 넘어, 동적/정적 상태로 분류되며 동적 셀의 경우 추정된 속도(방향 포함)를 RViz에서 시각화하여 확인할 수 있습니다.

### 핵심 기능
-   **실시간 동적 맵핑**: LiDAR 데이터를 이용해 실시간으로 동적 점유 격자 지도(DOGM)를 생성합니다.
-   **동적/정적 분류**: 각 셀을 동적 또는 정적 상태로 분류합니다.
-   **속도 추정**: 파티클 필터를 사용하여 동적 셀의 2D 속도를 추정합니다.
-   **유연한 설정**: 그리드 크기, 해상도, 파티클 수 등의 주요 파라미터를 YAML 파일로 관리합니다.
-   **시각화**: `nav_msgs/OccupancyGrid`와 `visualization_msgs/MarkerArray` 토픽을 발행하여 RViz에서 맵과 속도 벡터를 직관적으로 확인할 수 있습니다.

---

## 2. 핵심 이론 및 구현 전략

### 2.1. Dempster-Shafer (DS) 이론
[cite_start]각 셀의 상태를 확률이 아닌 '믿음(Belief)' 또는 '증거(Evidence)'의 개념으로 표현합니다[cite: 578]. [cite_start]`m(Occupied)`(점유 질량), `m(Free)`(비점유 질량)을 각각 관리하며, `1 - m(O) - m(F)`는 '알 수 없음(Unknown)'에 대한 질량이 됩니다[cite: 603, 604, 605]. [cite_start]이를 통해 관측되지 않은 영역과 명확히 비어있는 영역을 효과적으로 구분합니다[cite: 578].

### 2.2. DS-PHD/MIB 필터
[cite_start]논문에서 제안된 실시간 근사 필터입니다[cite: 1100]. [cite_start]점유에 대한 증거(`m(Occupied) > 0`)가 있는 셀에 대해서만 **파티클 필터**를 적용하여 동적 상태(속도)를 추정함으로써, 전체 계산량을 크게 줄여 CPU 환경에서의 실시간성을 확보합니다[cite: 579, 681].

### 2.3. 파티클 필터 (Particle Filter)
[cite_start]점유된 셀의 속도 분포라는 비선형/비가우시안 시스템을 추정하기 위해 다수의 파티클(가중치를 가진 샘플)을 사용합니다[cite: 144]. [cite_start]각 파티클은 '특정 속도'라는 가설을 나타내며, **예측(Prediction) → 업데이트(Update) → 재샘플링(Resampling)** 과정을 반복하며 실제 속도 분포에 근사해갑니다[cite: 145].

### 2.4. CPU 최적화 전략
[cite_start]논문의 핵심 아이디어인 **파티클 정렬(Sorting)**을 채택합니다[cite: 778]. [cite_start]예측 단계 이후 모든 파티클을 자신이 속한 그리드 셀 인덱스 기준으로 정렬합니다[cite: 778]. [cite_start]이 간단한 전처리만으로 각 셀에 속한 파티클 그룹을 매우 빠르게 찾을 수 있어, 전체 업데이트 과정의 계산 효율성이 비약적으로 향상됩니다[cite: 781, 784].

---

## 3. 시스템 요구사항 및 의존성

-   **OS**: Ubuntu 18.04
-   **ROS**: ROS Melodic Morenia
-   **의존성 패키지**:
    -   `roscpp`
    -   `tf2_ros`
    -   `sensor_msgs`
    -   `nav_msgs`
    -   `visualization_msgs`
    -   `geometry_msgs`

---

## 4. 패키지 구조

`catkin_create_pkg dogm_ros [의존성...]` 명령어로 패키지를 생성한 후, 아래와 같이 디렉토리와 파일을 구성합니다.
dogm_ros/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── dogm.launch
├── config/
│   └── params.yaml
└── src/
├── dogm_node.cpp         # ROS 인터페이스 및 메인 루프
├── dynamic_grid_map.cpp  # 그리드 관련 로직
├── dynamic_grid_map.h
├── particle_filter.cpp   # 파티클 필터 알고리즘
├── particle_filter.h
└── structures.h          # 공용 자료구조 (Particle, GridCell)

---

## 5. 구현 아키텍처 및 동작 흐름

### 5.1. 클래스 구조 및 역할

코드는 역할에 따라 3개의 핵심 클래스로 분리하여 구현합니다.

| 클래스명          | 파일                      | 역할                                                                                                 |
| ----------------- | ------------------------- | ---------------------------------------------------------------------------------------------------- |
| `DOGMNode`        | `dogm_node.cpp`           | **ROS 인터페이스 및 총괄 지휘자**. 토픽 Sub/Pub, TF, 파라미터 관리 및 메인 필터 루프를 실행하며 다른 클래스들을 조율. |
| `DynamicGridMap`  | `dynamic_grid_map.h/.cpp` | **그리드 관리자**. 그리드 데이터 저장, LiDAR로부터 측정 그리드 생성, 뎀스터 조합 규칙을 이용한 셀 상태 업데이트 담당. |
| `ParticleFilter`  | `particle_filter.h/.cpp`  | **동적 상태 추정 엔진**. 파티클의 예측, 정렬, 재샘플링 등 순수 파티클 필터 알고리즘을 수행.                     |

### 5.2. 실시간 동작 흐름 (Real-time Workflow)

프로그램은 **LiDAR 콜백**과 **타이머 콜백(메인 루프)** 두 축으로 동작합니다.

1.  **(LiDAR 콜백) `scanCallback` - 비동기적 실행**:
    -   `/scan` 토픽이 수신될 때마다 호출됩니다.
    -   `tf`를 이용해 현재 센서의 위치를 확인합니다.
    -   [cite_start]`DynamicGridMap` 클래스를 이용해 LiDAR 스캔 데이터로부터 **측정 그리드(Measurement Grid)**를 생성하고 저장합니다[cite: 126]. [cite_start]이 그리드는 각 셀에 대한 최신 관측 증거(`m_z(O)`, `m_z(F)`)를 담고 있습니다[cite: 625].

2.  **(타이머 콜백) `updateFilter` - 주기적 실행**:
    -   `params.yaml`에 정의된 `filter_update_rate` (예: 20Hz)에 맞춰 주기적으로 호출됩니다.
    -   [cite_start]**① 예측 (Prediction)**: `ParticleFilter`가 모든 파티클의 다음 상태(위치, 속도)를 예측하고 생존 확률을 곱해 가중치를 업데이트합니다[cite: 151, 643].
    -   [cite_start]**② 정렬 (Sorting)**: `ParticleFilter`가 예측된 파티클들을 그리드 셀 인덱스 기준으로 정렬합니다[cite: 821].
    -   [cite_start]**③ 점유 상태 업데이트 (Occupancy Update)**: `DynamicGridMap`이 정렬된 파티클 정보와 최신 측정 그리드를 이용해 각 셀의 `m(O)`, `m(F)`를 **뎀스터 조합 규칙**으로 갱신합니다[cite: 673]. [cite_start]이 과정에서 각 셀의 점유 질량이 지속(`rho_p`) 및 신규(`rho_b`) 부분으로 분리됩니다[cite: 676, 683, 684].
    -   **④ 파티클 가중치 업데이트 및 생성 (Weight Update & Birth)**:
        -   [cite_start]`rho_p` 값에 비례하여 각 셀에 속한 지속(persistent) 파티클들의 가중치를 정규화합니다[cite: 701, 704].
        -   [cite_start]`rho_b` 값에 비례하여 해당 셀에 새로운(new-born) 파티클들을 생성합니다[cite: 727].
    -   [cite_start]**⑤ 재샘플링 (Resampling)**: `ParticleFilter`가 전체 파티클(지속+신규)을 대상으로 가중치 기반 재샘플링을 수행하여 다음 루프를 준비합니다[cite: 159, 744].
    -   [cite_start]**⑥ 통계 계산 및 분류 (Statistics & Classification)**: `DynamicGridMap`이 최종 파티클 분포를 바탕으로 각 셀의 평균 속도, 분산을 계산하고 [cite: 859] [cite_start]마할라노비스 거리를 이용해 동적/정적 상태를 분류합니다[cite: 1053, 1055].
    -   **⑦ 결과 발행 (Publishing)**: `DOGMNode`가 `DynamicGridMap`의 최종 결과물을 가져와 `nav_msgs/OccupancyGrid`와 `visualization_msgs/MarkerArray` 토픽으로 발행합니다.

---

## 6. 설정 (`config/params.yaml`)

```yaml
# params.yaml

# 1. ROS Topics & Frames
lidar_topic: "/scan"
grid_topic: "/dogm/grid"
marker_topic: "/dogm/velocity_markers"
fixed_frame: "odom"      # 맵의 기준이 될 고정 좌표계 (e.g., odom, map)
base_frame: "base_link"  # 라이다 기준 좌표계

# 2. Grid Map Parameters
grid_size: 50.0      # 그리드 맵 한 변의 길이 (meters)
grid_resolution: 0.2 # 셀 하나의 크기 (meters/cell)

# 3. Particle Filter Parameters
num_particles: 200000  # 전체 파티클 수

# 4. Process Model Parameters (DS-PHD/MIB)
persistence_prob: 0.99   # 파티클 생존 확률 (p_S)
birth_prob: 0.02         # 신규 객체 탄생 확률 (p_B)
process_noise_pos: 0.05  # 위치 프로세스 노이즈 표준편차 (m)
process_noise_vel: 0.50  # 속도 프로세스 노이즈 표준편차 (m/s)
newborn_vel_stddev: 2.0  # 신규 파티클 초기 속도 표준편차 (m/s)

# 5. Classification Parameters
mahalanobis_dist_thresh: 9.21 # 동적/정적 분류 임계값 (카이제곱 분포, 2자유도, p=0.01)

# 6. Update Frequency
filter_update_rate: 20.0 # 메인 필터 루프 실행 주기 (Hz)