# DOGM ROS 토픽 설명

## 주요 토픽들

### 1. `/dogm/grid` (nav_msgs/OccupancyGrid)
- **용도**: 표준 ROS OccupancyGrid 메시지
- **값 범위**: -1 (unknown), 0 (free), 1-100 (occupied)
- **특징**: 
  - RViz에서 흑백/회색으로 표시
  - 동적/정적 구분 불가
  - 표준 ROS navigation stack과 호환

### 2. `/dogm/cell_state` (visualization_msgs/MarkerArray)
- **용도**: 4가지 셀 상태를 컬러로 시각화
- **색상 코딩**:
  - 🔵 파란색: 정적 장애물 (static obstacles)
  - 🔴 빨간색: 동적 장애물 (dynamic obstacles)  
  - ⚪ 흰색: 자유 공간 (free space)
  - 🟫 갈색: 미지 영역 (unknown)
- **특징**:
  - 동적/정적 구분 가능
  - 더 직관적인 시각화
  - DOGM 알고리즘의 핵심 정보 표현

## RViz 설정 권장사항

1. **성능을 위해**: `/dogm/grid` 토픽 비활성화
2. **시각화를 위해**: `/dogm/cell_state` 토픽만 활성화
3. **Grid 형태**: 정사각형 3m x 3m, 0.3m 해상도 (10x10 셀)

## 성능 최적화

- 파티클 수: 200개 (실시간 성능)
- 업데이트 주기: 20Hz
- Grid 크기: 3m x 3m (작은 영역으로 성능 향상)
