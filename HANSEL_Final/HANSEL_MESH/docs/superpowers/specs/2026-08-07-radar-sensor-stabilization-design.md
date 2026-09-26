# HANSEL 레이더 센서 안정화 및 확률 주행 지도 설계

**작성일:** 2026-08-07
**대상 기준:** `main@433e713d81eb6e2dff77427bce08c44e0d61e64c`

## 1. 목표

IWRL6432BOOST의 희소한 point cloud와 range-azimuth heatmap을 이용해 다음 세 기능을 만든다.

1. 안정적인 2D/2.5D 확률 장애물 지도
2. 로봇 폭을 반영한 안전한 단거리 경로
3. 카메라가 먼지로 가려졌을 때 사용할 레이더 주행 보조 화면

이번 단계의 우선순위는 센서 정보 안정화다. Doppler odometry, IMU/엔코더 융합,
전역 SLAM은 센서 출력과 로컬 지도가 충분히 안정화된 뒤 별도 단계에서 구현한다.

## 2. 제약과 안전 원칙

- 레이더 관련 Python, JavaScript, 설정, 테스트, 문서만 변경한다.
- 현재 검증된 8 Hz 센서 프로필과 UART 복구 동작은 유지한다.
- `DANGER` 진입 0.10 m, 해제 0.13 m 기준은 유지한다.
- 정지한 벽과 잔해를 보존하기 위해 `clutterRemoval 0`을 유지한다.
- 관측되지 않은 공간은 `FREE`가 아니라 `UNKNOWN`이다.
- 필터는 측정값을 안정화할 뿐 센서가 측정하지 않은 윤곽이나 높이를 생성하지 않는다.
- 경로 출력은 운전자 보조 정보이며 모터 자동 제어 권한을 갖지 않는다.
- stale, fault, calibration mismatch에서는 기존 fail-closed 정지 오버레이를 유지한다.

## 3. 현재 증거와 문제 정의

동일한 8 Hz 프로필의 장시간 기록에는 90,284개의 완전한 레이더 프레임과
529,879개의 포인트가 있다. 프레임당 포인트 중앙값은 6개이고, 약 9.77 cm의
고정 self-clutter가 모든 프레임에 존재한다. 현재 clutter model은 이 점을 scene에서
제거하지만 3D 화면은 `frame.points`를 직접 사용하므로 제거 전 점을 다시 표시한다.

현재 host 처리의 주요 불안정 요인은 다음과 같다.

- 3D 패널이 확정 scene이 아니라 raw point를 표시한다.
- 5 mm rolling cell은 2회 hit 후 TTL이나 confidence decay 없이 남는다.
- 현재 프로필의 heatmap 거리 간격 9.765625 cm보다 지나치게 정밀한 지도를 사용한다.
- radar-only ego-motion이 미확정 point를 최소 3개만 사용해 지도 전체를 변환할 수 있다.
- point 위치는 새 측정으로 즉시 덮어쓰며 robust position filter가 없다.
- 경로는 hold와 switch confirmation 없이 매 프레임 바뀐다.
- path/occupancy가 point의 높이를 버리므로 화면의 높이 정보와 경로 판단이 분리돼 있다.
- 기존 clutter calibration은 프로필 일치만 검증하고 장착 방향, 케이블, 로봇 몸체,
  보드 일련번호, 온도 또는 촬영 장소 변경을 자동 검출하지 못한다.

## 4. 검토한 접근법

### 접근 A: 화면에만 smoothing 적용

raw point에 잔상, Gaussian blur 또는 단순 EMA를 적용한다.

- 장점: 구현이 가장 빠르고 화면이 덜 흔들려 보인다.
- 단점: ghost를 실제 장애물처럼 부드럽게 만들고 경로 지도 신뢰도는 개선하지 못한다.
- 결정: 채택하지 않는다.

### 접근 B: host 확률 지도와 확정 evidence 중심 처리

raw point와 heatmap을 품질 판정, 시간 확인, robust 위치 추정, 확률 occupancy로 변환하고
경로와 주행 화면이 동일한 filtered scene을 사용한다.

- 장점: 현재 하드웨어와 8 Hz 프로필을 유지하면서 화면과 경로의 원인을 함께 개선한다.
- 단점: IMU/엔코더가 연결되기 전에는 장시간 이동 누적 지도가 아니라 짧은 로컬 지도다.
- 결정: 이번 단계에서 채택한다.

### 접근 C: TI on-device tracker 및 profile 재튜닝 우선

TI Group Tracker와 새로운 CFAR/profile을 먼저 적용한다.

- 장점: 센서 가까운 단계에서 point 수와 object track을 조절할 수 있다.
- 단점: 정적 벽 지도, raw UI 우회, 영구 rolling map 문제는 그대로 남고 새 profile마다
  실측 calibration이 필요하다.
- 결정: host 안정화와 계측이 완료된 뒤 실측 A/B 단계에서 검토한다.

## 5. 전체 데이터 흐름

```text
TI UART point cloud + range-azimuth heatmap
  -> profile-bound calibration 검사
  -> robot/self-clutter mask
  -> point/heatmap 품질 점수와 진단 카운터
  -> 거리-각도 기반 association
  -> 최근 5프레임 중 3프레임 temporal confirmation
  -> robust track 위치와 불확실성
  -> 5 cm robot-relative probabilistic occupancy
  -> 시간 decay와 최대 수명 적용
  -> 로봇 폭/안전 여유만큼 costmap inflation
  -> 경로 충돌 검사와 경로 전환 hysteresis
  -> 동일 filtered scene을 2D/2.5D/주행 화면에서 표시
```

Raw point는 기록과 디버그에는 보존하지만 경로와 기본 운전자 화면의 장애물로 직접
사용하지 않는다.

## 6. 측정 evidence와 track 안정화

### 6.1 Point evidence

Point는 다음 조건을 거친다.

- 기존 scene range/FOV 검사
- profile-bound self-clutter point cluster 제거
- 유한한 x/y/z/range 검사
- SNR 유무, SNR 값, noise 값, range를 진단 정보에 보존
- 동일 프레임의 한 point가 둘 이상의 track을 갱신하지 못하도록 one-to-one association

전역 고정 거리 18 cm nearest-neighbour 대신 range와 azimuth의 불확실성을 분리한다.

- range gate: `max(0.10 m, 1.25 * heatmap_range_step_m)`
- azimuth gate: 현재 보드의 거친 각도 분해능을 반영한 거리 비례 lateral gate
- 이미 현재 프레임에서 갱신된 track은 association 후보에서 제외

`heatmap_range_step_m`은 해당 프레임 메타데이터를 우선 사용하고, 메타데이터가 없을 때만
현재 profile의 검증된 0.09765625 m 값을 사용한다. 둘 다 알 수 없으면 새 track을 확정하지
않고 진단 카운터를 증가시킨다.

Point track의 위치는 새 좌표로 덮어쓰지 않는다. 최근 5개 연관 측정의 좌표별 median을
사용하고, median에서 크게 벗어난 측정은 위치 갱신에 사용하지 않는다. SNR은 obstacle
확정의 유일한 조건이 아니라 품질 점수의 한 요소로만 사용한다.

### 6.2 Temporal confirmation

일반 지도 evidence는 최근 5프레임 중 최소 3프레임에서 공간적으로 일치해야 확정된다.
8 Hz에서 연속 3회 확인은 약 0.375초이며 느린 로봇의 운전자 보조 화면에 허용 가능한
지연이다.

10 cm 충돌 경고는 기존 2-of-3 confirmed point 조건과 10/13 cm hysteresis를 유지한다.
경로 지도 안정화 때문에 충돌 경고가 늦어지지 않도록 hazard와 map confirmation을
서로 분리한다.

### 6.3 Heatmap evidence

기존 median/MAD residual, connected component, point correlation을 유지한다. Heatmap-only
반사는 높이를 만들지 않으며 point correlation과 시간 지속성이 없는 cluster는
`reflection_evidence`로만 표시한다.

## 7. 확률 occupancy 지도

새 `ProbabilisticRollingGrid`는 scene 추적과 경로 계획 사이의 독립 구성요소다.

### 7.1 지도 형상

- robot-centered 반경: 0.50 m
- cell 크기: 0.05 m
- 좌표: 현재와 동일한 robot-relative forward/lateral
- 기본 확률: 0.5 (`UNKNOWN`)
- occupied 출력 임계값: 0.75
- map 투입 조건: 3-of-5 confirmed strong evidence

5 cm는 센서의 9.77 cm range cell보다 작지만 경로용 discretization이다. 측정 정확도로
표현하지 않으며 각 cell에는 confidence와 range/lateral uncertainty를 함께 제공한다.

### 7.2 누적과 decay

- confirmed hit마다 log-odds를 증가시킨다.
- radar miss는 자유 공간 증거로 사용하지 않는다.
- 마지막 hit 이후 log-odds를 1.0초 half-life로 0에 수렴시킨다.
- 마지막 hit 이후 2.5초가 지나면 cell을 제거한다.
- probability와 hit count의 상한을 두어 오래된 cell이 새 evidence를 영구 지배하지 못하게 한다.
- map reset, producer change, profile/calibration mismatch, sensor fault 시 즉시 비운다.

### 7.3 Radar-only motion 적용 제한

IMU/엔코더 전 단계에서는 radar-only transform을 완전히 신뢰하지 않는다.

- motion 후보는 raw point가 아니라 strong confirmed point track만 사용한다.
- 5개 미만의 일치점, match ratio 0.70 미만 또는 RMSE 0.05 m 초과이면 map을 변환하지 않는다.
- 승인되지 않은 motion에서는 map을 현재 robot-relative 좌표에 고정하고 정상 decay만 수행한다.
- map payload는 `pose_mode: robot_relative_short_memory`로 장기 SLAM 지도가 아님을 명시한다.

## 8. 2.5D 표현

Point cloud의 실제 z는 filtered point track에 보존한다. Heatmap track은 높이가 없으므로
`height_m: null`을 유지한다.

IMU 자세와 radar-to-base extrinsic이 아직 없으므로 높이는 `sensor_relative_uncalibrated`다.
화면은 다음 세 시각 band만 사용한다.

- `below_sensor`: z < -0.10 m
- `sensor_level`: -0.10 m <= z <= 0.10 m
- `above_sensor`: z > 0.10 m
- `unknown`: heatmap-only 또는 높이 없음

이 band는 운전자에게 반사점의 상대 높이를 알려 주기 위한 것이며 통과 가능 높이를
판단하지 않는다. 이번 단계의 경로 계획은 안전하게 모든 confirmed band를 2D 장애물로
투영한다. IMU와 extrinsic calibration이 연결된 뒤 로봇 실제 높이와 clearance를 사용한다.

## 9. 안전 경로

경로 계획 입력은 probabilistic occupancy의 occupied cell과 짧은 수명의 confirmed reactive
evidence뿐이다. tentative/raw point는 기본 경로를 막지 않는다.

- 로봇 폭: 0.15 m
- 좌우 안전 여유: 각 0.03 m
- 최소 통과 폭: 0.21 m
- 기존 0.60 m local planning 범위와 curved lattice planner 유지
- obstacle uncertainty를 inflation 반경에 추가
- `UNKNOWN`을 free ray로 만들지 않음

경로 전환은 다음 hysteresis를 사용한다.

- 현재 경로가 새 confirmed reactive evidence 또는 occupied cell과 충돌하면 즉시 재계획
- 충돌하지 않으면 새 curvature가 3프레임 연속 선택돼야 전환
- 후보가 사라지면 pending switch를 초기화
- 같은 curvature는 selected duration을 유지
- UI의 1 cm 곡선 sampling은 표시 해상도이며 센서 정확도가 아님을 API에 명시

## 10. 운전자 화면

### 10.1 기본 2D 지도

- 5 cm occupancy cell을 probability에 따라 투명도와 색으로 표시
- confirmed point, confirmed heatmap, tentative reflection을 서로 다른 기호로 표시
- uncertainty는 점을 흐리게 키우는 대신 얇은 범위/각도 외곽선으로 표시
- 로봇 폭과 안전 여유, 현재 선택 경로, blocked 구간을 함께 표시
- 오래된 cell은 probability decay에 따라 자연스럽게 약해지고 2.5초 후 사라짐

### 10.2 2.5D 보조 패널

- raw `frame.points` 대신 filtered point track만 표시
- relative height band별 색상과 실제 거리 label 표시
- heatmap-only obstacle은 `Z UNKNOWN`으로 표시
- raw point 표시는 기본 OFF인 `RAW DEBUG` 옵션에서만 제공
- 필터가 숨긴 point 수와 confirmed obstacle 수를 상태 패널에 표시

`RAW DEBUG`는 표시 전용이며 켜더라도 확률 지도, 충돌 판단, 경로 선택에는 영향을 주지 않는다.

### 10.3 안전 상태

- `DANGER`: 기존 confirmed point 0.10 m 이하
- `NORMAL`: 0.10 m 이하 confirmed point 없음
- `UNKNOWN`: 신뢰 가능한 scene evidence 없음
- `SENSOR_FAULT`: stale/fault/profile/calibration 실패

`NORMAL`은 전체 통로가 안전하다는 의미가 아니며 화면에 해당 문구를 유지한다.

## 11. API와 진단

기존 `/api/radar` 소비자를 깨지 않도록 현재 필드는 유지하고 다음 필드를 추가한다.

- `scene.pose_mode = "robot_relative_short_memory"`
- `scene.probabilistic_local_map`
  - `resolution_m`, `radius_m`, `occupied_threshold`, `decay_half_life_ms`, `ttl_ms`
  - 각 cell의 `forward_m`, `lateral_m`, `probability`, `confidence`, `hits`, `age_ms`
- filtered point track의 `height_m`, `height_band`, `height_reference`
- `scene.diagnostics`
  - raw point 수
  - self-clutter 제거 수
  - quality gate 제거 수
  - tentative/confirmed point track 수
  - heatmap reflection/confirmed 수
  - probability threshold 통과 cell 수
  - decay/TTL로 제거된 cell 수
  - path switch pending/accepted 수

기존 `rolling_local_map`은 한 릴리스 동안 동일한 occupied cell을 제공하는 호환 alias로
남기고 retention 의미를 `probability_decay_with_ttl`로 변경한다. 브라우저가 새 map을
지원하면 새 필드를 우선 사용한다.

## 12. Calibration과 센서 설정

이번 구현에서는 검증된 8 Hz cfg 값을 변경하지 않는다. CFAR와 sidelobe threshold는
host 단계별 진단이 추가된 뒤 동일 장면 A/B 실측으로만 조정한다.

현재 포함된 clutter model은 임시 검증용이다. 실제 로봇에는 다음 조건으로 새 model을
생성한다.

- 최종 레이더 장착 방향, 케이스, 케이블, 나사, 배터리 상태
- 로봇과 레이더 모두 고정
- 전방에 이동 표적이 없는 상태
- 하나의 방 배경을 지도처럼 외우지 않도록 여러 방향/배경에서 공통인 near self-clutter 확인
- profile ID, 보드 식별자, 장착 ID, 생성 시각을 provenance로 기록

TI range bias와 RX channel phase 측정은 host clutter model과 별개의 보정이다. 측정 전에는
현재 기본 coefficient를 보드별 보정값으로 간주하지 않는다.

## 13. 제외 범위

- Doppler odometry와 Doppler hard gate
- IMU/엔코더 드라이버 및 pose EKF
- radar-to-base 높이·pitch·roll extrinsic calibration
- 전역 occupancy, loop closure, unit drop 위치 기록
- 모터 자동 조향 또는 비상 정지 명령
- 카메라 영상 융합
- 새로운 chirp, frame rate 또는 TI firmware 적용

현재 저장 로그의 Doppler가 모두 0이므로 Doppler 기반 기능은 non-zero Doppler가 검증된
별도 profile/firmware 단계 전까지 사용하지 않는다.

## 14. 시험 전략

### 14.1 Python 단위·회귀 시험

- 2회만 나타난 ghost는 occupied cell이 되지 않는다.
- 5프레임 중 3회 같은 위치에 나타난 point는 occupied cell이 된다.
- confirmed cell은 새 hit가 없으면 probability가 감소하고 2.5초 후 제거된다.
- 오래된 rolling cell은 경로 입력에서 사라진다.
- calibration에 일치하는 9.77 cm self-clutter는 filtered 2.5D point에 나타나지 않는다.
- point 위치가 한 프레임 튀어도 최근 median track 위치는 크게 이동하지 않는다.
- 불확실한 radar-only motion은 map을 변환하지 않는다.
- 확실한 motion만 short-memory map에 적용된다.
- 현재 경로가 막히지 않으면 다른 curvature가 3프레임 연속 선택된 뒤 전환된다.
- 현재 경로가 막히면 즉시 전환된다.
- 10/13 cm danger hysteresis는 변하지 않는다.
- calibration/profile/fault/reset이 probability map과 pending path를 비운다.

### 14.2 Browser 시험

- 기본 2.5D 패널은 raw frame point를 사용하지 않는다.
- raw debug는 기본 OFF이며 사용자가 켠 경우에만 표시된다.
- probability가 낮아지는 cell은 투명해지고 TTL 이후 제거된다.
- 높이 band와 `Z UNKNOWN` 표시가 API 의미와 일치한다.
- robot footprint, inflation, path가 동일 좌표계를 사용한다.

### 14.3 기록 replay

- 저장된 clutter fixture에서 self-clutter가 화면과 경로 양쪽에서 제거된다.
- 짧은 반복 ghost fixture가 영구 rolling obstacle을 만들지 않는다.
- 기존 안정 로그의 첫 2,400프레임에서 path switch와 map cell 수가 유한하게 유지된다.
- replay 결과는 raw/filtered/confirmed/map/path 단계별 카운터로 보고한다.

작은 synthetic/recorded fixture는 저장소에 포함해 CI 회귀 시험에 사용한다. 장시간 원본 로그는
용량과 개인정보 가능성 때문에 저장소에 넣지 않고, 존재할 때만 실행하는 로컬 검증으로 분리한다.

### 14.4 실측 검증

각 장면을 최소 60초 기록한다.

1. 최종 장착 상태의 빈 장면
2. 정면 0.30, 0.50, 1.00 m 반사판
3. 좌우 약 30도 같은 거리의 반사판
4. 나무와 금속 장애물을 같은 위치에 둔 장면
5. 벽을 향해 느리게 접근하고 멀어지는 장면

각 시험은 raw false point/frame, confirmed false obstacle/frame, 위치 표준편차,
목표 검출률, ghost 잔존 시간, path switch/min, 처리 지연, frame drop을 기록한다.

## 15. 합격 기준

1. 기본 주행 화면에는 raw point와 calibrated self-clutter가 나타나지 않는다.
2. 단발 또는 2회 ghost는 확률 지도와 정상 경로를 장기 차단하지 않는다.
3. confirmed obstacle이 사라지면 2.5초 이내에 local map에서 제거된다.
4. 고정 표적의 filtered 위치 jitter가 raw 위치 jitter보다 작다.
5. 현재 경로가 막히지 않은 상황에서 단일 프레임 노이즈로 경로가 전환되지 않는다.
6. 로봇 폭 0.15 m와 좌우 0.03 m 여유가 모든 path collision 검사에 적용된다.
7. 10 cm 위험 진입과 13 cm 해제 동작이 그대로 유지된다.
8. 높이가 보정되지 않은 상태에서 z 정보로 통과 가능 공간을 만들지 않는다.
9. 8 Hz replay와 live 입력에서 처리 시간이 frame period 125 ms를 넘지 않는다.
10. 레이더 scene, calibration, viewer, web 회귀 시험이 모두 통과한다.

## 16. 이후 단계 경계

이 설계가 실측 합격 기준을 만족한 뒤 다음 단계에서 IMU와 엔코더 timestamp 계약,
robot pose EKF, radar-to-base extrinsic, Doppler 유효성 검증, radar scan-to-submap 보정을
추가한다. 확률 지도 API는 pose 입력을 받을 수 있도록 분리하되 이번 구현에서는 해당
입력을 요구하지 않는다.
