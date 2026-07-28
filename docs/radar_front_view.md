# IWRL6432BOOST R9 레이더 LiDAR형 조종 화면

R9 화면은 카메라 영상을 합성하지 않는다. IWRL6432BOOST가 실제로 측정한 포인트와
range-azimuth heatmap의 반사 증거를 로봇 기준 LiDAR형 탑뷰로 표시한다.

- 주 화면: 전방 `0~3m`, 좌우 `-1.5~+1.5m`
- 충돌 확대창: 전방 `0~50cm`
- 격자 해상도: 5cm
- 포인트와 점유 증거 유지 시간: 최대 300ms
- 거리 기반 빨강 경고: 고정 클러터가 아닌 확인된 포인트가 `10cm` 이하일 때만 표시

주 화면은 현재 로봇을 원점으로 한 순간적인 반사 증거다. 세계 좌표에 누적된 지도나
SLAM 결과가 아니다. 로봇이 움직여도 과거 구조를 지도에 남기지 않으므로, 화면에
없는 영역과 반사가 없는 영역은 `UNKNOWN`이다. `UNKNOWN`은 빈 공간, 안전 통로,
`FREE`를 뜻하지 않는다.

## 1. 화면만 먼저 확인

저장소 루트에서 다음 명령을 실행한다.

```powershell
python monitor\radar_front.py --demo --bind 127.0.0.1 `
  --http-port 8081 --max-range-m 3 --history-window 0.3
```

브라우저에서 `http://127.0.0.1:8081/`을 연다. 상단의 `UI R9`,
`0~3m FORWARD MAP`, `ROBOT RELATIVE`와 우측의 `0~50cm 충돌 확대`가 보여야 한다.
데모 데이터는 합성 데이터이므로 실제 보드 감지 성능을 증명하지 않는다.

## 2. 보드 프로필 적용

화면 프로세스는 UART를 열지 않는다. `radar-live` 한 프로세스만 Application/User
UART를 소유하고 mission JSONL을 기록하며, 화면은 그 파일을 따라간다.

보드의 RESET 버튼을 한 번 누른 뒤, 실제 Application/User UART 번호에 맞게
`COM3`을 바꾸어 실행한다.

```powershell
python scripts\configure_ti_radar.py `
  --port COM3 `
  --cfg configs\radar\iwrl6432_3d_operator_near_10hz.cfg `
  --command-timeout 1.5
```

결과에서 `commands_completed=25`, `new_baud_prompt_observed=true`,
`first_magic_observed=true`를 확인한다. `new_baud_verified_by_version`은 새 baud의
프롬프트를 직접 확인하지 못해 fallback `version` probe를 사용한 경우에만
`true`다. 프롬프트를 직접 확인한 정상 경로에서는 이 값이 `false`여도 정상이다.
프로필 적용 후에는 다시 RESET하지 않는다. 이 프로필은 10Hz, 128 range bins,
16 azimuth bins, range step `0.09765625m`를 사용한다.

## 3. 빈 장면 캘리브레이션

고정 안테나 누설, 보드·케이블·장착물의 반복 반사를 실제 장애물에서 분리하려면
프로필별 빈 장면 캘리브레이션이 필요하다.

1. 보드와 케이블을 실제 로봇 장착 상태로 완전히 고정한다.
2. 레이더 전방 3m 안에 사람, 반사판, 상자 등 움직이거나 임시로 놓인 물체가 없는
   넓은 빈 공간을 준비한다.
3. 보드가 정지한 상태에서 최소 50개의 완전한 heatmap 프레임을 기록한다.

```powershell
python -m sensors radar-live `
  --port COM3 `
  --baud 1250000 `
  --allow-elided-empty-point-tlv `
  --allow-nonzero-padding `
  --heatmap-azimuth-bins 16 `
  --heatmap-range-bins 128 `
  --heatmap-range-step-m 0.09765625 `
  --output missions\radar-empty-scene.jsonl `
  --raw-output captures\radar-empty-scene.bin `
  --mission-id radar-empty-scene `
  --profile-id lsdk-05.05.04.02-presence-near-heatmap16-elev8-cfar15-10hz-v1 `
  --calibration-id uncalibrated `
  --duration 10
```

이미 같은 출력 파일이 있으면 먼저 별도 이름으로 보관하거나 새 파일명을 사용한다.
캡처가 끝난 뒤 다음 명령으로 클러터 모델을 만든다.

```powershell
New-Item -ItemType Directory -Force -Path configs\radar\calibrations

python -m sensors radar-calibrate missions\radar-empty-scene.jsonl `
  --output configs\radar\calibrations\head-near.json `
  --min-frames 50
```

캘리브레이션은 다음 항목에 묶인다.

- 캡처의 `profile_id`
- heatmap shape인 range/azimuth bin 수
- heatmap range step
- motion mode
- 전방·좌우 축과 부호

프로필, heatmap shape/range step 또는 axes가 달라지면 모델을 자동 적용하지 않고
`PROFILE MISMATCH`로 차단한다. 보드 방향, 프로필, 케이블 또는 레이더 주변 고정
장착물이 바뀌면 빈 장면을 다시 기록해 캘리브레이션한다. 이 클러터 모델의
`calibration_id`는 `SensorHeader.calibration_id`와 별도다.

기본 축은 TI 좌표의 `+Y` 전방, `+X` 우측이다. 축을 바꾸어 장착했다면
`radar-calibrate`와 `radar_front.py` 양쪽에 동일한 `--forward-axis`,
`--forward-sign`, `--lateral-axis`, `--lateral-sign` 값을 사용해야 한다.

## 4. 실제 운용 데이터 캡처

캘리브레이션 때와 같은 프로필·heatmap 설정으로 실제 장면을 기록한다. 목표 출력
파일이 이미 있으면 보관한 뒤 실행한다.

```powershell
python -m sensors radar-live `
  --port COM3 `
  --baud 1250000 `
  --allow-elided-empty-point-tlv `
  --allow-nonzero-padding `
  --heatmap-azimuth-bins 16 `
  --heatmap-range-bins 128 `
  --heatmap-range-step-m 0.09765625 `
  --output missions\radar-board-live.jsonl `
  --raw-output captures\radar-board-live.bin `
  --mission-id radar-board-live `
  --profile-id lsdk-05.05.04.02-presence-near-heatmap16-elev8-cfar15-10hz-v1 `
  --calibration-id uncalibrated
```

이 PowerShell은 캡처가 끝날 때까지 닫지 않는다. 출력 요약의
`heatmap_frames`와 `heatmap_cells_decoded`가 증가하고
`missing_heatmap_frames`가 계속 증가하지 않는지 확인한다.

## 5. R9 조종 화면 실행

별도 PowerShell에서 다음 명령을 그대로 실행한다.

```powershell
python monitor\radar_front.py `
  --follow missions\radar-board-live.jsonl `
  --clutter-calibration configs\radar\calibrations\head-near.json `
  --max-range-m 3 `
  --history-window 0.3
```

브라우저에서 `http://127.0.0.1:8081/`을 열고, 이전 화면이 캐시에 남으면
`Ctrl+F5`로 새로 고친다.

저장된 기록은 같은 캘리브레이션을 지정해 재생한다.

```powershell
python monitor\radar_front.py `
  --replay missions\radar-board-live.jsonl `
  --clutter-calibration configs\radar\calibrations\head-near.json `
  --max-range-m 3 `
  --history-window 0.3 `
  --speed 1
```

## 6. 화면 읽는 법

### 0~3m 주 지도

- 화면 아래 중앙이 로봇과 레이더의 현재 원점이다.
- 0.5m 간격 반원은 로봇으로부터의 전방 거리다.
- 청록·흰색 셀은 점유 또는 반사 증거다. 어두운 배경은 `UNKNOWN`이다.
- 테두리가 선명한 마커는 point-cloud의 실제 `x/y/z` 포인트다.
- 짧은 호는 heatmap의 거리·각도 불확실성을 포함한 반사 증거이며 높이는 없다.
- `z` 라벨은 point cloud가 실제 높이를 제공한 경우에만 표시한다.
- 최대 5개의 가까운 track에 거리 라벨을 표시한다.
- track은 마지막 관측 후 정확히 300ms가 되면 제거된다.

이 화면은 포인트 사이를 임의의 벽이나 표면으로 연결하지 않는다. 희소 포인트 때문에
광학 카메라나 2D LiDAR처럼 연속 윤곽이 보이지 않을 수 있지만, 측정하지 않은 구조를
만들어 통로로 오인하는 것보다 안전한 표현이다.

### 0~50cm 충돌 확대

50cm는 주 화면의 최대 표시 거리가 아니라 같은 장면의 근거리 확대창이다. 주 화면은
계속 3m까지 표시한다. 확대창의 10/20/30/40/50cm 반원은 정확한 거리 기준이다.

빨강은 다음 조건을 모두 만족할 때만 나타난다.

1. 고정 클러터로 제거되지 않은 point-cloud 포인트다.
2. 최근 3개 프레임 중 최소 2개 프레임에서 같은 위치로 확인됐다.
3. 확인된 track의 수평거리가 `10cm` 이하다.

위험 track이 13cm 이상으로 멀어지거나 300ms 동안 다시 관측되지 않으면 빨강을
해제한다. 70cm나 1m의 반사는 점유 증거로 보이지만 충돌 빨강이 아니다.
heatmap의 가까운 bin만으로도 빨강을 만들지 않는다.

`NORMAL`은 “10cm 이내 확인 포인트가 없음”만 뜻한다. 장애물이 없거나 경로가
안전하다는 뜻이 아니다. 반사가 없거나 센서가 놓친 공간도 `UNKNOWN`으로 남는다.

## 7. 차단 상태와 안전 한계

- `CALIBRATION REQUIRED`: 클러터 모델이 없어 두 지도를 신뢰할 수 없음
- `PROFILE MISMATCH`: 프로필, heatmap 형상/range step 또는 axes가 모델과 다름
- `WAITING`: 첫 완전 프레임을 기다리는 중
- `STALE`: 마지막 정상 프레임 후 0.75초 이상 지남
- `FAULT`: 마지막 정상 프레임 후 2초 이상 지났거나 소스 오류
- `REPLAY END`: 기록 재생이 끝남

이 상태에서는 이전 점유를 현재 정보처럼 남기지 않고 두 지도를 차단 오버레이로
가린다. missing return, heatmap의 빈 셀, 가까운 bin 미검출은 절대로 안전 판정이
아니다. 화면은 모터 정지 명령을 보내지 않는다.

현재 heatmap range step은 약 9.8cm이고 10cm 안쪽은 안테나 직접 누설과 겹친다.
따라서 신뢰 가능한 1cm 거리 분리는 보장할 수 없다. 최종 로봇에는 범퍼 또는 별도
근접센서, 저속 제한, 비상 정지 절차가 필요하다.

## 8. 실측 검증

빈 장면 캘리브레이션 후 큰 금속 반사판을 사용해 다음 순서로 기록한다.

1. 정면 0.2/0.3/0.5/1/2/3m에서 거리 라벨이 일관되게 변하는지 확인한다.
2. 좌·중·우 위치가 지도에서 올바른 방향에 나타나는지 확인한다.
3. 0.70m 반사판이 빨강이 되지 않는지 확인한다.
4. 20cm부터 5cm까지 2cm 간격으로 접근해, 확인 포인트가 10cm 이하일 때만
   빨강이 되고 13cm 이상에서 해제되는지 확인한다.
5. 케이블을 분리해 `STALE`, `FAULT` 차단 오버레이가 나타나는지 확인한다.
6. 반사가 사라져도 녹색이나 `FREE`가 아닌 `UNKNOWN`으로 남는지 확인한다.
7. 레이더와 화면을 함께 녹화하고 mission JSONL과 raw capture를 보존한다.

광학 자의 물체 외곽이 아니라 레이더 안테나 기준점에서 거리를 잰다. point-cloud와
heatmap 거리 양자화 때문에 몇 cm 차이가 날 수 있다.

## 9. 다음 단계

현재 R9는 IMU 없이 동작하는 로봇 기준 즉시뷰다. 다음 단계에서 엔코더와 IMU의
timestamp·좌표계를 맞추고 radar-to-base 외부 보정을 적용해 이동량을 보상한다.
그 다음에만 여러 프레임을 세계 좌표에 누적하는 rolling occupancy map,
Doppler odometry, radar scan-to-submap 정합과 SLAM을 추가할 수 있다. 유닛 분리
위치는 그때의 pose와 불확실성을 함께 저장해야 한다.
