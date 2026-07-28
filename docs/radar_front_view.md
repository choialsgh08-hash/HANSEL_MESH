# IWRL6432 전방 레이더 조종 화면

이 화면은 점 몇 개만 그리는 산점도 대신, 기본값으로 로봇 중심 3D 반구형 좌표 맵을 사용해
다음 정보를 한 화면에 합성한다.

1. TI range-azimuth heatmap(TLV 304/305)의 반사 강도
2. 방위각별 최근접 강한 반사를 연결한 거리-방위 추정 윤곽(높이 없음)
3. 최신 프레임의 선명한 실측 `x/y/z` 점, 바닥 접점, 높이선과 거리/높이 라벨
4. 근접 실측점끼리만 잇는 점선 보조 윤곽
5. 전방을 다섯 구역으로 나눈 근접 반사 경고

IWRL6432BOOST는 3Rx와 2Tx로 6개 가상 안테나 쌍을 만들며, 보드 문서가 명시하는
명목 각도 분해능은 방위각 약 29°, 고도각 약 58°다. 따라서 point-cloud TLV의
`x/y/z`는 실측 3D 좌표로 표시할 수 있지만 높이는 매우 거칠다. 반면 TLV 304/305는
range-azimuth 2D 강도 격자이므로 높이 정보를 갖지 않는다. 3D 화면은 이 격자를
높이 0인 기준면에 투영하고, 그 위에 최신 프레임의 실측 `x/y/z` 점만 선명한 마름모와
높이선으로 겹친다. 점에는 시간 누적을 적용하지 않는다. 강도면을 벽이나 바닥의 실제 높이라고
해석하면 안 된다.

프레임 사이의 깜빡임만 줄일 정도로 0.2초를 짧게 누적하며 긴 잔상은 만들지 않는다. 광학 영상은 아니다. 물체의 색, 문자,
모양, 표면 질감은 볼 수 없다. 반사가 없는 곳도 빈 통로가 아니라 **미확인**이다.

## 화면만 먼저 확인

저장소 루트에서 실행한다.

```powershell
python monitor/radar_front.py --demo
```

브라우저에서 `http://127.0.0.1:8081`을 연다. 데모는 실제 heatmap과 같은
`log-u8` 격자를 합성하므로 다음 기능을 하드웨어 없이 확인할 수 있다.

- 3 m 기본 근거리 확대 / 5 m 전환
- 3D 포인트 맵 / 3D 깊이 카메라 / 2D 위보기 전환
- 0.2초 / 0.4초 / 0.8초 단기 누적
- 다섯 전방 구역의 `미확인`, `반사`, `근접 반사` 표시
- 원시 포인트 겹쳐 보기
- 프레임이 오래되거나 끊겼을 때 정지 오버레이

## 실제 보드 데이터 연결

화면 프로세스는 UART를 열지 않는다. `radar-live` 한 프로세스만 보드 UART를 소유하고
mission JSONL을 기록하며, 화면은 그 파일을 따라간다.

### 1. TI 프로필에서 heatmap 출력 활성화

사용하는 IWRL6432 애플리케이션과 cfg가 range-azimuth heatmap TLV 304 또는 305를
실제로 출력해야 한다. 현재 준비한 기본 프로필은 10 Hz, azimuth 16 bins, range
128 bins, range step `0.09765625 m`다. 처리 여유가 부족할 때만 5 Hz, azimuth
32 bins 프로필로 바꾼다. 캡처 옵션은 실제로 보드에 전송한 프로필과 반드시 일치해야
한다.

### 2. 보드 리셋 후 cfg 적용

보드의 RESET 버튼을 눌러 CLI를 115200 baud 초기 상태로 되돌린 뒤 10 Hz 기본 cfg를
적용한다.

```powershell
python scripts/configure_ti_radar.py `
  --port COM3 `
  --cfg configs/radar/iwrl6432_heatmap_10hz.cfg
```

5 Hz fallback을 시험할 때는 RESET을 다시 누르고 cfg 파일만
`configs/radar/iwrl6432_heatmap_5hz.cfg`로 바꾼다. 결과 JSON에서
`new_baud_prompt_observed`가 `true`인지 확인한다. cfg 적용 뒤에는 보드를 다시
리셋하지 않는다.

실측 높이 점을 더 많이 얻는 실험에는
`configs/radar/iwrl6432_3d_operator_10hz.cfg`를 사용할 수 있다. 이 프로필은
elevation FFT를 4에서 8로 늘리고 검증된 CFAR threshold 15 dB는 유지한다.
높이 샘플 격자는 촘촘해질 수 있지만 물리 고도각 분해능은 약 58° 그대로다.
반드시 정지 상태의 거리/높이 표적 시험을 통과한 뒤 사용한다. elevation FFT 16은
현재 10 Hz 프로필에서 시작 직후 스트림이 멈춰 사용하지 않는다.

### 3. 캡처 시작

10 Hz 기본 프로필은 다음처럼 캡처한다. `COM3`은 실제 포트로 바꾼다.

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
  --profile-id iwrl6432-heatmap-profile `
  --calibration-id uncalibrated
```

5 Hz fallback 프로필을 보드에 적용했다면 위 명령의
`--heatmap-azimuth-bins 16`만 `--heatmap-azimuth-bins 32`로 바꾼다. 세 heatmap
옵션은 반드시 함께 지정한다. 출력 요약에서 다음을 확인한다.

- `heatmap_frames`가 계속 증가한다.
- `missing_heatmap_frames`가 0이다.
- `heatmap_cells_decoded`가 증가한다.
- `major_heatmap_frames` 또는 `minor_heatmap_frames`가 증가한다.

heatmap TLV가 없으면 화면은 최근 포인트를 부드럽게 누적한 `POINT EVIDENCE` 모드로
동작한다. 이 모드는 산점도보다 읽기 쉽지만, 실제 `RAW HEATMAP`보다 정보량이 많아지는
것은 아니다.

실제 heatmap의 azimuth bin은 선형 각도가 아니다. FFT-shift된 bin `i`를
`asin(2 × (i - N/2) / N)`으로 각도에 투영하고 ±70°만 표시한다. 0.25 m 안쪽은
블라인드존, 7.5 m 바깥은 이 조종 화면의 유효 범위 밖으로 취급한다.

### 4. 화면 시작

별도 PowerShell에서 실행한다.

```powershell
python monitor/radar_front.py `
  --follow missions\radar-board-live.jsonl `
  --history-window 0.2
```

상단 입력 모드는 다음 중 하나로 표시된다.

- `RAW HEATMAP + 3D`: 보드의 range-azimuth 강도면과 실측 `x/y/z` 점군 사용
- `POINT EVIDENCE`: 포인트만 시간 누적
- `HEATMAP 대기`: 아직 표시할 반사 데이터 없음

저장된 기록은 다음처럼 재생한다.

```powershell
python monitor/radar_front.py `
  --replay missions\radar-board-live.jsonl `
  --speed 1 `
  --loop
```

## 조종 화면 읽는 법

- 3D 포인트 맵의 바닥 격자와 거리 링: 레이더 기준 실제 전방 거리와 좌우 위치
- 작은 마름모: 최신 프레임에서 측정된 실측 `x/y/z` 반사점(점 잔상 없음)
- 마름모의 세로 연결선: 기준면에서 실측 높이까지의 높이
- 마름모 아래 작은 원: 해당 포인트의 바닥 기준 위치
- 겹치지 않게 표시되는 최대 5개 `거리 · z높이` 라벨
- 굵은 실선: heatmap에서 방위각별 최근접 강한 반사를 이은 추정 윤곽(높이 미측정)
- 바닥의 청록 실선: 거리가 비슷한 최신 실측점의 바닥 접점을 방위각 순서로 연결
- 가는 점선: 서로 가까운 최신 실측 3D 점의 보조 연결이며 실제 물체 경계로 확정할 수 없음
- 점의 화면 크기: 같은 반사 강도에서는 가까운 점이 크게, 먼 점이 작게 보이는 원근 표현
- 세로광: range-azimuth heatmap의 반사 방향이며 높이는 측정되지 않음
- 3D 공간 원근 보기의 색 면: 거리-좌우 반사 강도이며 높이는 측정되지 않음
- 3D 깊이 카메라의 좌우/위아래: point-cloud TLV의 실측 방위각/고도각
- 청록: 약하거나 중간 정도의 반사 증거
- 노랑: 강한 반사 증거
- 빨강: 0.15 m 이하의 최신 실측 포인트 또는 즉시 정지 상태
- 주황: 0.15 m 초과 0.30 m 이하의 최신 실측 포인트
- 다섯 구역의 `미확인`: 안전하거나 비어 있다는 뜻이 아님
- 다섯 구역의 거리: 그 구역에서 최근 누적된 가장 가까운 반사

다섯 구역은 경로 계획기가 아니라 사람이 화면을 빠르게 읽기 위한 보조 표시다.
구역을 녹색 또는 `FREE`로 표시하지 않는다.

근거리 프로필은 SDK의 근거리 예시와 동일하게 `rangeSelCfg` 최소값을 0.07 m로 설정한다.
현재 range bin 간격은 약 0.098 m이므로 첫 유효 셀은 약 0.10 m다. 0.15 m 이하의 최신
point-cloud 실측점만 빨강으로 분류하고, heatmap 단독 반사는 가까워도 빨강 판정에 사용하지
않아 직접 누설로 인한 상시 경고를 줄인다. 7 cm 미만과 안테나 직접 누설 구간은 신뢰할 수
없으므로 최종 충돌 정지는 범퍼 또는 별도 근접센서로 보완해야 한다.

## 안전 상태

- `WAITING`: 첫 프레임이 오기 전이므로 주행 금지
- `STALE`: 마지막 정상 프레임이 0.75초보다 오래됨
- `FAULT`: 마지막 정상 프레임이 2초보다 오래됐거나 HTTP 연결 끊김
- `DEGRADED`: 프레임 누락, 불완전 데이터 또는 파싱 이상
- `REPLAY END`: 마지막 기록 화면이 고정됨

`WAITING`, `STALE`, `FAULT`, `REPLAY END`에서는 이전 강도 영상을 회색으로 낮추고
정지 오버레이로 화면을 가린다. 동결된 장애물 흔적을 현재 정보로 오인하지 않게 하기
위해서다.

## 좌표와 장착 검증

IWRL6432 demo의 기본 좌표는 `+Y` 전방, `+X` 우측이다. 보드를 뒤집거나 회전해
장착했다면 실행 옵션으로 축과 부호를 바꾼다.

```powershell
python monitor/radar_front.py --demo `
  --forward-axis y --forward-sign -1 `
  --lateral-axis x --lateral-sign -1
```

장착 후에는 0.5 m, 1 m, 2 m 거리와 좌/중/우 위치에 큰 반사판을 놓고 거리와 방향을
검증한다. 보드 기준점, 자의 시작 위치, 레이더 range bin 양자화가 다르므로 자로 잰
외곽 거리와 화면의 최근점은 몇 cm 차이 날 수 있다. 보정 전에는
`calibration_id=uncalibrated`를 유지한다.

## 실제 운용 전에 확인할 항목

1. 정면 0.5/1/2/3 m 반사판 거리가 일관되게 증가하는지 확인한다.
2. 좌·중·우 반사판이 올바른 구역에 표시되는지 확인한다.
3. 벽과 큰 장애물이 포인트보다 heatmap 면으로 안정적으로 보이는지 확인한다.
4. 케이블 분리 시 0.75초 안에 `STALE`, 2초 안에 `FAULT`가 표시되는지 확인한다.
5. 반사가 사라져도 구역이 녹색이 아닌 `미확인`으로 남는지 확인한다.
6. 10분 이상 실행해 프레임률, 누락 수, 메모리 사용량을 확인한다.

이 화면만으로 충돌 가능 여부를 확정하면 안 된다. 실제 주행에는 카메라, 운전자 판단,
저속 제한, 비상 정지 절차를 함께 사용한다.
