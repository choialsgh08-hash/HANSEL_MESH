# IWRL6432BOOST R9 Windows 재연결

아래 명령은 저장소 루트에서 실행한다. 예시의 `COM3`은 Application/User UART이며,
장치 관리자에서 확인한 실제 포트로 바꾼다. 보드 UART는 한 프로세스만 소유해야 한다.

## 1. 기존 프로세스 종료와 보드 재설정

1. 캡처 PowerShell에서 `Ctrl+C`를 눌러 기존 `radar-live`를 종료한다.
2. 화면 서버가 실행 중이면 그 PowerShell도 `Ctrl+C`로 종료한다.
3. 보드의 RESET 버튼을 한 번 누른다.
4. 다음 명령으로 R9 근거리 프로필을 적용한다.

```powershell
python scripts\configure_ti_radar.py `
  --port COM3 `
  --cfg configs\radar\iwrl6432_3d_operator_near_10hz.cfg `
  --command-timeout 1.5
```

결과에서 `commands_completed=25`, `new_baud_verified_by_version=true`,
`first_magic_observed=true`를 확인한다. 프로필 적용 후에는 다시 RESET하지 않는다.

## 2. 캘리브레이션 파일 확인

```powershell
Test-Path configs\radar\calibrations\head-near.json
```

`True`이고 프로필, 보드 방향, heatmap shape/range step, axes, 케이블과 고정 장착물이
이전과 같으면 기존 파일을 사용할 수 있다. 다음 경우에는 빈 장면 캘리브레이션을
다시 해야 한다.

- `CALIBRATION REQUIRED` 또는 `PROFILE MISMATCH`가 표시됨
- cfg나 `profile_id`를 바꿈
- range/azimuth bin 수 또는 range step을 바꿈
- 레이더 장착 방향이나 axes 옵션을 바꿈
- 보드, 케이블 또는 근접 고정 장착물의 위치를 바꿈

전방 3m 안에 사람과 임시 물체가 없는 빈 공간에서 보드와 케이블을 고정하고 최소
50개의 완전한 프레임을 기록한다.

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

기존 출력 파일이 있으면 보관하거나 다른 이름을 사용한다. 캡처 후 실행한다.

```powershell
python -m sensors radar-calibrate missions\radar-empty-scene.jsonl `
  --output configs\radar\calibrations\head-near.json `
  --min-frames 50
```

기존 캘리브레이션 파일을 의도적으로 교체할 때만 마지막에 `--overwrite`를 추가한다.

## 3. 실제 레이더 캡처 시작

첫 번째 PowerShell에서 실행한다. `missions\radar-board-live.jsonl`과
`captures\radar-board-live.bin`이 이미 있으면 먼저 보관한다.

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

이 창은 캡처가 끝날 때까지 닫지 않는다.

## 4. R9 LiDAR형 화면 시작

두 번째 PowerShell에서 실행한다.

```powershell
python monitor\radar_front.py `
  --follow missions\radar-board-live.jsonl `
  --clutter-calibration configs\radar\calibrations\head-near.json `
  --max-range-m 3 `
  --history-window 0.3
```

브라우저에서 `http://127.0.0.1:8081/`을 연다. 기존 탭은 `Ctrl+F5`로 새로 고친다.
정상 화면에는 `UI R9`, `0~3m FORWARD MAP`, `ROBOT RELATIVE`,
`0~50cm 충돌 확대`가 보인다.

50cm는 전체 시야 제한이 아니라 근거리 확대창이다. 주 지도는 전방 3m까지 표시한다.
빨강은 고정 클러터가 아닌 확인 포인트가 10cm 이하일 때만 허용된다. heatmap의
가까운 bin과 missing return은 안전을 뜻하지 않으며, 어두운 영역은 `UNKNOWN`이다.

## 5. 다시 끊길 때

- `ClearCommError`가 나오면 두 프로세스를 종료하고 USB 케이블과 보드 전원을 확인한다.
- 가능하면 USB 허브를 거치지 말고 노트북 본체 포트와 데이터용 케이블을 사용한다.
- COM 포트가 다시 보이면 RESET 후 1단계부터 반복한다.
- 웹이 `WAITING`, `STALE`, `FAULT`, `REPLAY END`이면 화면이 남아 있어도 주행에
  사용하지 않는다.
- `PROFILE MISMATCH`이면 캘리브레이션과 실제 캡처의 `profile_id`,
  heatmap shape/range step, axes가 같은지 확인한다.
- `CALIBRATION REQUIRED`이면 `--clutter-calibration` 경로와 파일을 확인한다.
- HTTP 8081 포트가 사용 중이면 이전 화면 서버를 종료한 뒤 다시 실행한다.

## 6. 근거리 안전 한계

현재 heatmap range step은 약 9.8cm이고 TI 근거리 프로필은 7cm부터 시작한다.
따라서 7cm 미만은 직접 누설이나 고정 반사일 수 있으며 1cm 물체의 신뢰 가능한 거리
분리를 보장하지 않는다. `NORMAL`도 “10cm 이내 확인 포인트 없음”일 뿐 안전 판정이
아니다. 최종 로봇에는 범퍼 또는 별도 근접센서, 저속 제한과 비상 정지 절차가 필요하다.

현재 지도는 로봇 기준 현재 증거이며 SLAM이 아니다. 엔코더와 IMU의 시간 동기화 및
motion compensation은 다음 단계다.
