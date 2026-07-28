# IWRL6432BOOST Windows 재연결

아래 명령은 저장소 루트에서 실행한다. COM3는 Application/User UART이고 COM4는 Auxiliary 포트다.

## 1. 보드 초기화와 프로필 적용

1. 기존 `radar-live` 프로세스를 `Ctrl+C`로 종료한다.
2. 보드의 RESET 버튼을 한 번 누른다.
3. 첫 번째 PowerShell에서 다음을 실행한다.

```powershell
.\.venv\Scripts\python.exe scripts\configure_ti_radar.py `
  --port COM3 `
  --cfg configs\radar\iwrl6432_3d_operator_near_10hz.cfg `
  --command-timeout 1.5
```

결과에서 `commands_completed=25`, `new_baud_verified_by_version=true`,
`first_magic_observed=true`를 확인한다. 프로필 적용 후에는 다시 RESET하지 않는다.

## 2. 레이더 캡처 시작

같은 PowerShell에서 실행한다. 파일명이 겹치지 않도록 현재 시각을 붙인다.

```powershell
$stamp = Get-Date -Format yyyyMMdd-HHmmss
.\.venv\Scripts\python.exe -m sensors radar-live `
  --port COM3 `
  --baud 1250000 `
  --allow-elided-empty-point-tlv `
  --allow-nonzero-padding `
  --heatmap-azimuth-bins 16 `
  --heatmap-range-bins 128 `
  --heatmap-range-step-m 0.09765625 `
  --output "missions\radar-close-$stamp.jsonl" `
  --raw-output "captures\radar-close-$stamp.bin" `
  --mission-id "radar-close-$stamp" `
  --profile-id lsdk-05.05.04.02-presence-near-heatmap16-elev8-cfar15-10hz-v1 `
  --calibration-id uncalibrated
```

이 창은 캡처가 끝날 때까지 닫지 않는다.

## 3. 50cm 3D 화면 시작

두 번째 PowerShell을 저장소 루트에서 열고 실행한다.

```powershell
$mission = (Get-ChildItem missions\radar-close-*.jsonl |
  Sort-Object LastWriteTime -Descending |
  Select-Object -First 1).FullName

.\.venv\Scripts\python.exe monitor\radar_front.py `
  --follow $mission `
  --bind 127.0.0.1 `
  --http-port 8081 `
  --max-range-m 0.5 `
  --min-forward-m 0.0 `
  --max-points 2048 `
  --history-window 0.2
```

브라우저에서 `http://127.0.0.1:8081/`을 연다. 기존 탭은 `Ctrl+F5`로 새로 고친다.
정상 상태는 `LIVE`, `UI R7`, `3D HEMISPHERE MAP`, `CLOSE 0-0.50m`이다.

## 다시 끊길 때

- `ClearCommError`가 나오면 캡처를 종료하고 USB 케이블과 보드 전원을 확인한다.
- 가능하면 USB 허브를 거치지 말고 노트북 본체 포트와 데이터용 케이블을 사용한다.
- COM3가 다시 보이면 RESET 후 1단계부터 반복한다.
- 웹이 `STALE`, `FAULT`, `WAITING`이면 화면이 남아 있어도 주행에 사용하지 않는다.

## 근거리 한계

UI는 1cm 이상으로 들어오는 모든 전방 포인트를 버리지 않는다. 그러나 현재 프로필의 heatmap range
step은 약 9.8cm이며, TI 제공 근거리 설정도 7cm부터 시작한다. 따라서 7cm 미만 숫자는 직접 누설이나
고정 반사일 수 있고 1cm 물체의 신뢰성 있는 거리 분리는 보장하지 않는다.
