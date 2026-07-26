# IWRL6432 전방 레이더 조종 화면

이 화면은 **IMU 없이 현재 레이더 프레임을 로봇 상대 좌표로 표시**한다.
SLAM 지도, 위치 추정, 장애물 자동 회피 기능은 아직 포함하지 않는다. 기존
모터 제어·카메라·통신 대시보드와 독립적으로 실행되며 모터 명령을 보내지
않는다.

## 오늘: 레이더 없이 화면 확인

Windows PowerShell 또는 Raspberry Pi에서 저장소 루트로 이동해 실행한다.

```powershell
python monitor/radar_front.py --demo
```

브라우저에서 다음 주소를 연다.

```text
http://127.0.0.1:8081
```

합성 화면에는 좌우 잔해, 중앙 장애물, 접근하는 이동 표적이 표시된다. 한
프레임에 감지점이 0개인 경우도 주기적으로 발생한다. 이때 화면은
`NO RETURNS — 빈 공간 보장 아님`이라고 경고한다.

## 내일: 실제 IWRL6432BOOST 연결

IWRL6432의 demo/app cfg를 먼저 적용하고 processed-data UART가 출력 중인
상태여야 한다. TI Visualizer, 터미널, 다른 캡처 프로그램과 같은 COM 포트를
동시에 열지 않는다.

### 1. Head Pi/보드 연결 PC에서 UART 기록 시작

```powershell
python -m pip install -r requirements-sensors.txt
python -m sensors radar-live `
  --port COM5 `
  --baud 115200 `
  --output missions\radar-bench-01.jsonl `
  --raw-output captures\radar-bench-01.bin `
  --mission-id radar-bench-01 `
  --profile-id lsdk-05.05.00.02-appcfg-CFGHASH `
  --calibration-id uncalibrated
```

Linux에서는 `COM5` 대신 실제 장치(예: `/dev/ttyACM0`)를 사용한다. cfg가
UART baud를 1,250,000으로 바꿨다면 `--baud 1250000`으로 반드시 맞춘다.
`profile-id`에는 SDK/appimage/cfg 조합을 구별할 이름 또는 cfg 해시를 넣는다.

### 2. 별도 터미널에서 최신 프레임 화면 시작

같은 장치에서 실행할 때:

```powershell
python monitor/radar_front.py --follow missions\radar-bench-01.jsonl
```

`--follow`는 시작 시 이미 완성되어 있던 과거 레코드를 건너뛰고, 실행 뒤
추가되는 프레임만 표시한다. 저장된 장면을 보고 싶을 때는 `--follow`가
아니라 아래의 `--replay`를 사용한다. 쓰는 중인 마지막 줄은 줄바꿈으로
완성될 때까지 표시하지 않는다.

`radar-live`는 기본 0.5초마다 파서 오류, 폐기 바이트, 프레임 누락과 로그
writer drop 누계가 담긴 health 레코드를 함께 기록한다. 따라서 캡처를
종료할 때까지 기다리지 않고 `--follow` 화면이 오류를 `DEGRADED`로
고정한다.

Head Pi에서 실행하고 메인 PC가 mesh를 통해 볼 때:

```bash
python3 monitor/radar_front.py \
  --follow missions/radar-bench-01.jsonl \
  --bind 192.168.50.10 \
  --http-port 8081
```

메인 PC 브라우저에서 `http://HEAD_BAT0_IP:8081`을 연다. 저장소 기본
구성이라면 예시는 `http://192.168.50.10:8081`이다. 현장 외부 네트워크에는
이 포트를 노출하지 않는다. `0.0.0.0` 대신 Head의 `bat0` 주소에만 바인딩해
`wlan1` 구조용 AP에서 조종 화면이 의도치 않게 공개되지 않도록 한다.

### 3. 기록 재생

```powershell
python monitor/radar_front.py `
  --replay missions\radar-bench-01.jsonl `
  --speed 1 `
  --loop
```

## 좌표와 색상

IWRL6432 demo native 좌표 기본값:

- `+Y`: 전방
- `+X`: 우측
- `+Z`: 위

따라서 화면 위쪽은 TI `+Y`, 화면 오른쪽은 TI `+X`다. 보드를 반대로
장착했거나 펌웨어 좌표가 다르면 실행 옵션으로 바꿀 수 있다.

```powershell
python monitor/radar_front.py --demo `
  --forward-axis y --forward-sign -1 `
  --lateral-axis x --lateral-sign -1
```

실제 보드 장착 후 정면 1/2/3 m, 좌측, 우측에 반사체를 놓아 방향을 반드시
검증한다. `calibration-id=uncalibrated`인 동안 화면에는 `장착축 미보정`
배지가 계속 표시된다.

Doppler 색은 TI 부호 기준이다.

- 빨강/주황: 음수, 레이더에 접근
- 흰색: 거의 정적
- 파랑: 양수, 레이더에서 멀어짐

로봇 자체가 움직이면 정적인 벽도 Doppler를 갖는다. 색은 물체 추적 결과나
세계 좌표 속도가 아니다.

## 실제 보드 첫 점검

1. 정면 1/2/3 m 반사체가 올바른 거리로 보이는지 확인한다.
2. 좌/우 반사체가 화면 좌/우와 일치하는지 확인한다.
3. 사람이 접근/후퇴할 때 Doppler 색이 바뀌는지 확인한다.
4. UART 케이블을 제거했을 때 0.75초 안에 `STALE`, 2초 안에 `FAULT`가
   표시되는지 확인한다.
5. 유효한 0-point 프레임과 통신 단절이 서로 다르게 표시되는지 확인한다.
   TI point-cloud TLV 자체가 없는 프레임은 잘못된 cfg 가능성이 있으므로
   `LIVE`가 아니라 `DEGRADED`로 표시되어야 한다.
6. 10분 이상 실행해 메모리 사용과 프레임률을 확인한다.

이 화면만 보고 안전거리나 통과 가능 여부를 확정하면 안 된다. 실제
주행에서는 카메라 화면, 운전자 판단, 저속 운용, 비상 정지 절차를 함께
사용한다.

프레임 누락, 불완전 프레임, 로그 오류가 한 번이라도 검출되면 짧은 경고가
브라우저 갱신 사이에 사라지지 않도록 해당 뷰어 실행 동안 `DEGRADED`가
유지된다. 원인을 확인하고 캡처/배선을 정상화한 뒤 뷰어를 재시작해 새
진단 세션을 시작한다.
