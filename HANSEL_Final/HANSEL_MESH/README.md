# HANSEL_MESH

구동·카메라·레이더·오도메트리를 함께 실행하는 순서는
[HANSEL Final 설치·실행 안내](../README.md)를 따른다.
아래는 네트워크와 개별 센서 기능을 위한 문서이며, 이미 구성된 장비에서
통합 실행을 시작하려고 네트워크 설치 절차를 다시 수행할 필요는 없다.

HANSEL_GRETEL 다유닛 로봇을 위한 BATMAN-adv 기반 Wi-Fi Mesh 릴레이 통신 프로젝트입니다.

- 계층 구조/토폴로지: [Architecture](docs/architecture.md)
- AR9271 동글 도착 후 실행 절차: [Dongle Arrival Runbook](docs/dongle_arrival_runbook.md)

## 목표 구조

```text
구조자 PC
   |
Base Pi
   |
Relay Node Pi
   |
Head Pi / Robot
```

| Hostname | 역할                     | Config            | bat0 IP       |
| -------- | ------------------------ | ----------------- | ------------- |
| base     | 구조자 쪽 기준 노드       | configs/base.env  | 192.168.50.1  |
| head     | 로봇 헤드 / 카메라 / 제어 | configs/head.env  | 192.168.50.10 |
| node1    | 릴레이 유닛 1            | configs/node1.env | 192.168.50.11 |
| node2    | 릴레이 유닛 2            | configs/node2.env | 192.168.50.12 |

`node3`는 예비 Pi가 생겼을 때 추가할 optional 릴레이입니다.
조종 클라이언트의 기본 active 목록도 `head,node1,node2`이다. node3를 실제로
연결한 시험에서는 `--active-targets head,node1,node2,node3`를 추가한다.

## 기본 실행 순서

```bash
chmod +x scripts/*.sh
sudo ./scripts/install_mesh.sh
sudo ./scripts/start_role_network.sh base
```

장치별로 `base`만 `head`, `node1`, `node2`로 바꿔서 실행합니다.

## 자동 실행

```bash
sudo ./scripts/enable_mesh_autostart.sh base
sudo systemctl start hansel-mesh@base
```

다른 장치는 `base`를 `head`, `node1`, `node2`로 바꿔서 사용합니다. 자세한 절차는 [Mesh Autostart Runbook](docs/mesh_autostart_runbook.md)을 따른다.

## 상태 확인

```bash
sudo ./scripts/check_mesh.sh
ping 192.168.50.1
sudo batctl n
sudo batctl o
```

## 카메라 + 조종

각 노드는 AR9271 동글(`wlan1`)을 Mesh 전용으로 쓰고, 내장 `wlan0`는 조난자 핸드폰 AP 등 향후 용도로 남깁니다. AR9271 동글 도착 전까지는 이 config로 Mesh가 뜨지 않으며, 그 전에 카메라 영상과 조종 명령 경로를 먼저 준비합니다.

실행 순서는 [Camera And Control Quickstart](docs/camera_control_quickstart.md)를 따른다.

다음 현장 테스트 시작 순서는 [Next Field Test Checklist](docs/next_field_test_checklist.md)를 따른다.

실제 GPIO 모터/엔코더 제어는 [Motor Control Quickstart](docs/motor_control_quickstart.md)를 따른다.

내일 구동 + 카메라 테스트 전체 순서는 [Tomorrow Drive + Camera Runbook](docs/tomorrow_drive_camera_runbook.md)을 따른다.

팀원 설명용 코드/원리 문서는 [HANSEL_MESH Code And Network Explainer](docs/hansel_mesh_code_explainer.md)를 따른다.

## IWRL6432BOOST 근거리 레이더 지도 (R31)

TI IWRL6432BOOST의 Application/User UART에서 Point Cloud와 Range-Azimuth Heatmap을
받아 브라우저에서 2D 롤링 지도와 3D Point Cloud를 함께 표시한다. Windows PC 직결
시험과 Head Raspberry Pi 수집을 모두 지원한다. Head 운용에서는 경로와 장애물 상태를
`192.168.50.10:8081`에 제공하고 구조자 PC가 BATMAN-adv 경로로 조회해 렌더링한다.

현재 구현의 목표는 SLAM이나 세계 좌표 지도를 만드는 것이 아니라, 움직이는
레이더를 기준으로 반지름 50cm 안의 강한 장애물 증거를 유지하면서 로봇이 당장
이동할 수 있는 국소 경로를 제안하는 것이다.

### 현재 구현된 기능

- Windows COM 포트 자동 탐색과 XDS110 보드 선택
- 3D/elevation 8, 16 azimuth bin, 128 range bin, 8Hz 레이더 프로필
- Point Cloud와 Heatmap의 별도 수집·분류
- 빈 장면 캘리브레이션 median/MAD를 이용한 고정 클러터 제거
- Heatmap 인접 셀 clustering, Point Cloud 공간 연관, 반복 검출과 위치 안정성 확인
- 신뢰도에 따른 `confirmed_obstacle`과 `reflection_evidence` 분리
- 레이더를 원 중심에 둔 반지름 50cm 로봇 상대 롤링 지도
- 레이더 포인트 정합으로 추정한 이동량을 이용한 장애물·이동 궤적 보정
- 강하고 반복된 장애물을 0.5cm 단위 롤링 셀에 누적
- 최근 1초 근거리 장애물과 누적 장애물을 결합한 반응형 경로 계획
- 2.5cm 탐색 격자에서 계획한 경로를 1cm 간격 곡선으로 표시
- 로봇 최소 폭 15cm와 좌우 안전 여유 3cm씩 적용(최소 통과 판단 폭 약 21cm)
- 좌우 편향 없는 ±60도 전방 탐색
- 노란색 계획 경로와 파란색 레이더 이동 궤적 표시
- 별도 0~50cm 3D Point Cloud 화면과 마우스 드래그 시점 회전
- 확인된 Point Cloud가 10cm 이내일 때만 `DANGER`, 13cm 이상에서 해제
- 센서 연결 끊김, 오래된 프레임, 보정 누락 및 프로필 불일치 시 화면 차단

### 화면 기호

| 표시 | 의미 | 유지 및 경로 반영 |
| --- | --- | --- |
| 주황색 원 | 여러 프레임에서 확인된 강한 장애물을 누적한 롤링 지도 셀 | 최소 2회 확인 후 표시한다. 레이더 이동에 맞춰 상대 좌표를 보정하며 반지름 50cm 밖으로 나갈 때까지 유지한다. 경로 계산에 강하게 반영한다. |
| 십자가 표적 | Point Cloud로 직접 검출한 현재 track | 최근 관측 후 최대 300ms 유지한다. 흰색으로 선명할수록 반복 확인된 포인트이며 확인된 포인트는 장애물 판단에 사용한다. |
| 하늘색 네모 | 5cm 단위 점유 증거 셀 | 현재 확인 track, 근거리 반응형 셀과 누적 셀의 최대 신뢰도를 표시한다. 표시가 없다고 빈 공간을 뜻하지 않는다. |
| 짧은 청록/흰색 호 | Heatmap cluster의 거리·각도 및 불확실성 | 반복성, Point Cloud 연관성과 위치 안정성이 확인된 경우에만 장애물로 승격한다. |
| 노란색 선 | 앞으로 이동할 국소 계획 경로 | 1cm 간격으로 표시한다. 50cm 끝까지 도달하지 못하면 현재 확인 가능한 지점까지만 그린다. |
| 파란색 점선 | 레이더가 지나온 것으로 추정한 상대 이동 궤적 | 엔코더/IMU가 아닌 레이더 포인트 정합 기반 이동 추정값이다. |
| 어두운 배경 | `UNKNOWN` | `FREE` 또는 안전한 통로라는 뜻이 아니다. |

3D 창은 좌우 `X`, 전후 `Y`, 상하 `Z`를 사용한다. 위쪽 포인트는 주황색,
아래쪽은 파란색, 레이더 높이와 가까운 포인트는 흰색/청록색으로 그린다. 3D 창의
포인트는 완성된 물체 표면이 아니라 레이더가 실제로 받은 희소 반사점이다.

### Head에서 수집하고 PC에서 보기

레이더를 Head Pi에 USB로 연결한 뒤 다음을 실행한다.

```bash
cd ~/HANSEL_MESH
bash scripts/start_head_radar_mesh.sh configs/radar/head_mesh.env.example
```

구조자 PC는 Base와 유선 연결하고 메시 경로를 설정한 뒤 실행한다.

```bash
sudo ./scripts/setup_laptop_mesh_routes.sh <PC의-유선-인터페이스>
./scripts/open_radar_mesh_view.sh
```

전체 설치, 서비스 등록, 검증 및 장애 해결 절차는
[Head Radar over BATMAN Runbook](docs/radar_mesh_head_runbook.md)을 따른다.

### Windows에서 직접 실행

보드가 연결되면 장치 관리자에서 다음 두 포트가 보여야 한다.

- `XDS110 Class Application/User UART`: 레이더 데이터 포트
- `XDS110 Class Auxiliary Data Port`: 보조/설정 포트

저장소 루트의 PowerShell에서 supervisor를 실행한다. `--xds-serial`은 연결한
XDS110의 serial로 바꾸고, 실행마다 `--run-id`를 고유하게 지정한다.

```powershell
$env:PYTHONPATH = (Resolve-Path '.deps').Path

python scripts\run_radar_stack.py `
  --xds-serial RI32RI32 `
  --cfg configs\radar\iwrl6432_3d_operator_near_8hz.cfg `
  --profile-id lsdk-05.05.04.02-presence-near-heatmap16-elev8-cfar12-8hz-v2 `
  --clutter-calibration configs\radar\calibrations\head-near-8hz.json `
  --run-id radar-r31-local
```

정상 실행 후 [http://127.0.0.1:8081/](http://127.0.0.1:8081/)을 연다.
상단에 `LIVE`, `UI R31`이 표시되고 3D 창에 `Z LIVE`가 나타나야 한다.
supervisor는 캡처와 웹 viewer를 감시하고, viewer가 종료되면 자동으로 다시 띄운다.

현재 포함된 `configs/radar/calibrations/head-near-8hz.json`은 기존 보드 장착과
시험 장소에서 만든 빈 장면 모델이다. 보드 방향, 케이블, 장착물, 프로필 또는 시험
공간이 달라지면 그대로 신뢰하지 말고 [IWRL6432 Front Radar View](docs/radar_front_view.md)의
절차로 빈 장면을 다시 기록해 캘리브레이션해야 한다.

### 데모 및 테스트

레이더가 없을 때는 합성 데이터로 UI를 확인할 수 있다.

```powershell
python monitor\radar_front.py --demo --bind 127.0.0.1 --http-port 8081
```

주요 Python 및 웹 테스트는 다음과 같이 실행한다.

```powershell
python -m unittest tests.test_radar_scene tests.test_radar_front

Get-ChildItem tests\web -Filter *.test.js | ForEach-Object {
  node --test $_.FullName
}
```

### 안전 및 정확도 한계

- 현재 2D 경로 계산에서는 높이를 제거하고 좌우·전후 평면만 사용한다. `Z`는 별도
  3D 참고 화면에만 사용한다.
- 엔코더와 IMU가 없으므로 이동 보정은 레이더 포인트 정합 결과에 의존한다. 반사가
  적거나 multipath가 심하면 누적 지도와 이동 궤적이 밀리거나 회전할 수 있다.
- Heatmap은 물체 표면이 아니라 반사 에너지다. 금속, 모서리, 바닥 반사, sidelobe와
  multipath 때문에 실제 물체보다 넓게 보일 수 있다.
- `NORMAL`은 확인된 포인트가 10cm 안에 없다는 의미일 뿐, 전체 경로가 안전하다는
  보장이 아니다.
- 현재 화면은 SLAM, 세계 좌표 지도 또는 모터 정지 장치가 아니다. 실제 로봇에는
  범퍼/근접 센서, 저속 제한과 비상 정지가 별도로 필요하다.

수동 캡처, 빈 장면 캘리브레이션, 기록 재생과 장애 상태 설명은
[IWRL6432 Front Radar View](docs/radar_front_view.md)를 참고한다.

## 통신 원칙

Base Pi가 명령 내용을 해석해 각 유닛으로 다시 보내는 dispatcher 구조가 아니라, 모든 유닛이 BATMAN-adv 릴레이 노드로 동작합니다.

데이터는 `bat0` 위에서 end-to-end로 흐르고, 중간 node는 BATMAN-adv next-hop으로 프레임을 포워딩합니다.
