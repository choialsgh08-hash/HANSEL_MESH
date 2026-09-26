# Head Radar over BATMAN Runbook

이 문서는 IWRL6432BOOST 레이더를 Head Raspberry Pi에 연결하고, Head가 계산한
장애물·점유 지도·국소 경로를 BATMAN-adv 메시를 통해 구조자 PC에서 시각화하는
절차를 설명한다.

## 1. 최종 구조

```text
IWRL6432BOOST
  | USB Application/User UART
  v
Head Pi (192.168.50.10)
  capture -> clutter filter -> scene/path planner -> HTTP API :8081
  |
  | bat0 / BATMAN-adv (필요하면 node1, node2가 Layer 2 중계)
  v
Base Pi bat0 192.168.50.1 <-> eth0 192.168.60.1
  |
  | Ethernet
  v
Operator PC 192.168.60.2 -> browser visualization
```

레이더 원시 UART 바이트를 PC로 전달하지 않는다. Head가 센서 프레임을 파싱하고
경로를 계산한 뒤 `/api/radar` 상태 스냅샷을 제공한다. 브라우저는 이를 약 100ms마다
요청하고 직접 2D 지도와 3D 포인트클라우드를 그린다. 큰 JSON 응답은 gzip으로
압축되므로 카메라 RTP 스트림과 공유하는 메시 대역폭을 덜 사용한다.

## 2. 통합 코드 배포

개발 PC에서 통합 변경을 커밋하고 원격 저장소에 푸시한 다음, 인터넷이 연결된 Base가
최신 코드를 받는다.

```bash
cd ~/HANSEL_MESH
git pull --ff-only
```

메시가 연결된 상태에서 Base가 같은 커밋을 Head와 노드에 배포한다.

```bash
cd ~/HANSEL_MESH
./scripts/deploy_from_base.sh
```

`deploy_from_base.sh`는 안전을 위해 깨끗한 Git 작업 트리와 SSH 키 인증을 요구하고,
구동 서버가 실행 중이면 배포를 중단한다. 레이더만 급히 검증할 때도 먼저 구동 서버를
정상 종료한 뒤 전체 스냅샷을 배포한다.

## 3. Head 준비

Head는 Raspberry Pi OS의 패키지로 pyserial과 진단 도구를 설치한다.

```bash
sudo apt update
sudo apt install -y python3-serial curl
sudo usermod -aG dialout hansel
```

그룹 변경은 로그아웃 후 다시 로그인하거나 재부팅해야 적용된다. 레이더를 USB에
연결하고 다음을 확인한다.

```bash
groups
python3 -c "import serial; print(serial.__version__)"
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

자동 탐색은 TI VID/PID와 `Application/User UART`를 우선한다. Linux에서 두 포트가
모두 일반 `XDS110`으로 표시되면 USB 인터페이스 위치가 `.0`인 `/dev/ttyACM*`를
Application UART로 선택하고 `.3`인 Auxiliary 포트는 거부한다. 여러 XDS110이 연결돼
있으면 `/etc/hansel-mesh/radar-head.env`의 `RADAR_XDS_SERIAL` 또는
`RADAR_SERIAL_PORT`를 지정한다.

## 4. 프로필과 캘리브레이션

기본 프로필은 다음 파일이다.

```text
configs/radar/iwrl6432_2p5d_operator_nearwall_5hz.cfg
```

기본 클러터 캘리브레이션은 다음 파일이다.

```text
configs/radar/calibrations/head-nearwall-16az-adc-5hz.json
```

포함된 캘리브레이션은 기존 장착 각도와 빈 공간에서 측정한 값이다. 레이더 장착 높이,
기울기, 브래킷 또는 주변 구조물이 바뀌면 다시 캘리브레이션해야 한다. 잘못된
캘리브레이션은 실제 장애물을 지우거나 고정 구조물을 장애물로 남길 수 있다.

## 5. 수동 실행

### Head

Head의 메시와 PC 회신 경로를 먼저 확인한다.

```bash
cd ~/HANSEL_MESH
sudo systemctl start hansel-mesh@head
sudo ./scripts/setup_mesh_route_to_laptop.sh
ip -brief addr show bat0
ip route get 192.168.60.2
ping -c 3 192.168.50.1
```

`bat0`에 `192.168.50.10/24`가 있고 PC 경로가 Base `192.168.50.1`을 향해야 한다.
레이더를 시작한다.

```bash
bash scripts/start_head_radar_mesh.sh configs/radar/head_mesh.env.example
```

자동 탐색이 틀리면 포트를 명시한다.

```bash
RADAR_SERIAL_PORT=/dev/ttyACM0 bash scripts/start_head_radar_mesh.sh
```

### Base

```bash
cd ~/HANSEL_MESH
sudo systemctl start hansel-mesh@base
sudo ./scripts/setup_base_gateway.sh
sysctl net.ipv4.ip_forward
ping -c 3 192.168.50.10
sudo batctl n
sudo batctl o
```

`net.ipv4.ip_forward = 1`이어야 하고 Head ping이 성공해야 한다.

### 구조자 PC

Base와 연결된 유선 인터페이스 이름을 확인하고 PC 주소와 메시 경로를 설정한다.

```bash
ip -brief addr
cd ~/HANSEL_MESH
sudo ./scripts/setup_laptop_mesh_routes.sh enx00e04c68070e
ping -c 3 192.168.60.1
ping -c 3 192.168.50.10
./scripts/open_radar_mesh_view.sh
```

유선 인터페이스 이름은 실제 장치명으로 바꾼다. 브라우저에서 다음 주소를 직접 열어도
된다.

```text
http://192.168.50.10:8081/
```

## 6. Head 자동 실행

수동 검증이 끝난 뒤 Head에서 서비스와 설정 파일을 설치한다.

```bash
cd ~/HANSEL_MESH
sudo ./scripts/enable_head_radar_service.sh
sudo nano /etc/hansel-mesh/radar-head.env
sudo systemctl start hansel-radar-head
```

상태와 로그는 다음 명령으로 확인한다.

```bash
systemctl status hansel-radar-head --no-pager
journalctl -u hansel-radar-head -n 100 --no-pager
journalctl -u hansel-radar-head -f
```

레이더 서비스는 `hansel-mesh@head.service` 이후 시작되고 실패하면 3초 뒤 재시작한다.
설치 스크립트는 서비스를 enable만 하며, 하드웨어 확인 전에는 자동으로 시작하지 않는다.

## 7. 전송 데이터

PC는 `GET /api/radar`로 다음 데이터를 받는다.

- 센서 frame 번호와 수신 시각
- Point Cloud와 압축 heatmap
- 확인 장애물, 반사 증거, 점유 셀
- 로봇 기준 planned path와 이동 궤적
- `DANGER`를 포함한 hazard 상태
- 센서 연결, 오래된 프레임, 캘리브레이션 상태 등 health 정보

이 트래픽은 TCP/HTTP 위에서 동작한다. BATMAN-adv는 중간 노드의 Layer 2 경로를
선택하며 애플리케이션은 중계 노드를 직접 지정하지 않는다. 브라우저가 gzip을
지원하면 `/api/radar` 응답은 자동 압축된다.

## 8. 확인 순서와 장애 해결

화면이 열리지 않으면 아래 순서로 끊긴 지점을 찾는다.

```bash
# PC
ip route get 192.168.50.10
ping -c 3 192.168.60.1
ping -c 3 192.168.50.10
curl -v --max-time 3 http://192.168.50.10:8081/api/radar

# Base
sysctl net.ipv4.ip_forward
sudo batctl n
sudo batctl o
ping -c 3 192.168.50.10

# Head
ip -brief addr show bat0
ip route get 192.168.60.2
systemctl status hansel-radar-head --no-pager
journalctl -u hansel-radar-head -n 100 --no-pager
```

- Head ping 실패: 메시 Cell/ESSID/frequency, `batctl n/o`, stale neighbor를 확인한다.
- API만 실패: 레이더 서비스와 8081 listen 상태를 확인한다.
- `Permission denied` 직렬 오류: `dialout` 그룹과 장치 소유권을 확인한다.
- 포트 자동 탐색 실패: Application/User UART의 `/dev/tty*`를 설정 파일에 명시한다.
- 센서 연결은 됐지만 화면 차단: 프로필 ID, calibration, stale-frame health를 확인한다.
- 카메라와 함께 느림: 카메라 bitrate를 먼저 낮추고 메시 hop별 품질을 확인한다.

```bash
ss -lntp | grep 8081
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

운용 성공 기준은 PC에서 Head API가 지속 갱신되고, 노드를 중간에 둔 상태에서도
frame 시각, 장애물, planned path가 끊김 없이 변하는 것이다.
