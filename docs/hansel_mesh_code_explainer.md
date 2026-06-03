# HANSEL_MESH Code And Network Explainer

작성일: 2026-06-03

## 1. 프로젝트 요약

HANSEL_MESH는 붕괴 현장에 진입하는 다유닛 로봇의 통신 거리를 확장하기 위한 프로젝트다. 구조자 노트북은 base Pi와 유선 LAN으로 연결되고, base/head/node1/node2는 BATMAN-adv 기반 mesh에 참여한다.

목표 흐름:

```text
구조자 노트북
  |
base Pi
  |
BATMAN Mesh
  |
node2
  |
node1
  |
head Pi + Camera + Motor
```

중요한 설계 원칙:

- base는 명령 내용을 해석하지 않는다.
- node1/node2는 애플리케이션 relay가 아니라 BATMAN-adv Layer 2 relay로 동작한다.
- 조종 명령과 카메라 영상은 `bat0` 위에서 end-to-end로 흐른다.
- 현재 동글 없는 구성에서는 조난자 핸드폰 AP 기능은 구현하지 않는다.
- head만 조향한다. node1/node2는 head가 만든 경로를 직진/저속 추종한다.

## 2. 코드 구조

```text
configs/
  base.env
  head.env
  node1.env
  node2.env

scripts/
  start_mesh.sh
  stop_mesh.sh
  check_mesh.sh
  setup_base_gateway.sh
  setup_laptop_mesh_routes.sh
  setup_mesh_route_to_laptop.sh
  start_camera_stream.sh
  receive_camera_stream.sh

controller/
  mesh_control_client.py

robot/
  mesh_control_server.py
  motor_driver.py
  __init__.py

docs/
  tomorrow_drive_camera_runbook.md
  motor_control_quickstart.md
  camera_control_quickstart.md
  next_field_test_checklist.md
```

## 3. 통신부 개요

### 3.1 IP 대역

| 장치 | 인터페이스 | IP |
| ---- | ---------- | -- |
| laptop | LAN adapter | 192.168.60.2/24 |
| base | eth0 | 192.168.60.1/24 |
| base | bat0 | 192.168.50.1/24 |
| head | bat0 | 192.168.50.10/24 |
| node1 | bat0 | 192.168.50.11/24 |
| node2 | bat0 | 192.168.50.12/24 |

`192.168.60.0/24`는 노트북과 base 사이의 관리망이다. `192.168.50.0/24`는 BATMAN mesh 위에서 사용하는 로봇망이다.

### 3.2 BATMAN-adv 원리

BATMAN-adv는 Linux kernel module로 동작하는 Layer 2 mesh 방식이다. Linux kernel documentation에 따르면 batman-adv는 IP routing table을 직접 다루는 방식이 아니라 Ethernet frame을 Layer 2에서 전달하며, 참여 노드들이 하나의 가상 switch처럼 보이게 한다. 따라서 IPv4, IPv6, DHCP 같은 상위 프로토콜이 mesh topology 변화에 크게 영향을 받지 않는다.

이 프로젝트에서 중요한 의미:

- `wlan0`는 IBSS/ad-hoc 무선 링크 역할만 한다.
- `wlan0`에는 IP를 주지 않는다.
- `bat0`가 상위 애플리케이션이 보는 실제 통신 인터페이스다.
- IP는 `bat0`에만 부여한다.
- node가 중간에 배치되면 BATMAN-adv가 next-hop을 자동 선택한다.

### 3.3 batctl 확인 명령

`batctl`은 batman-adv kernel module 상태를 확인하는 도구다. batctl documentation은 neighbor table과 originator table을 통해 직접 이웃과 목적지별 next-hop을 볼 수 있다고 설명한다.

핵심 명령:

```bash
sudo batctl n
sudo batctl o
```

해석:

- `batctl n`: 현재 직접 수신 가능한 neighbor
- `batctl o`: mesh 전체 originator와 목적지별 selected next-hop
- `batctl o`에서 `*`가 붙은 줄은 BATMAN이 선택한 현재 best nexthop이다.
- 목적지가 head인데 selected `Nexthop`이 node1/node2 MAC이면 실제 multi-hop relay 중이다.

### 3.4 start_mesh.sh

역할:

1. Wi-Fi 차단 해제
2. `batman-adv` kernel module load
3. hostapd/dnsmasq/wpa_supplicant 등 충돌 가능 서비스 정리
4. 기존 `bat0` 삭제
5. `wlan0`를 mesh point 또는 IBSS로 구성
6. `bat0` 생성
7. `wlan0`를 BATMAN hard interface로 추가
8. `bat0`에 고정 IP 부여

Pi 3 내장 Wi-Fi는 실제 테스트에서 `mesh point`가 아니라 `IBSS`를 사용했다. 그래서 `start_mesh.sh`는 `MESH_MODE=auto`에서 가능한 모드를 확인하고 IBSS fallback을 수행한다.

중요 변수:

```text
MESH_IF=wlan0
BAT_IF=bat0
MESH_ID=HANSEL_MESH
MESH_FREQ=2437
IBSS_BSSID=02:12:34:56:78:9a
IP_ADDR=192.168.50.x
NETMASK_CIDR=24
```

### 3.5 base gateway

`setup_base_gateway.sh`는 base의 `eth0`를 노트북 관리망으로 설정한다.

```text
base eth0 = 192.168.60.1/24
base bat0 = 192.168.50.1/24
```

또한 Linux IPv4 forwarding을 활성화한다.

```bash
sysctl -w net.ipv4.ip_forward=1
```

base는 여기서도 애플리케이션 relay가 아니다. 단순히 kernel routing으로 `eth0`와 `bat0` 사이를 연결한다.

### 3.6 laptop route

`setup_laptop_mesh_routes.sh`는 노트북 LAN adapter를 `192.168.60.2/24`로 설정하고, mesh 대역을 base로 보낸다.

```bash
ip route replace 192.168.50.0/24 via 192.168.60.1
```

이 route가 있어야 노트북이 `192.168.50.10` head로 직접 UDP/ping을 보낼 수 있다.

### 3.7 head route back to laptop

head에서 `setup_mesh_route_to_laptop.sh`를 실행하면 `192.168.60.0/24`로 가는 route가 base `bat0`를 향한다.

```bash
ip route replace 192.168.60.0/24 via 192.168.50.1 dev bat0
```

카메라 영상은 head에서 노트북 `192.168.60.2`로 UDP 송신하므로 이 reverse route가 필요하다.

## 4. 데이터 형태와 통신 형태

### 4.1 조종 명령 UDP JSON

노트북의 `controller/mesh_control_client.py`는 UDP JSON packet을 보낸다.

예:

```json
{
  "seq": 12,
  "target": "head",
  "command": "forward",
  "source": "operator",
  "time": 1780400166.3324077,
  "speed": 0.4
}
```

필드:

| 필드 | 의미 |
| ---- | ---- |
| seq | 송신 순번 |
| target | head, node1, node2 |
| command | forward, stop 등 명령 |
| source | operator |
| time | 송신 시각 |
| speed | 선택 필드, 0.0~1.0 속도 scale |

UDP port:

```text
7000
```

UDP를 쓰는 이유:

- 조종 명령은 최신 상태가 중요하다.
- 오래된 명령을 TCP처럼 재전송받는 것보다 watchdog으로 멈추는 편이 안전하다.
- packet loss가 있어도 live mode가 주기적으로 명령을 반복 송신한다.

### 4.2 조종 명령 흐름

```text
controller/mesh_control_client.py
  -> UDP JSON
  -> laptop LAN 192.168.60.2
  -> base eth0 192.168.60.1
  -> base bat0 192.168.50.1
  -> BATMAN selected next-hop
  -> target bat0
  -> robot/mesh_control_server.py
  -> robot/motor_driver.py
  -> GPIO motor driver
```

### 4.3 live mode 라우팅

`--target all`일 때 명령 분배:

| 키 | head | node1/node2 |
| -- | ---- | ----------- |
| w | forward | forward |
| s | backward | backward |
| a | left | stop |
| d | right | stop |
| q | forward_left | slow_forward |
| e | forward_right | slow_forward |
| z | backward_left | slow_backward |
| c | backward_right | slow_backward |
| x/space | stop | stop |

즉 조향은 head만 수행하고, node는 head가 만든 경로를 따라가기 위해 직진/저속/정지만 한다.

node 서버에서도 이중 안전장치를 둔다. node가 실수로 `left`, `right`, `forward_left` 같은 조향 명령을 직접 받아도 `stop`, `slow_forward`, `slow_backward`로 변환한다.

### 4.4 카메라 영상 UDP H.264

head에서:

```bash
rpicam-vid -t 0 --nopreview --width 640 --height 480 --framerate 15 --codec h264 --inline --bitrate 1200000 -o udp://192.168.60.2:5600
```

노트북에서:

```bash
./scripts/receive_camera_stream.sh 5600
```

특징:

- H.264 elementary stream
- UDP port 5600
- `--inline`으로 SPS/PPS를 주기적으로 포함해 decoder가 중간 수신해도 복구하기 쉽게 함
- mesh 환경에서는 해상도와 bitrate를 낮춰 안정성을 우선한다.

추천 시작값:

```text
320x240, 10fps, 600kbps
```

가까운 거리 안정값:

```text
640x480, 15fps, 1200kbps
```

## 5. 구동부 개요

### 5.1 파일 역할

`robot/mesh_control_server.py`:

- UDP socket bind
- JSON packet parsing
- watchdog timeout
- dry-run 지원
- `motor_driver`에 command 전달

`robot/motor_driver.py`:

- GPIO pin map
- PWM motor output
- encoder polling
- PID control loop
- head servo
- detach servo
- head front DC motors
- node command safety normalization

`controller/mesh_control_client.py`:

- 노트북 조종 입력
- line mode
- live mode
- target별 UDP JSON 송신
- `--target all`에서 head/node 명령 분배

### 5.2 GPIO pin map

공통 drive motor:

| 기능 | GPIO |
| ---- | ---- |
| ENA left PWM | 18 |
| IN1 left | 23 |
| IN2 left | 24 |
| ENB right PWM | 13 |
| IN3 right | 27 |
| IN4 right | 22 |

encoder:

| 기능 | GPIO |
| ---- | ---- |
| LEFT_ENC_A | 20 |
| LEFT_ENC_B | 21 |
| RIGHT_ENC_A | 16 |
| RIGHT_ENC_B | 26 |

servo:

| 기능 | GPIO |
| ---- | ---- |
| detach servo | 6 |
| head servo | 17 |

head front DC motors:

| 기능 | GPIO |
| ---- | ---- |
| FRONT_ENA | 12 |
| FRONT_IN1 | 3 |
| FRONT_IN2 | 8 |
| FRONT_ENB | 19 |
| FRONT_IN3 | 5 |
| FRONT_IN4 | 7 |

### 5.3 구동 메커니즘

head:

- 좌우 두 바퀴 RPM 차이로 조향한다.
- 앞쪽 고개 부분 DC모터 2개도 기본적으로 주행 바퀴와 같은 방향/PWM ratio를 따라간다.
- `forward_left`면 왼쪽 target CPS가 낮고 오른쪽 target CPS가 높다.
- `left`면 왼쪽은 후진, 오른쪽은 전진해 제자리 조향한다.

node1/node2:

- 직진/후진/정지만 수행한다.
- 조향 명령을 받으면 서버 내부에서 안전하게 변환한다.
- head 조향 중에는 `slow_forward` 또는 `slow_backward`로 따라간다.

### 5.4 엔코더 처리

encoder는 A/B 두 신호의 quadrature transition을 읽는다.

상태:

```text
state = (A << 1) | B
```

transition:

```text
transition = (last_state << 2) | new_state
```

forward count 증가 transition:

```text
0001, 0111, 1110, 1000
```

backward count 감소 transition:

```text
0010, 1011, 1101, 0100
```

RPi.GPIO event detect는 OS/kernel 조합에서 문제가 날 수 있어 polling thread를 사용한다.

기본 polling interval:

```text
0.001s
```

### 5.5 PID 제어 로직

제어 주기:

```text
CONTROL_INTERVAL = 0.05s
```

각 주기에서:

```text
delta_count = current_count - previous_count
measured_cps = abs(delta_count) / dt
error = target_cps - measured_cps
integral += error * dt
derivative = (error - previous_error) / dt
```

feed-forward:

```text
base_pwm = MIN_PWM + (target_cps / max_cps) * (MAX_PWM - MIN_PWM)
```

PID output:

```text
pid_output = Kp * error + Ki * integral + Kd * derivative
pwm_request = clamp(base_pwm + pid_output, 0, 100)
```

PWM ramp 제한:

```text
max_delta = PWM_RAMP_PER_SEC * dt
pwm = clamp(pwm_request, previous_pwm - max_delta, previous_pwm + max_delta)
```

이 구조의 이유:

- feed-forward는 목표 속도에 맞는 기본 PWM을 빠르게 제공한다.
- PID는 부하/마찰/배터리 전압 차이를 보정한다.
- ramp 제한은 급격한 전류/토크 변화를 줄인다.
- integral limit은 wind-up을 방지한다.

기본값:

| 파라미터 | 값 |
| -------- | -- |
| FULL_SPEED_CPS | 800 |
| MIN_PWM | 25 |
| MAX_PWM | 100 |
| KP | 0.035 |
| KI | 0.015 |
| KD | 0 |
| INTEGRAL_LIMIT | 500 |
| PWM_RAMP_PER_SEC | 220 |

### 5.6 안전 기능

- UDP command watchdog: 기본 0.5초 동안 명령이 없으면 stop
- `Ctrl+C` 종료 시 stop 전송
- GPIO cleanup
- encoder polling error 발생 시 motor stop 시도
- node 조향 명령 무시/변환
- `--dry-run`으로 GPIO 없이 네트워크만 테스트 가능
- duplicate GPIO pin check

## 6. 라이브러리

Python 표준 라이브러리:

- `socket`: UDP 송수신
- `json`: command packet encoding/decoding
- `time`: timestamp, control loop timing
- `threading`: encoder polling loop와 PID control loop
- `argparse`: CLI option parsing
- `select`, `termios`, `tty`: live keyboard input
- `dataclasses`: pin/config 구조화

Raspberry Pi 라이브러리:

- `RPi.GPIO`: motor driver direction pin, PWM, encoder input
- `pigpio`: head servo pulse width control

외부 시스템 도구:

- `batctl`: BATMAN-adv 상태 확인
- `iw`, `iwconfig`: IBSS/ad-hoc 무선 설정 확인
- `ip`: interface/IP/route 설정
- `rpicam-vid`: Camera Module 3 영상 송신
- `ffplay`, `gst-launch-1.0`, `vlc`: 노트북 영상 수신

## 7. 주요 실행 흐름

### 7.1 mesh 시작

```text
start_mesh.sh
  -> config source
  -> Wi-Fi manager stop
  -> wlan0 IBSS/ad-hoc join
  -> bat0 create
  -> batctl if add wlan0
  -> bat0 IP assign
```

### 7.2 조종 서버 시작

```text
mesh_control_server.py
  -> build_robot_controller(role)
  -> GPIO setup or dry-run
  -> UDP bind 0.0.0.0:7000
  -> receive packet
  -> parse JSON
  -> watchdog update
  -> motor_driver.handle_command()
```

### 7.3 motor command 처리

```text
handle_command()
  -> normalize command
  -> if node: steering command -> straight/stop
  -> command_map lookup
  -> set target CPS and direction
  -> PID loop changes PWM
```

### 7.4 camera 처리

```text
receive_camera_stream.sh on laptop
  -> ffplay/gstreamer/vlc wait UDP 5600

start_camera_stream.sh on head
  -> rpicam-vid
  -> H.264 UDP
  -> 192.168.60.2:5600
```

## 8. BATMAN relay 판단 방법

가까운 거리:

```text
base -> head direct
```

이때 `batctl o`의 selected nexthop은 목적지 MAC과 같을 수 있다.

멀리 배치:

```text
base -> node2 -> node1 -> head
```

이때 base의 `batctl o`에서 head originator의 selected nexthop이 node2 또는 node1 쪽 MAC으로 바뀌면 실제 relay다.

검증 명령:

```bash
ping 192.168.50.10
sudo batctl n
sudo batctl o
```

## 9. 문제 해결

### 9.1 base와 노트북 연결이 끊김

base:

```bash
sudo ./scripts/setup_base_gateway.sh
```

노트북:

```bash
sudo ./scripts/setup_laptop_mesh_routes.sh enx00e04c68070e
```

확인:

```bash
ping -c 4 192.168.60.1
ping -c 4 192.168.50.10
```

### 9.2 카메라 busy

```bash
sudo pkill -f rpicam-vid
sudo pkill -f libcamera-vid
sudo pkill -f rpicam-hello
sudo pkill -f libcamera-hello
sudo pkill -f rpicam-still
sudo pkill -f libcamera-still
```

### 9.3 motor server 시작 실패

확인:

- `sudo`로 실행했는가
- `RPi.GPIO` 설치 여부
- encoder 핀 배선
- 이전 제어 서버가 GPIO를 잡고 있는지
- pin conflict 메시지

### 9.4 방향 반대

환경변수로 보정:

```bash
HANSEL_LEFT_REVERSE=yes sudo -E python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head
HANSEL_RIGHT_REVERSE=yes sudo -E python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head
```

### 9.5 영상 깨짐

낮은 설정으로 시작:

```bash
WIDTH=320 HEIGHT=240 FPS=10 BITRATE=600000 ~/HANSEL_MESH/scripts/start_camera_stream.sh 192.168.60.2 5600
```

## 10. 참고 자료

- Linux kernel batman-adv documentation: https://www.kernel.org/doc/html/v4.17/networking/batman-adv.html
- batctl upstream repository and man page text: https://github.com/open-mesh-mirror/batctl
- batctl HTML man page: https://downloads.open-mesh.org/batman/manpages/batctl.8.html
- Previous HANSEL_GRETEL repository referenced for motor pin map and control structure: https://github.com/SSING-rloz/2026-_HANSEL_GRETEL
