# Tomorrow Drive + Camera Runbook

내일 테스트는 이 문서만 보고 처음부터 진행한다.

## 0. 목표

```text
노트북
  |
LAN
  |
base
  |
BATMAN Mesh
  |
node2
  |
node1
  |
head + camera + motor control
```

성공 기준:

- 노트북에서 `192.168.50.10` head ping 성공
- base에서 `sudo batctl n`, `sudo batctl o`로 mesh neighbor/originator 확인
- 조종 명령이 head/node1/node2에 end-to-end 도착
- `w` 입력 시 모든 유닛이 같은 목표 CPS로 전진
- head 조향 시 node1/node2는 직진 속도를 낮추거나 정지
- head 카메라 영상이 노트북에서 수신됨
- 멀리 배치했을 때 node1/node2가 릴레이로 동작해 통신 유지

## 1. 장비 준비

필요 장비:

- 노트북
- USB-C LAN 어댑터
- LAN 케이블
- base Pi
- head Pi
- node1 Pi
- node2 Pi
- head Camera Module 3
- 각 Pi 전원

IP:

| 장치 | IP |
| ---- | -- |
| laptop LAN | 192.168.60.2 |
| base eth0 | 192.168.60.1 |
| base bat0 | 192.168.50.1 |
| head bat0 | 192.168.50.10 |
| node1 bat0 | 192.168.50.11 |
| node2 bat0 | 192.168.50.12 |

LAN 연결:

```text
노트북 USB-C LAN 어댑터
  |
LAN 케이블
  |
base Pi eth0
```

테스트 중 노트북 Wi-Fi가 `192.168.50.x`를 잡으면 mesh 대역과 충돌할 수 있다. 유선 테스트 중에는 가능하면 끈다.

```bash
nmcli radio wifi off
```

## 2. 최신 파일 배포

base mesh와 LAN gateway가 살아 있어야 파일 복사가 쉽다. 새로 켠 상태면 3장, 4장을 먼저 하고 다시 돌아와도 된다.

노트북에서 base로:

```bash
cd ~/Projects/HANSEL_MESH
ssh hansel@192.168.60.1 'mkdir -p /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/controller /home/hansel/HANSEL_MESH/scripts /home/hansel/HANSEL_MESH/docs'
scp -r robot controller scripts docs README.md hansel@192.168.60.1:/home/hansel/HANSEL_MESH/
```

base에서 각 Pi로:

```bash
ssh hansel@192.168.50.10 'mkdir -p /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/scripts'
ssh hansel@192.168.50.11 'mkdir -p /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/scripts'
ssh hansel@192.168.50.12 'mkdir -p /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/scripts'

scp -r /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/scripts hansel@192.168.50.10:/home/hansel/HANSEL_MESH/
scp -r /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/scripts hansel@192.168.50.11:/home/hansel/HANSEL_MESH/
scp -r /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/scripts hansel@192.168.50.12:/home/hansel/HANSEL_MESH/
```

## 3. Pi 전원 켜기

추천 순서:

1. base
2. node2
3. node1
4. head

각 Pi 로그인 후 hostname 확인:

```bash
hostname
```

예상값:

```text
base / node2 / node1 / head
```

## 4. Mesh 시작

base에서:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/start_mesh.sh configs/base.env
sudo ./scripts/setup_base_gateway.sh
```

node2에서:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/start_mesh.sh configs/node2.env
```

node1에서:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/start_mesh.sh configs/node1.env
```

head에서:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/start_mesh.sh configs/head.env
sudo ./scripts/setup_mesh_route_to_laptop.sh
```

정상 확인:

```bash
ip -brief addr
```

예상:

- base `eth0`: `192.168.60.1/24`
- base `bat0`: `192.168.50.1/24`
- head `bat0`: `192.168.50.10/24`
- node1 `bat0`: `192.168.50.11/24`
- node2 `bat0`: `192.168.50.12/24`
- `bat0 UNKNOWN`은 정상일 수 있음

## 5. 노트북 route 설정

노트북에서 유선 인터페이스 확인:

```bash
ip -brief addr
```

예전과 같으면 `enx00e04c68070e`를 사용한다.

```bash
cd ~/Projects/HANSEL_MESH
sudo ./scripts/setup_laptop_mesh_routes.sh enx00e04c68070e
```

확인:

```bash
ping -c 4 192.168.60.1
ping -c 4 192.168.50.10
ping -c 4 192.168.50.11
ping -c 4 192.168.50.12
```

## 6. BATMAN 릴레이 확인

base에서:

```bash
ping 192.168.50.10
sudo batctl n
sudo batctl o
```

가까운 상태에서는 head/node1/node2가 모두 direct neighbor로 보일 수 있다.

멀리 배치 테스트:

1. base와 head만 멀리 둬서 ping이 불안정하거나 끊기는지 확인
2. node2를 base와 head 사이 중간에 둔다
3. node1을 head 쪽 중간에 둔다
4. base에서 `ping 192.168.50.10`이 살아나는지 확인
5. base에서 `sudo batctl o` 확인
6. head originator의 selected nexthop이 node1/node2 쪽 MAC이면 실제 릴레이 중

## 7. 기존 제어 프로세스 정리

각 Pi에서 이전 제어 서버가 떠 있으면 정리한다.

```bash
sudo pkill -f Head_control.py
sudo pkill -f Node1_control.py
sudo pkill -f Node2_control.py
sudo pkill -f Node3_control.py
sudo pkill -f mesh_control_server.py
```

head 고개 서보는 pigpio가 있으면 pigpio를 쓰고, 없으면 RPi.GPIO PWM으로 자동 fallback한다. 더 안정적인 서보 펄스를 쓰려면 head에서:

```bash
sudo systemctl enable --now pigpiod
```

## 8. Dry-run 조종 확인

GPIO를 잡기 전 네트워크만 확인한다.

head에서:

```bash
cd ~/HANSEL_MESH
python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head --dry-run
```

노트북에서:

```bash
cd ~/Projects/HANSEL_MESH
python3 controller/mesh_control_client.py --target head
```

입력:

```text
w
a
s
d
x
quit
```

head 로그에 `packet from=('192.168.60.2', ...)`가 보이면 성공.

## 9. 실제 모터 서버 실행

바퀴를 바닥에서 띄운 상태로 시작한다.

head에서:

```bash
cd ~/HANSEL_MESH
sudo python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head
```

node1에서:

```bash
cd ~/HANSEL_MESH
sudo python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role node1
```

node2에서:

```bash
cd ~/HANSEL_MESH
sudo python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role node2
```

서버 시작 실패 시 확인:

- `sudo`로 실행했는가
- `RPi.GPIO`가 설치되어 있는가
- encoder A/B 핀 배선이 맞는가
- GPIO 핀 중복이 없는가
- 이전 제어 프로세스가 아직 GPIO를 잡고 있지 않은가

## 10. 실제 조종 테스트

처음은 저속:

```bash
cd ~/Projects/HANSEL_MESH
python3 controller/mesh_control_client.py --target all --speed 0.4 --live
```

키:

| 키 | 동작 |
| -- | ---- |
| w | head/node1/node2 모두 같은 목표 CPS로 전진 |
| s | head/node1/node2 모두 같은 목표 CPS로 후진 |
| a | head만 left spin, node1/node2 stop |
| d | head만 right spin, node1/node2 stop |
| q | head forward_left, node1/node2 slow_forward |
| e | head forward_right, node1/node2 slow_forward |
| z | head backward_left, node1/node2 slow_backward |
| c | head backward_right, node1/node2 slow_backward |
| x 또는 space | stop |
| u | head servo up |
| j | head servo down |
| k | head servo center |
| 1 | detach_press, target all에서는 안전상 전송 안 함 |
| 2 | detach_rest, target all에서는 안전상 전송 안 함 |
| Ctrl+C | stop 보내고 종료 |

현재 기본값은 `w`가 물리 전진, `s`가 물리 후진이 되도록 주행 모터 방향을 reverse 처리한다. 특정 모터가 다시 반대로 돌면 서버 실행 전 환경변수로 보정:

```bash
HANSEL_LEFT_REVERSE=no sudo -E python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head
HANSEL_RIGHT_REVERSE=no sudo -E python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head
```

방향 확인 후 최대속도:

```bash
python3 controller/mesh_control_client.py --target all --live
```

## 11. 카메라 영상 테스트

노트북에서 수신 먼저:

```bash
cd ~/Projects/HANSEL_MESH
./scripts/receive_camera_stream.sh 5600
```

head에서 카메라 busy 정리:

```bash
sudo pkill -f rpicam-vid
sudo pkill -f libcamera-vid
sudo pkill -f rpicam-hello
sudo pkill -f libcamera-hello
sudo pkill -f rpicam-still
sudo pkill -f libcamera-still
```

head에서 저대역폭 송신:

```bash
cd ~/HANSEL_MESH
WIDTH=320 HEIGHT=240 FPS=10 BITRATE=600000 ~/HANSEL_MESH/scripts/start_camera_stream.sh 192.168.60.2 5600
```

가까운 거리에서 안정적이면:

```bash
WIDTH=640 HEIGHT=480 FPS=15 BITRATE=1200000 ~/HANSEL_MESH/scripts/start_camera_stream.sh 192.168.60.2 5600
```

카메라 단독 확인:

```bash
rpicam-vid -t 2000 --nopreview --width 640 --height 480 --framerate 15 --codec h264 --inline --bitrate 1200000 -o /tmp/test.h264
ls -lh /tmp/test.h264
```

## 12. 구동 + 카메라 동시 테스트

1. 노트북에서 camera receiver 실행
2. head에서 camera sender 실행
3. head/node1/node2에서 motor server 실행
4. 노트북에서 live control 실행
5. `w`로 직진 확인
6. `q/e`로 head 조향 + node 저속 추종 확인
7. base에서 `sudo batctl o`로 경로 확인
8. relay 배치에서 영상 끊김/조종 지연 기록

기록할 것:

- 배치 거리
- `batctl o` selected nexthop
- ping packet loss
- 영상 지연 체감
- 조종 반응
- 모터 방향 보정 필요 여부
- PID 로그에서 target/measured CPS 차이

## 13. 종료

노트북 조종:

```text
Ctrl+C
```

노트북 카메라 수신:

```text
Ctrl+C
```

head 카메라 송신:

```text
Ctrl+C
```

각 Pi 종료 순서:

```text
head -> node1 -> node2 -> base
```

각 Pi에서:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/stop_mesh.sh
sudo poweroff
```

base는 마지막에 끈다.
