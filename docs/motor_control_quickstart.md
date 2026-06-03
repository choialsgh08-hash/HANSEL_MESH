# Motor Control Quickstart

이 문서는 BATMAN Mesh 위에서 실제 모터/엔코더 제어를 실행하는 순서다.

## 1. 구조

```text
노트북 controller/mesh_control_client.py
  |
base routing only
  |
BATMAN Mesh
  |
head/node1/node2 robot/mesh_control_server.py
  |
robot/motor_driver.py
  |
GPIO motor driver + encoder + servo
```

Base는 명령 내용을 해석하지 않는다. 노트북이 각 unit의 mesh IP로 직접 UDP 명령을 보낸다.

## 2. 기존 제어 프로세스 정리

각 Pi에서 이전 AP 방식 제어 서버가 떠 있으면 먼저 끈다.

```bash
sudo pkill -f Head_control.py
sudo pkill -f Node1_control.py
sudo pkill -f Node2_control.py
sudo pkill -f Node3_control.py
sudo pkill -f mesh_control_server.py
```

## 3. 파일 배포

노트북에서 Base로:

```bash
cd ~/Projects/HANSEL_MESH
ssh hansel@192.168.60.1 'mkdir -p /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/controller'
scp -r robot controller hansel@192.168.60.1:/home/hansel/HANSEL_MESH/
```

Base에서 Head/node1/node2로:

```bash
scp -r /home/hansel/HANSEL_MESH/robot hansel@192.168.50.10:/home/hansel/HANSEL_MESH/
scp -r /home/hansel/HANSEL_MESH/robot hansel@192.168.50.11:/home/hansel/HANSEL_MESH/
scp -r /home/hansel/HANSEL_MESH/robot hansel@192.168.50.12:/home/hansel/HANSEL_MESH/
```

## 4. Dry-run 확인

GPIO를 잡기 전에 네트워크/명령만 확인하려면 `--dry-run`을 붙인다.

Head에서:

```bash
python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head --dry-run
```

노트북에서:

```bash
cd ~/Projects/HANSEL_MESH
python3 controller/mesh_control_client.py --target head
```

`w`, `a`, `s`, `d`, `x`를 입력했을 때 Head 로그가 찍히면 dry-run 성공이다.

## 5. 실제 모터 서버 실행

GPIO 접근 때문에 실제 모터 제어는 `sudo`로 실행한다.

Head에서:

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

Head 고개 서보를 쓰려면 pigpio daemon이 필요할 수 있다.

```bash
sudo systemctl enable --now pigpiod
```

pigpio가 없어도 주행 모터와 detach servo는 계속 동작한다.

## 6. 조종

줄 전체를 같이 움직일 때:

```bash
cd ~/Projects/HANSEL_MESH
python3 controller/mesh_control_client.py --target all --live
```

Head만 테스트할 때:

```bash
python3 controller/mesh_control_client.py --target head --speed 0.5 --live
```

Live mode 키:

| 키 | 명령 |
| -- | ---- |
| w | head/node1/node2 모두 forward, 같은 목표 CPS |
| s | head/node1/node2 모두 backward, 같은 목표 CPS |
| a | head left spin, node1/node2 stop |
| d | head right spin, node1/node2 stop |
| q | head forward_left, node1/node2 slow_forward |
| e | head forward_right, node1/node2 slow_forward |
| z | head backward_left, node1/node2 slow_backward |
| c | head backward_right, node1/node2 slow_backward |
| x 또는 space | stop |
| u | head servo up |
| j | head servo down |
| f | head front motor forward |
| v | head front motor stop |
| 1 | detach_press, 단 `--target all`에서는 안전상 전송 안 함 |
| Ctrl+C | stop 보내고 종료 |

Line mode도 가능하다.

```bash
python3 controller/mesh_control_client.py --target head --speed 0.5
```

Line mode 명령:

```text
w, a, s, d, x
fl, fr, bl, br
hu, hd
front, front_stop
detach
t head
t node1
t node2
t all
quit
```

## 7. 튜닝 환경변수

현재 기본값은 모든 주행 모터와 head front motor가 reverse 상태다. 즉 `w`가 물리 전진, `s`가 물리 후진이 되도록 맞춰져 있다.

특정 모터가 다시 반대로 돌면 해당 Pi에서 서버 실행 전 환경변수를 `no` 또는 `yes`로 바꿔 보정한다.

```bash
HANSEL_LEFT_REVERSE=no sudo -E python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head
HANSEL_RIGHT_REVERSE=no sudo -E python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head
```

속도를 낮춰 시작:

```bash
HANSEL_SPEED_SCALE=0.45 sudo -E python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head
```

Node 기본 속도만 낮추기:

```bash
HANSEL_NODE_SPEED_SCALE=0.5 sudo -E python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role node1
```

기본값은 head/node 모두 `1.0`이다. 즉 `w`만 누르면 모든 유닛이 같은 목표 CPS로 최대속도를 낸다. Head 조향 중에만 node 쪽 명령이 `slow_forward` 또는 `slow_backward`로 낮아진다.

앞쪽 Head DC모터를 임시로 끄기:

```bash
HANSEL_FRONT_MOTOR_ENABLED=no sudo -E python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head
```

PID 튜닝:

```bash
HANSEL_KP_LEFT=0.035 HANSEL_KI_LEFT=0.015 HANSEL_KP_RIGHT=0.035 HANSEL_KI_RIGHT=0.015 sudo -E python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head
```

기본값은 이전 GitHub 제어 코드의 엔코더 PID 값을 가져온다.

## 8. 안전 기준

- 첫 실제 주행은 `--speed 0.4` 이하로 시작한다.
- 바퀴가 공중에 뜬 상태에서 방향을 먼저 확인한다.
- 방향이 반대면 배선을 바꾸기 전에 `HANSEL_LEFT_REVERSE`, `HANSEL_RIGHT_REVERSE`로 보정한다.
- 서버 watchdog은 기본 0.5초다. 명령이 끊기면 자동으로 `stop`을 실행한다.
- node1/node2는 서버에서도 조향 명령을 직진/정지로 변환한다. 실수로 node에 `left/right`가 들어가도 회전 조향하지 않는다.
