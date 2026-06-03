# Next Field Test Checklist

다음 테스트를 시작할 때 이 순서대로 진행한다.

## 1. 장비 배치

```text
노트북
  |
LAN
  |
base
  |
node2
  |
node1
  |
head
```

IP 기준:

| 장치 | IP |
| ---- | -- |
| laptop LAN | 192.168.60.2 |
| base eth0 | 192.168.60.1 |
| base bat0 | 192.168.50.1 |
| head bat0 | 192.168.50.10 |
| node1 bat0 | 192.168.50.11 |
| node2 bat0 | 192.168.50.12 |

## 2. Base 시작

Base에서:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/start_mesh.sh configs/base.env
sudo ./scripts/setup_base_gateway.sh
```

확인:

```bash
ip -brief addr
sudo batctl n
sudo batctl o
```

정상 기준:

- `eth0`에 `192.168.60.1/24`
- `bat0`에 `192.168.50.1/24`

## 3. Node 시작

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

각 node에서 확인:

```bash
ip -brief addr
sudo batctl n
sudo batctl o
```

정상 기준:

- node1 `bat0`: `192.168.50.11/24`
- node2 `bat0`: `192.168.50.12/24`
- `wlan0`는 ad-hoc/IBSS 상태
- `bat0` 상태가 `UNKNOWN`이어도 IP가 있으면 정상

## 4. Head 시작

Head에서:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/start_mesh.sh configs/head.env
sudo ./scripts/setup_mesh_route_to_laptop.sh
```

확인:

```bash
ip -brief addr
ping -c 4 192.168.50.1
ping -c 4 192.168.60.2
```

## 5. Laptop route

노트북에서 유선 인터페이스 이름 확인:

```bash
ip -brief addr
```

예전과 같으면 `enx00e04c68070e`를 사용한다.

```bash
cd ~/Projects/HANSEL_MESH
sudo ./scripts/setup_laptop_mesh_routes.sh enx00e04c68070e
```

노트북 Wi-Fi가 `192.168.50.x` 대역이면 테스트 중에는 꺼둔다.

```bash
nmcli radio wifi off
```

확인:

```bash
ping -c 4 192.168.60.1
ping -c 4 192.168.50.10
```

## 6. Relay 검증

Base에서:

```bash
ping 192.168.50.10
sudo batctl o
```

가까운 상태에서는 `Nexthop`이 목적지 MAC과 같을 수 있다. 이는 직접 연결이 가능하다는 뜻이다.

진짜 relay 검증은 다음 순서로 한다.

1. base와 head만 멀리 떨어뜨려 직접 ping이 불안정하거나 끊기는지 확인
2. node2를 base와 head 사이 중간에 둔다
3. node1을 head 쪽 중간에 둔다
4. base에서 `ping 192.168.50.10`이 살아나는지 확인
5. base에서 `sudo batctl o`를 실행해 head originator의 `*` 줄 `Nexthop`이 node1/node2 쪽 MAC으로 바뀌는지 확인

성공 기준:

- 직접 연결이 어려운 거리에서 node 배치 후 ping 복구
- `batctl o`에서 head 경로의 selected nexthop이 node 쪽으로 표시
- 조종 명령 도착
- 카메라 영상 유지

## 7. 조종 테스트

실제 줄 전체 주행을 테스트하려면 Head, node1, node2에서 모두 서버를 켠다.

Head에서:

```bash
sudo python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head
```

node1에서:

```bash
sudo python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role node1
```

node2에서:

```bash
sudo python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role node2
```

노트북에서:

```bash
cd ~/Projects/HANSEL_MESH
python3 controller/mesh_control_client.py --target all --live
```

Live mode 입력:

```text
w
a
s
d
x
```

Head 로그에 `packet from=('192.168.60.2', ...)`가 보이면 노트북에서 Head까지 end-to-end 도착한 것이다.

조향 원칙:

- `w`: head/node1/node2 모두 같은 목표 CPS로 전진
- `s`: head/node1/node2 모두 같은 목표 CPS로 후진
- `a/d`: head만 제자리 조향, node1/node2는 정지
- `q/e/z/c`: head만 RPM 차이로 곡선 조향, node1/node2는 느린 직진/후진

모터 없이 네트워크만 확인하려면 Head에서 서버를 `--dry-run`으로 실행한다.

```bash
python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head --dry-run
```

## 8. 카메라 테스트

카메라가 busy이면 Head에서 먼저 정리한다.

```bash
sudo pkill -f rpicam-vid
sudo pkill -f libcamera-vid
sudo pkill -f rpicam-hello
sudo pkill -f libcamera-hello
sudo pkill -f rpicam-still
sudo pkill -f libcamera-still
```

노트북에서 수신:

```bash
cd ~/Projects/HANSEL_MESH
./scripts/receive_camera_stream.sh 5600
```

Head에서 송신:

```bash
rpicam-vid -t 0 --nopreview --width 320 --height 240 --framerate 10 --codec h264 --inline --bitrate 600000 -o udp://192.168.60.2:5600
```

가까운 거리에서 안정적이면 다음 설정으로 올린다.

```bash
rpicam-vid -t 0 --nopreview --width 640 --height 480 --framerate 15 --codec h264 --inline --bitrate 1200000 -o udp://192.168.60.2:5600
```

## 9. 종료

노트북 수신/조종 창:

```text
Ctrl + C
```

각 Pi는 head, node1, node2, base 순서로 종료한다.

```bash
cd ~/HANSEL_MESH
sudo ./scripts/stop_mesh.sh
sudo poweroff
```

Base는 마지막에 끈다.
