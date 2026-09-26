# Video-First Quality Control and Auto Detach Runbook

이 문서는 카메라 영상이 끊기기 전에 속도를 줄이고, 위험 상태가 지속되면 node를 순차적으로 분리하기 위한 실행 절차다.

핵심 기준은 `원격 조종 가능 여부 = 카메라 영상 품질`이다. UDP 제어 패킷은 영상보다 훨씬 작아서, 영상이 이미 깨져도 조종 명령만 살아있는 상황이 생길 수 있다. 그래서 자동 분리 판단은 RSSI 하나만 보지 않고 수신 영상 품질을 1순위로 본다.

## 판단 값

1. 영상 수신 품질

- `fps_ratio`: 실제 수신 FPS / 목표 FPS
- `err_rate`: ffmpeg H.264 decode error per second
- `drop_rate`: drop frame per second
- `video stale`: 일정 시간 새 영상 샘플이 없으면 위험

2. 네트워크 품질

- `RTT`: laptop/base에서 head까지 ping 왕복 시간
- `packet loss`: ping 손실률
- `BATMAN TQ`: `batctl o`의 selected TQ. 255에 가까울수록 좋다.

3. 현재 기본 임계값

- GOOD: 정상 주행, camera profile 0
- WARN: 3초 이상 영상/네트워크가 나빠짐. 속도 `0.35`로 제한, camera profile 1
- DANGER: 1.5초 이상 위험. 즉시 stop, camera profile 2, `node2 -> node1` 순서로 분리 절차 시작. 기본 모드에서는 각 유닛의 실제 분리를 운전자가 확인해야 완료된다.

## 카메라 프로파일

영상 전송은 기본적으로 `RTP/H.264 over UDP`를 쓴다.

```text
head rpicam-vid -> H.264 -> RTP packetizer -> UDP 5600 -> laptop video_probe/receiver
```

조종 명령은 version/session/sequence/TTL/ACK가 포함된 UDP 7000 프로토콜을 쓴다. 영상 포트와 제어 포트가 다르기 때문에 서로 충돌하지 않는다.

`scripts/start_camera_stream.sh`는 `PROFILE` 값으로 해상도/비트레이트를 바꾼다.

```bash
PROFILE=0  # high:     640x480@15, 1.2 Mbps
PROFILE=1  # medium:   480x360@12, 0.8 Mbps
PROFILE=2  # low:      320x240@10, 0.6 Mbps
PROFILE=3  # survival: 320x240@8,  0.4 Mbps
```

자동 프로파일 변경은 기존 UDP 제어 채널을 쓴다. 노트북이
`camera_profile` 명령을 head로 보내면, head의 `mesh_control_server.py`는
root 소유 `/run/hansel-camera-profile` 토큰만 갱신하고
`hansel-camera.service` 재시작을 예약한다. 목적지와 전송 방식은 관리자가
`/etc/hansel-mesh/camera.env`에 설정한 값을 유지한다. 클라이언트는 적용
ACK를 받은 뒤에만 전환을 확정하며, 거부 또는 timeout이면 제어 루프를 막지
않고 재시도한다.

## 필요한 패키지

노트북:

```bash
sudo apt install -y ffmpeg
```

head는 RTP packetizer가 하나 필요하다. 둘 중 하나만 있으면 된다.

```bash
sudo apt install -y ffmpeg
```

또는:

```bash
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```

RTP가 현장에서 바로 안 되면 예전 raw UDP 방식으로 되돌릴 수 있다.

head:

```bash
sudoedit /etc/hansel-mesh/camera.env
# CAMERA_TRANSPORT="raw", PROFILE="1"로 변경
sudo rm -f /run/hansel-camera-profile
sudo systemctl restart hansel-camera.service
```

노트북:

```bash
CAMERA_TRANSPORT=raw ./scripts/receive_camera_stream.sh 5600
python3 monitor/video_probe.py --transport raw --port 5600 --target-fps 12 --log video_quality.jsonl
```

## 실행 순서

1. 노트북과 base 유선 연결

노트북에서:

```bash
cd ~/Projects/HANSEL_MESH
sudo ./scripts/setup_laptop_mesh_routes.sh enx00e04c68070e
ping -c 4 192.168.60.1
ping -c 4 192.168.50.10
```

2. base gateway 설정

base에서:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/setup_base_gateway.sh
```

3. head/node 서버 실행

각 Pi에서:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/start_mesh.sh configs/head.env   # head에서만
sudo ./scripts/start_mesh.sh configs/node1.env  # node1에서만
sudo ./scripts/start_mesh.sh configs/node2.env  # node2에서만
sudo ./scripts/setup_mesh_route_to_laptop.sh
sudo python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head --host 192.168.50.10 --allow-source 192.168.60.2/32
```

node1/node2는 마지막 줄의 role만 바꾼다.

```bash
sudo python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role node1 --host 192.168.50.11 --allow-source 192.168.60.2/32
sudo python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role node2 --host 192.168.50.12 --allow-source 192.168.60.2/32
```

4. head 카메라 시작

head에서:

```bash
cd ~/HANSEL_MESH
sudo install -d -m 0755 /etc/hansel-mesh
sudo test -f /etc/hansel-mesh/camera.env || \
  sudo install -m 0644 configs/camera.env.example /etc/hansel-mesh/camera.env
sudoedit /etc/hansel-mesh/camera.env
# CAMERA_ENABLED="yes", CAMERA_DEST_IP="192.168.60.2",
# CAMERA_TRANSPORT="rtp", PROFILE="0" 확인
sudo ./scripts/enable_mesh_autostart.sh head --with-camera
sudo rm -f /run/hansel-camera-profile
sudo systemctl restart hansel-camera.service
```

기본값은 `CAMERA_TRANSPORT=rtp`다. 시작부터 안정성을 우선하면 profile 1로 시작해도 된다.

```bash
sudoedit /etc/hansel-mesh/camera.env
# PROFILE="1"로 변경
sudo rm -f /run/hansel-camera-profile
sudo systemctl restart hansel-camera.service
```

5. 노트북에서 영상 수신 + 품질 로그 생성

노트북에서 새 터미널:

```bash
cd ~/Projects/HANSEL_MESH
python3 monitor/video_probe.py --transport rtp --port 5600 --target-fps 15 --log video_quality.jsonl
```

화면 표시 없이 측정만 할 때:

```bash
python3 monitor/video_probe.py --no-display --transport rtp --port 5600 --target-fps 15 --log video_quality.jsonl
```

6. 노트북에서 품질 감시 조종 시작

노트북에서 또 다른 터미널:

```bash
cd ~/Projects/HANSEL_MESH
python3 controller/mesh_control_client.py --target all --speed 1.0 --live \
  --quality-log video_quality.jsonl \
  --quality-target-fps 15 \
  --quality-base-ssh hansel@192.168.60.1 \
  --quality-warn-speed 0.35 \
  --camera-transport rtp \
  --auto-detach \
  --detach-order node2,node1
```

`--auto-detach`를 빼면 분리 없이 stop/속도 제한/카메라 프로파일 변경만 테스트할 수 있다.

조종 시작 시에는 선택된 target마다 `stop` ACK를 먼저 받아 새 제어 세션의
monotonic 시간 기준을 만든다. 첫 품질 판정 전 `NOT_READY`, 감시 예외 `ERROR`,
갱신 중단 `STALE` 상태에서는 속도 cap이 0이며 이동 명령을 보내지 않는다.
영상 샘플 누락이나 유효한 FPS 부재도 `DANGER`로 처리한다. 단, 시작 시 로그가
아직 없다는 이유만으로 유닛을 떨어뜨리지 않도록 자동분리는 한 번 이상
`GOOD` 또는 `WARN`의 사용 가능한 영상 샘플을 본 뒤에만 무장된다.

```bash
python3 controller/mesh_control_client.py --target all --speed 1.0 --live \
  --quality-log video_quality.jsonl \
  --quality-target-fps 15 \
  --quality-base-ssh hansel@192.168.60.1 \
  --quality-warn-speed 0.35 \
  --camera-transport rtp
```

## 기대 동작

- 영상이 정상일 때: `status=GOOD`, profile 0, 속도 1.0
- 영상 FPS가 낮아지거나 decode error가 늘 때: `status=WARN`, profile 1, 속도 0.35
- 영상이 심하게 깨지거나 ping loss/RTT/TQ가 위험할 때: `status=DANGER`, stop 전송, profile 2, node2부터 detach
- node2가 떨어진 뒤에는 조종 대상에서 node2를 제거해서 head/node1만 움직인다.

## 튜닝 포인트

- 영상이 너무 빨리 낮아지면 `--quality-warn-speed`만 조절하지 말고 `controller/quality_supervisor.py`의 `warn_hold_s`, `danger_hold_s`를 늘린다.
- 너무 늦게 분리되면 `fps_danger_ratio`, `err_danger_rate`, `drop_danger_rate`, `rtt_danger_ms`, `loss_danger_pct`, `tq_danger`를 더 보수적으로 잡는다.
- 카메라 화면이 항상 먼저 깨진다면 `PROFILE=0` 시작값 자체를 `PROFILE=1`로 낮춰도 된다.
- `--quality-base-ssh`는 비밀번호 입력을 기다리지 않도록 `BatchMode`로 실행된다. SSH key가 없으면 TQ만 빠지고, 영상 품질과 ping 판단은 계속 동작한다.
- 각 Pi의 `start_mesh.sh`는 기본적으로 `iw dev wlan0 set power_save off`를 수행한다.
- multi-hop 경로를 더 빨리 쓰게 하고 싶으면 각 Pi의 env에 `BATMAN_HOP_PENALTY=15`를 넣고 재시작한다. 기본값보다 낮을수록 hop을 덜 불리하게 본다.
- 이동 중 경로 전환 반응을 빠르게 보고 싶으면 `BATMAN_ORIG_INTERVAL=500`을 실험한다. 단, 너무 낮추면 관리 트래픽이 늘어난다.

## 안전 종료

노트북 컨트롤러에서 `Ctrl-C`를 누르면 stop을 한 번 더 보낸다.

각 Pi에서:

```bash
# head에서만 먼저 실행
sudo systemctl stop hansel-camera.service
sudo pkill -f mesh_control_server.py
sudo ./scripts/stop_mesh.sh
sudo shutdown now
```
