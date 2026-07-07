# Full Drive + Servo + Monitor Test Runbook

이 문서는 서보까지 포함한 전체 구동 테스트와 monitor 실행 방법을 정리한다.

## Monitor 구성 이해

### 1. video_probe.py

노트북에서 실행한다. head가 보내는 RTP/H.264 카메라 영상을 직접 수신하고, 동시에 영상 품질을 측정한다.

측정 값:

- `fps`: 수신/디코딩 FPS
- `fps_ratio`: 수신 FPS / 기대 FPS
- `err_rate`: H.264 decode error per second
- `drop_rate`: drop frame per second
- `bitrate_kbps`: 수신 bitrate 추정

이 파일은 자동 감속/자동 분리 판단에 가장 중요하다.

### 2. quality_supervisor.py

노트북 조종 클라이언트 내부에서 사용된다. `video_probe.py`가 만든 `video_quality.jsonl`을 읽고 다음 상태를 판단한다.

- `GOOD`: 정상
- `WARN`: 속도 제한 + 카메라 profile 낮춤
- `DANGER`: stop + 필요 시 자동 분리

직접 실행해서 상태만 볼 수도 있다.

```bash
python3 controller/quality_supervisor.py --video-log video_quality.jsonl --target-fps 12
```

### 3. metrics_agent.py

각 Pi에서 실행한다. 이 노드가 직접 보는 mesh 상태를 수집한다.

수집 값:

- `iw dev wlan0 station dump`: RSSI, bitrate
- `batctl n`: 직접 neighbor
- `batctl o`: BATMAN originator / TQ / nexthop
- `ip neigh show dev bat0`: MAC과 IP 매핑
- `ping`: end-to-end RTT/loss

이건 자동 조종에 필수는 아니고, dashboard 시각화용이다.

### 4. dashboard.py

노트북에서 실행한다. UDP `:7100`으로 `metrics_agent.py`와 `video_probe.py` 데이터를 받고, HTTP `:8080`으로 웹 대시보드를 띄운다.

브라우저:

```text
http://127.0.0.1:8080
```

인터넷이 없어서 CDN 라이브러리가 안 떠도, 기본 fallback 화면으로 최소 상태는 보이게 되어 있다.

## 테스트 전 최신 코드 배포

노트북에서 base로:

```bash
cd ~/Projects/HANSEL_MESH
scp -r configs controller docs monitor robot scripts services README.md \
  hansel@192.168.60.1:/home/hansel/HANSEL_MESH/
```

base에서 전체 유닛으로:

```bash
cd ~/HANSEL_MESH
./scripts/deploy_from_base.sh
```

## Mesh 확인

노트북에서:

```bash
ping -c 3 192.168.60.1
ping -c 3 192.168.50.10
ping -c 3 192.168.50.11
ping -c 3 192.168.50.12
```

base에서:

```bash
sudo batctl n
sudo batctl o
```

## 서버 실행

각 유닛에서 motor server를 실행한다.

서보 테스트 전에 head에서 pigpio 상태를 먼저 확인한다.

```bash
python3 -c "import pigpio; pi=pigpio.pi(); print('pigpio connected=', pi.connected); pi.stop()"
systemctl is-active pigpiod
```

기대값:

```text
pigpio connected= True
active
```

head:

```bash
cd ~/HANSEL_MESH
sudo python3 robot/mesh_control_server.py --role head
```

head에서 기대 로그:

```text
[head] pigpio connected; using hardware-timed PWM for detach servo
[head] pigpio connected; using hardware-timed PWM for head servo
```

node1:

```bash
cd ~/HANSEL_MESH
sudo python3 robot/mesh_control_server.py --role node1
```

node2:

```bash
cd ~/HANSEL_MESH
sudo python3 robot/mesh_control_server.py --role node2
```

base에서 원격으로 한 번에 터미널을 열어 실행해도 된다. 각 명령은 별도 터미널에서 실행해야 로그를 보기 쉽다.

```bash
ssh -t hansel@192.168.50.10 'cd ~/HANSEL_MESH && sudo python3 robot/mesh_control_server.py --role head'
ssh -t hansel@192.168.50.11 'cd ~/HANSEL_MESH && sudo python3 robot/mesh_control_server.py --role node1'
ssh -t hansel@192.168.50.12 'cd ~/HANSEL_MESH && sudo python3 robot/mesh_control_server.py --role node2'
```

## 카메라 RTP 시작

head에서 새 터미널:

```bash
cd ~/HANSEL_MESH
CAMERA_TRANSPORT=rtp bash scripts/restart_camera_profile.sh 1 192.168.60.2 5600
```

## 필수 monitor: 영상 품질 측정

노트북에서:

```bash
cd ~/Projects/HANSEL_MESH
python3 monitor/video_probe.py --transport rtp --port 5600 --target-fps 12 \
  --log video_quality.jsonl
```

dashboard에도 영상 품질을 보내려면:

```bash
python3 monitor/video_probe.py --transport rtp --port 5600 --target-fps 12 \
  --log video_quality.jsonl \
  --send 127.0.0.1:7100
```

화면 없이 품질만 측정:

```bash
python3 monitor/video_probe.py --no-display --transport rtp --port 5600 \
  --target-fps 12 --log video_quality.jsonl --send 127.0.0.1:7100
```

## 선택 monitor: 대시보드

노트북에서:

```bash
cd ~/Projects/HANSEL_MESH
python3 monitor/dashboard.py --http-port 8080 --udp-port 7100 \
  --log monitor_session.jsonl
```

브라우저:

```text
http://127.0.0.1:8080
```

대시보드 상단에는 자동분리 판단과 같은 기준의 실시간 품질 카드가 뜬다.

- `품질 판단`: GOOD / TRANSIENT / WARN / DANGER
- `수신 FPS`: 목표 FPS 대비 현재 FPS
- `Decode Error`: H.264 decode error per second
- `Frame Drop`: drop frame per second
- `예상 대응`: camera profile과 speed cap

그래프는 다음을 실시간으로 그린다.

- Mesh topology: node 간 링크, RSSI, BATMAN TQ
- RTT chart: ping 기반 종단 지연
- RSSI chart: 링크별 신호 세기
- Video quality chart: FPS, decode error, drop, WARN/DANGER 기준선
- Quality timeline: GOOD/WARN/DANGER 상태와 bitrate
- Correlation chart: 최저 RSSI와 영상 error/s 상관관계

각 Pi에서 metrics agent를 실행하면 dashboard에 RSSI/TQ/RTT가 들어온다.

base:

```bash
cd ~/HANSEL_MESH
python3 monitor/metrics_agent.py --self base --loop --interval 3 \
  --ping head node1 node2 \
  --send 192.168.60.2:7100
```

head:

```bash
cd ~/HANSEL_MESH
python3 monitor/metrics_agent.py --self head --loop --interval 3 \
  --ping base \
  --send 192.168.60.2:7100
```

node1:

```bash
cd ~/HANSEL_MESH
python3 monitor/metrics_agent.py --self node1 --loop --interval 3 \
  --ping base head \
  --send 192.168.60.2:7100
```

node2:

```bash
cd ~/HANSEL_MESH
python3 monitor/metrics_agent.py --self node2 --loop --interval 3 \
  --ping base head \
  --send 192.168.60.2:7100
```

만약 RSSI/TQ가 안 보이면 `sudo`로 실행한다.

```bash
sudo python3 monitor/metrics_agent.py --self head --loop --interval 3 \
  --ping base \
  --send 192.168.60.2:7100
```

base에서 원격 실행할 때는 이렇게 쓴다.

```bash
cd ~/HANSEL_MESH
python3 monitor/metrics_agent.py --self base --loop --interval 3 \
  --ping head node1 node2 \
  --send 192.168.60.2:7100

ssh -t hansel@192.168.50.10 'cd ~/HANSEL_MESH && python3 monitor/metrics_agent.py --self head --loop --interval 3 --ping base --send 192.168.60.2:7100'
ssh -t hansel@192.168.50.11 'cd ~/HANSEL_MESH && python3 monitor/metrics_agent.py --self node1 --loop --interval 3 --ping base head --send 192.168.60.2:7100'
ssh -t hansel@192.168.50.12 'cd ~/HANSEL_MESH && python3 monitor/metrics_agent.py --self node2 --loop --interval 3 --ping base head --send 192.168.60.2:7100'
```

## 서보 단독 테스트

head 고개 서보:

```bash
cd ~/Projects/HANSEL_MESH
python3 controller/mesh_control_client.py --target head --repeat 1
```

프롬프트에서:

```text
hc      # center
hu      # up 1 degree
hd      # down 1 degree
hmin    # min angle
hmax    # max angle
quit
```

분리 서보는 물리 연결 순서에 맞춰 수동 키로도 테스트할 수 있다.

```text
1  # head GPIO6 서보 동작, node1 분리
2  # node1 GPIO6 서보 동작, node2 분리
3  # node2 GPIO6 서보 동작, node3 분리
```

각 키는 먼저 분리될 node에 `relay_hold`를 보내서 주행을 잠그고, live mode의 `all` 주행 대상에서 그 node를 제거한다. 그래서 분리된 유닛은 따라 움직이지 않고 그 위치에서 BATMAN relay로 남는다.

개별 유닛의 GPIO6 서보만 직접 움직이고 싶으면 반드시 타겟을 하나로 지정해서 테스트한다. `--target all`에서는 안전상 skip된다.

node2 분리 서보:

```bash
python3 controller/mesh_control_client.py --target node2 --repeat 1
```

프롬프트에서:

```text
detach
detach_rest
quit
```

`detach`는 press 각도로 이동한 뒤 유지된다. 원위치 복귀가 필요할 때만 `detach_rest`를 따로 보낸다.

## 저속 전체 구동 테스트

처음에는 `--speed 0.35`로 시작한다.

```bash
cd ~/Projects/HANSEL_MESH
python3 controller/mesh_control_client.py --target all --speed 0.35 --live \
  --quality-log video_quality.jsonl \
  --quality-target-fps 12 \
  --quality-warn-speed 0.25 \
  --auto-camera-profile \
  --camera-transport rtp
```

키:

- `w`: 전체 전진
- `s`: 전체 후진
- `a/d`: head 제자리 조향, nodes stop
- `q/e`: head 곡선 조향, nodes slow forward
- `z/c`: head 후진 곡선 조향, nodes slow backward
- `x` 또는 space: stop
- `u/j/k`: head servo up/down/center
- `f/v`: head front motor forward/stop
- `1`: head GPIO6 서보 동작, node1 분리
- `2`: node1 GPIO6 서보 동작, node2 분리
- `3`: node2 GPIO6 서보 동작, node3 분리

수동 분리 키를 누른 뒤에는 해당 node가 live mode의 `all` 주행 대상에서 빠지고, 서버에서도 주행 명령을 무시한다. 다시 움직이게 하려면 해당 target에 `drive_enable`을 보낸다.

자동 분리는 영상/구동이 안정적으로 확인된 뒤에만 켠다. `--detach-order node2,node1`은 "node2를 분리하려면 node1 서보", "node1을 분리하려면 head 서보"를 움직이는 의미다.

```bash
python3 controller/mesh_control_client.py --target all --speed 0.6 --live \
  --quality-log video_quality.jsonl \
  --quality-target-fps 12 \
  --quality-warn-speed 0.35 \
  --auto-camera-profile \
  --camera-transport rtp \
  --auto-detach \
  --detach-order node2,node1
```

## 문제 시 fallback

RTP 영상이 안 뜨면 raw UDP로 비교한다.

head:

```bash
CAMERA_TRANSPORT=raw bash ~/HANSEL_MESH/scripts/restart_camera_profile.sh 1 192.168.60.2 5600
```

노트북:

```bash
python3 monitor/video_probe.py --transport raw --port 5600 --target-fps 12 \
  --log video_quality.jsonl
```

## 안전 종료

노트북 조종 클라이언트에서 `Ctrl-C`를 누르면 stop을 보낸다.

각 Pi:

```bash
sudo pkill -f mesh_control_server.py
sudo pkill -f metrics_agent.py
sudo pkill -f rpicam-vid
sudo pkill -f libcamera-vid
```
