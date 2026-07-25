# Camera And Control Quickstart

## 1. 전제

카메라 영상과 조종은 Wi-Fi 동글 없이 진행한다.

```text
구조자 PC 192.168.60.2
  |
base eth0 192.168.60.1
base bat0 192.168.50.1
  |
BATMAN Mesh
  |
head bat0 192.168.50.10
```

중간 `node1`, `node2`는 애플리케이션 relay가 아니라 BATMAN-adv 릴레이로만 동작한다.

테스트 중 노트북 Wi-Fi가 `192.168.50.x` 대역이면 Mesh 대역과 충돌할 수 있다. 가능하면 유선 테스트 중에는 노트북 Wi-Fi를 끈다.

```bash
nmcli radio wifi off
```

## 2. 파일 배포

노트북에서 Base로 먼저 보낸다.

```bash
cd ~/Projects/HANSEL_MESH
ssh hansel@192.168.60.1 'mkdir -p /home/hansel/HANSEL_MESH/scripts /home/hansel/HANSEL_MESH/controller /home/hansel/HANSEL_MESH/common /home/hansel/HANSEL_MESH/configs /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/services'
scp -r common configs controller robot scripts services docs/camera_control_quickstart.md hansel@192.168.60.1:/home/hansel/HANSEL_MESH/
```

Base에서 Head로 필요한 파일을 보낸다.

```bash
ssh hansel@192.168.60.1
ssh hansel@192.168.50.10 'mkdir -p /home/hansel/HANSEL_MESH/common /home/hansel/HANSEL_MESH/configs /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/scripts /home/hansel/HANSEL_MESH/services'
scp -r /home/hansel/HANSEL_MESH/common /home/hansel/HANSEL_MESH/configs /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/scripts /home/hansel/HANSEL_MESH/services hansel@192.168.50.10:/home/hansel/HANSEL_MESH/
```

Node를 직접 제어 대상으로 테스트하려면 Base에서 node1/node2에도 서버 파일과 route script를 보낸다.

```bash
ssh hansel@192.168.50.11 'mkdir -p /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/scripts'
ssh hansel@192.168.50.12 'mkdir -p /home/hansel/HANSEL_MESH/robot /home/hansel/HANSEL_MESH/scripts'
scp /home/hansel/HANSEL_MESH/scripts/setup_mesh_route_to_laptop.sh hansel@192.168.50.11:/home/hansel/HANSEL_MESH/scripts/
scp /home/hansel/HANSEL_MESH/scripts/setup_mesh_route_to_laptop.sh hansel@192.168.50.12:/home/hansel/HANSEL_MESH/scripts/
scp -r /home/hansel/HANSEL_MESH/common /home/hansel/HANSEL_MESH/robot hansel@192.168.50.11:/home/hansel/HANSEL_MESH/
scp -r /home/hansel/HANSEL_MESH/common /home/hansel/HANSEL_MESH/robot hansel@192.168.50.12:/home/hansel/HANSEL_MESH/
```

## 3. Base Gateway

Base에서:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/setup_base_gateway.sh
```

노트북에서 유선 인터페이스 이름을 확인한다.

```bash
ip -brief addr
```

예를 들어 `enx00e04c68070e`이면:

```bash
cd ~/Projects/HANSEL_MESH
sudo ./scripts/setup_laptop_mesh_routes.sh enx00e04c68070e
```

Head에서 PC 관리망으로 돌아오는 route를 추가한다.

```bash
cd ~/HANSEL_MESH
sudo ./scripts/setup_mesh_route_to_laptop.sh
```

node1/node2를 직접 조종 대상으로 테스트하려면 각 node에서도 같은 route script를 실행한다.

## 4. 조종 테스트

Head에서:

```bash
sudo python3 ~/HANSEL_MESH/robot/mesh_control_server.py --role head --host 192.168.50.10 --allow-source 192.168.60.2/32
```

노트북에서:

```bash
python3 ~/Projects/HANSEL_MESH/controller/mesh_control_client.py --target head
```

`w`, `a`, `s`, `d`, `x`를 입력하면 Head 로그에 직접 도착해야 한다.

이 명령은 Base Pi가 내용을 해석해서 다시 보내는 구조가 아니다. 노트북에서 Head mesh IP로 보낸 UDP packet이 Base의 커널 라우팅과 BATMAN-adv를 지나 end-to-end로 도착한다.

## 5. 카메라 테스트

노트북에서 수신:

```bash
CAMERA_TRANSPORT=rtp \
./scripts/receive_camera_stream.sh 5600
```

Head에서는 systemd 관리 서비스로 송신:

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

초기 설정은 `640x480`, `15fps`, RTP/H.264 over UDP다.

예전 raw UDP 방식이 필요하면 head의 `/etc/hansel-mesh/camera.env`에서
`CAMERA_TRANSPORT="raw"`로 바꾸고 서비스를 재시작한다. 노트북 수신 명령에는
`CAMERA_TRANSPORT=raw`를 붙인다. 관리자가 `PROFILE` 값을 강제로 적용할 때는
원격 제어가 남긴 `/run/hansel-camera-profile`을 지운 뒤 서비스를 재시작한다.

## 6. 확인

Base에서:

```bash
ping -c 4 192.168.50.10
sudo batctl n
sudo batctl o
```

노트북에서:

```bash
ping -c 4 192.168.50.10
```

Head에서:

```bash
ping -c 4 192.168.60.2
```
