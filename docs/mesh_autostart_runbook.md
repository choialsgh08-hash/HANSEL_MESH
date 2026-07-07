# Mesh Autostart Runbook

목표는 각 Pi에 전원만 넣으면 `wlan0 + bat0` BATMAN mesh가 자동으로 붙게 만드는 것이다. 이후 사용자는 노트북과 base를 랜선으로 연결하고, 노트북에서 영상/조종만 실행하면 된다.

## 자동으로 되는 것

- base: `bat0=192.168.50.1/24` + `eth0=192.168.60.1/24` + IPv4 forwarding
- head: `bat0=192.168.50.10/24` + `192.168.60.0/24 via base`
- node1: `bat0=192.168.50.11/24` + `192.168.60.0/24 via base`
- node2: `bat0=192.168.50.12/24` + `192.168.60.0/24 via base`

모터 서버와 카메라 서버는 안전 때문에 자동 시작하지 않는다. 전원이 들어오자마자 모터가 움직이는 상황을 피하기 위해서다.

## 각 Pi에서 한 번만 실행

코드가 `/home/hansel/HANSEL_MESH`에 배포되어 있어야 한다.

base:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/enable_mesh_autostart.sh base
sudo systemctl start hansel-mesh@base
```

head:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/enable_mesh_autostart.sh head
sudo systemctl start hansel-mesh@head
```

node1:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/enable_mesh_autostart.sh node1
sudo systemctl start hansel-mesh@node1
```

node2:

```bash
cd ~/HANSEL_MESH
sudo ./scripts/enable_mesh_autostart.sh node2
sudo systemctl start hansel-mesh@node2
```

## 노트북도 한 번만 자동 유선 프로파일 만들기

노트북에서 유선 인터페이스 이름을 확인한다.

```bash
ip -brief link
```

예전 장비 이름이 `enx00e04c68070e`라면:

```bash
cd ~/Projects/HANSEL_MESH
sudo ./scripts/setup_laptop_wired_profile.sh enx00e04c68070e
```

이후에는 랜선을 꽂으면 노트북이 자동으로:

- `192.168.60.2/24`
- `192.168.50.0/24 via 192.168.60.1`

을 잡는다.

## 전원 껐다 켠 뒤 실제 사용 순서

1. base/head/node1/node2 전원을 켠다.
2. 30초 정도 기다린다.
3. 노트북과 base를 랜선으로 연결한다.
4. 노트북에서 확인한다.

```bash
ping -c 3 192.168.60.1
ping -c 3 192.168.50.1
ping -c 3 192.168.50.10
ping -c 3 192.168.50.11
ping -c 3 192.168.50.12
```

5. base에서 relay 상태를 확인한다.

```bash
ssh hansel@192.168.60.1
sudo batctl n
sudo batctl o
```

## 상태 확인

각 Pi에서:

```bash
systemctl status hansel-mesh@head --no-pager
journalctl -u hansel-mesh@head -n 80 --no-pager
ip -brief addr
ip route
```

role만 `base`, `head`, `node1`, `node2`로 바꿔서 확인한다.

## 자동 실행 끄기

각 Pi에서:

```bash
sudo systemctl disable --now hansel-mesh@head
```

role만 자기 장치에 맞게 바꾼다.

## 수동 복구

자동 서비스가 실패했을 때:

```bash
cd ~/HANSEL_MESH
sudo systemctl stop hansel-mesh@head
sudo ./scripts/stop_mesh.sh
sudo ./scripts/start_role_network.sh head
```

base는 `head` 대신 `base`, node는 `node1` 또는 `node2`를 넣는다.
