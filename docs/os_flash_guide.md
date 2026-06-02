# Raspberry Pi OS Flash Guide

## 1. 목표

Day1 목표는 모든 Raspberry Pi가 BATMAN-adv Mesh 네트워크에 참여하고, `bat0` 인터페이스를 통해 서로 `ping` 되는 것이다.

Day1에서는 로봇 모터 제어, 카메라 스트리밍, detach 로직은 진행하지 않는다.

## 2. OS와 장치 이름

권장 OS:

```text
Raspberry Pi OS Lite 64-bit
Bookworm
```

Raspberry Pi Imager의 OS Customisation에서 아래 값을 설정한다.

| 역할  | Hostname | Username |
| ----- | -------- | -------- |
| Base  | base     | hansel   |
| Head  | head     | hansel   |
| Node1 | node1    | hansel   |
| Node2 | node2    | hansel   |
| Node3 | node3    | hansel   |

SSH는 Enable 한다. 초기 설치와 GitHub clone을 위해 실험실 Wi-Fi 또는 핫스팟도 설정한다.

주의: Mesh를 시작하면 `wlan0`가 일반 Wi-Fi client가 아니라 Mesh용 인터페이스로 바뀐다. Wi-Fi SSH는 끊길 수 있으므로 Base Pi는 구조자 노트북과 유선 LAN으로 연결해 관리한다.

## 3. 첫 SSH 접속

같은 Wi-Fi에 연결된 노트북에서 접속한다.

```bash
ssh hansel@base.local
ssh hansel@head.local
ssh hansel@node1.local
ssh hansel@node2.local
ssh hansel@node3.local
```

`.local` 접속이 안 되면 공유기 관리자 페이지 또는 현재 대역에 맞춘 `nmap`으로 IP를 찾는다.

```bash
sudo nmap -sn 192.168.0.0/24
```

각 Pi에서 기본 상태를 확인한다.

```bash
hostname
ip -brief addr
iw dev
uname -a
ping -c 3 github.com
```

## 4. Repo clone과 첫 부팅 설정

Git이 없으면 먼저 설치한다.

```bash
sudo apt update
sudo apt install -y git
```

각 Pi에서 repo를 clone하고 실행 권한을 부여한다.

```bash
cd ~
git clone https://github.com/choialsgh08-hash/HANSEL_MESH.git
cd HANSEL_MESH
chmod +x scripts/*.sh
```

역할별 first boot setup을 실행한다.

```bash
sudo ./scripts/pi_first_boot_setup.sh base
sudo ./scripts/pi_first_boot_setup.sh head
sudo ./scripts/pi_first_boot_setup.sh node1
sudo ./scripts/pi_first_boot_setup.sh node2
sudo ./scripts/pi_first_boot_setup.sh node3
```

각 Pi에서는 자기 역할에 맞는 명령 하나만 실행한다.

## 5. Mesh 설치

각 Pi에서 실행한다.

```bash
sudo ./scripts/install_mesh.sh
```

이 스크립트는 `batctl`, `iw`, `rfkill`, `traceroute`를 설치하고, `batman-adv` 커널 모듈을 부팅 시 자동 로드하도록 설정한다. 또한 `services/hansel-mesh@.service`를 systemd에 설치한다.

무선 모드 지원을 확인한다.

```bash
iw list | grep -A 40 "Supported interface modes"
```

`mesh point` 또는 `IBSS` 중 하나가 있어야 한다. `MESH_MODE=auto`에서는 `mesh point`를 먼저 시도하고, 실패하면 `IBSS`로 fallback한다.

## 6. 수동 Mesh 시작

장치별로 자기 config를 사용한다.

```bash
sudo ./scripts/start_mesh.sh configs/base.env
sudo ./scripts/start_mesh.sh configs/head.env
sudo ./scripts/start_mesh.sh configs/node1.env
sudo ./scripts/start_mesh.sh configs/node2.env
sudo ./scripts/start_mesh.sh configs/node3.env
```

각 Pi에서는 자기 역할에 맞는 명령 하나만 실행한다.

Mesh 시작 후 `wlan0`에는 IP가 없어야 하고, `bat0`에만 `192.168.50.x/24` IP가 있어야 한다.

## 7. 자동 실행 설정

수동 시작이 확인되면 역할별 systemd instance를 켠다.

```bash
sudo systemctl enable hansel-mesh@base
sudo systemctl start hansel-mesh@base
```

다른 장치는 `base`를 `head`, `node1`, `node2`, `node3`로 바꿔서 실행한다.

상태 확인:

```bash
sudo systemctl status hansel-mesh@base
journalctl -u hansel-mesh@base -n 80 --no-pager
```

## 8. Day1 확인

Base와 Head만 먼저 확인한다.

Base에서:

```bash
ping -c 4 192.168.50.10
sudo batctl n
sudo batctl o
```

Head에서:

```bash
ping -c 4 192.168.50.1
sudo batctl n
sudo batctl o
```

이후 Node1, Node2, Node3를 하나씩 추가하고 Base에서 확인한다.

```bash
ping -c 4 192.168.50.11
ping -c 4 192.168.50.12
ping -c 4 192.168.50.13
sudo batctl n
sudo batctl o
```

`traceroute`는 보조 확인용이다. BATMAN-adv는 Layer 2 Mesh라서 실제 relay 순서는 `batctl n`과 `batctl o`를 기준으로 판단한다.
