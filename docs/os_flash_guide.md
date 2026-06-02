# Raspberry Pi OS Flash Guide

## 1. 목표

HANSEL_MESH Day1 목표는 모든 Raspberry Pi가 BATMAN-adv Mesh 네트워크에 참여하고, `bat0` 인터페이스를 통해 서로 ping이 되는 것이다.

Day1에서는 로봇 모터 제어와 카메라 스트리밍은 하지 않는다.

## 2. 사용할 OS

권장 OS:

```text
Raspberry Pi OS Lite 64-bit
Bookworm

GUI가 필요 없고, SSH와 네트워크 설정만 필요하므로 Lite 버전을 사용한다.

3. Raspberry Pi Imager 설정

Raspberry Pi Imager 실행 후 다음 순서로 진행한다.

3.1 장치 선택

각 Pi 모델에 맞게 선택한다.

Raspberry Pi 4  -> Head Pi
Raspberry Pi 3  -> Base / Node1 / Node2 / Node3
3.2 OS 선택
Raspberry Pi OS Lite 64-bit
3.3 저장소 선택

각 Pi에 사용할 microSD 카드를 선택한다.

3.4 OS Customisation

반드시 설정한다.

Hostname

장치별로 아래처럼 설정한다.

hansel-base
hansel-head
hansel-node1
hansel-node2
hansel-node3
Username / Password

예시:

username: hansel
password: 프로젝트에서 사용할 비밀번호

모든 Pi에서 username은 동일하게 hansel로 통일하는 것을 추천한다.

Wi-Fi

초기 SSH 접속과 패키지 설치를 위해 실험실/핫스팟 Wi-Fi를 설정한다.

중요:

BATMAN Mesh를 시작하면 wlan0가 Mesh용으로 바뀌므로 일반 Wi-Fi 연결은 끊길 수 있다.

Day1 설치 과정에서는 먼저 일반 Wi-Fi로 SSH 접속해서 패키지를 설치하고, 이후 Mesh 모드로 전환한다.

SSH

SSH를 Enable 한다.

처음에는 password authentication을 사용해도 된다.

추후 안정화를 위해 SSH key 인증으로 바꾸는 것을 추천한다.

4. SD 카드 굽기 후 첫 부팅

각 Pi에 SD 카드를 삽입하고 전원을 연결한다.

부팅 후 같은 Wi-Fi에 연결된 노트북에서 다음 명령어로 접속을 시도한다.

ssh hansel@hansel-base.local
ssh hansel@hansel-head.local
ssh hansel@hansel-node1.local
ssh hansel@hansel-node2.local
ssh hansel@hansel-node3.local

.local 접속이 안 되면 공유기 관리자 페이지 또는 nmap으로 IP를 확인한다.

예시:

sudo nmap -sn 192.168.0.0/24

또는 현재 노트북 대역에 맞춰 사용한다.

5. 각 Pi에서 기본 확인

SSH 접속 후 다음 명령어를 실행한다.

hostname
ip -brief addr
iw dev
uname -a

확인할 것:

1. hostname이 의도한 이름인지
2. wlan0가 존재하는지
3. 인터넷 연결이 되는지
4. apt update가 되는지

인터넷 확인:

ping -c 3 8.8.8.8
ping -c 3 github.com
6. GitHub clone

각 Pi에서 다음을 실행한다.

cd ~
git clone https://github.com/choialsgh08-hash/HANSEL_MESH.git
cd HANSEL_MESH
chmod +x scripts/*.sh
7. 설치

각 Pi에서:

sudo ./scripts/install_mesh.sh
8. Mesh 시작

장치별로 하나씩 실행한다.

Base:

sudo ./scripts/start_mesh.sh configs/base.env

Head:

sudo ./scripts/start_mesh.sh configs/head.env

Node1:

sudo ./scripts/start_mesh.sh configs/node1.env

Node2:

sudo ./scripts/start_mesh.sh configs/node2.env

Node3:

sudo ./scripts/start_mesh.sh configs/node3.env
9. 주의사항

Mesh를 시작하면 wlan0가 IBSS/Mesh 용도로 바뀌기 때문에 기존 Wi-Fi SSH 접속이 끊길 수 있다.

처음 Day1 테스트에서는 HDMI/키보드가 있으면 안전하고, 없으면 다음 순서로 진행한다.

1. 각 Pi를 일반 Wi-Fi로 접속
2. repo clone
3. install_mesh.sh 실행
4. start_mesh.sh 실행
5. 이후 bat0 IP로 접속 시도

예시:

ssh hansel@192.168.50.1
ssh hansel@192.168.50.10

단, 노트북이 직접 bat0 네트워크에 있는 것은 아니므로, 구조자 노트북은 Base Pi를 통해 접근하거나 Base Pi에 유선/별도 Wi-Fi로 연결해야 한다.

Day1의 핵심은 Pi들끼리 bat0로 ping 되는 것이다.


---

## 2. `docs/day1_test_plan.md`

```markdown
# Day 1 Test Plan

## 1. Day1 목표

Day1의 목표는 다음 하나다.

```text
Base Pi, Relay Pi, Head Pi가 BATMAN-adv Mesh로 연결되고 bat0 IP로 ping이 되는 것

로봇 제어, 카메라, 자동 릴레이 투하는 Day1 목표가 아니다.

2. 장치 배치

권장 배치:

Pi 3  -> Base
Pi 3  -> Node1
Pi 3  -> Node2
Pi 3  -> Node3
Pi 4  -> Head
3. IP 계획
장치	Hostname	Config	bat0 IP
Base	hansel-base	configs/base.env	192.168.50.1
Head	hansel-head	configs/head.env	192.168.50.10
Node1	hansel-node1	configs/node1.env	192.168.50.11
Node2	hansel-node2	configs/node2.env	192.168.50.12
Node3	hansel-node3	configs/node3.env	192.168.50.13
4. 테스트 순서
Step 1. Base와 Head만 테스트

Base에서:

sudo ./scripts/start_mesh.sh configs/base.env

Head에서:

sudo ./scripts/start_mesh.sh configs/head.env

Base에서:

ping 192.168.50.10
sudo batctl n
sudo batctl o

Head에서:

ping 192.168.50.1
sudo batctl n
sudo batctl o

성공 기준:

Base ↔ Head ping 성공
batctl n에서 neighbor가 보임
batctl o에서 originator가 보임
Step 2. Node1 추가

Node1에서:

sudo ./scripts/start_mesh.sh configs/node1.env

Base에서:

ping 192.168.50.11
ping 192.168.50.10
sudo batctl n
sudo batctl o

성공 기준:

Base ↔ Node1 ping 성공
Base ↔ Head ping 유지
Step 3. Node2 추가

Node2에서:

sudo ./scripts/start_mesh.sh configs/node2.env

Base에서:

ping 192.168.50.12
ping 192.168.50.10
sudo batctl n
sudo batctl o
Step 4. Node3 추가

Node3에서:

sudo ./scripts/start_mesh.sh configs/node3.env

Base에서:

ping 192.168.50.13
ping 192.168.50.10
sudo batctl n
sudo batctl o
5. 거리 테스트

처음에는 모든 Pi를 책상 위에 놓고 테스트한다.

그다음 다음 순서로 멀리 둔다.

Base 고정
Node1을 3~5m 이동
Node2를 더 멀리 이동
Node3를 더 멀리 이동
Head를 가장 멀리 이동

각 단계에서 Base에서 다음을 확인한다.

ping -c 20 192.168.50.10
sudo batctl n
sudo batctl o
6. 기록할 값

각 거리 테스트마다 기록한다.

1. 배치 구조
2. ping 평균 RTT
3. packet loss
4. batctl n 출력
5. batctl o 출력
6. 끊김 여부
7. 실패 시 확인 순서
문제 1. bat0가 없음
ip addr show bat0
lsmod | grep batman

해결:

sudo modprobe batman-adv
sudo ./scripts/start_mesh.sh configs/해당장치.env
문제 2. wlan0가 없음
ip link
iw dev

해결:

sudo rfkill unblock wifi
sudo reboot
문제 3. ping이 안 됨

각 Pi에서 확인:

ip addr show bat0
sudo batctl n
sudo batctl o

확인할 것:

1. 모든 Pi의 MESH_ID가 같은지
2. 모든 Pi의 MESH_FREQ가 같은지
3. IP가 중복되지 않았는지
4. wlan0가 같은 모드인지
문제 4. SSH가 끊김

Mesh 시작 후 wlan0가 일반 Wi-Fi에서 Mesh로 바뀌면 SSH가 끊기는 것이 정상일 수 있다.

해결 방향:

1. HDMI/키보드로 직접 확인
2. Base Pi에 유선 LAN 사용
3. USB Wi-Fi 동글을 추가해서 관리용 Wi-Fi와 Mesh용 Wi-Fi 분리

Day1에서는 가능하면 HDMI/키보드 1세트를 준비한다.

8. Day1 완료 기준

다음이 모두 되면 Day1 완료다.

Base에서 Head ping 성공
Head에서 Base ping 성공
Base에서 Node1/Node2/Node3 ping 성공
batctl n에서 neighbor 확인
batctl o에서 originator 확인
Mesh 시작/정지 스크립트 정상 동작

---

## 3. `scripts/pi_first_boot_setup.sh`

```bash
#!/bin/bash

set -e

echo "========================================"
echo " HANSEL_MESH first boot setup"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root:"
    echo "sudo ./scripts/pi_first_boot_setup.sh"
    exit 1
fi

echo "[1/8] Checking OS information..."
cat /etc/os-release || true

echo "[2/8] Updating package index..."
apt update

echo "[3/8] Installing basic tools..."
apt install -y \
    git \
    curl \
    wget \
    vim \
    nano \
    net-tools \
    iproute2 \
    wireless-tools \
    iw \
    rfkill \
    batctl \
    tmux \
    htop \
    tree \
    avahi-daemon

echo "[4/8] Enabling SSH service..."
systemctl enable ssh
systemctl start ssh

echo "[5/8] Enabling avahi-daemon for .local hostname access..."
systemctl enable avahi-daemon
systemctl start avahi-daemon

echo "[6/8] Enabling batman-adv kernel module at boot..."
echo "batman-adv" > /etc/modules-load.d/batman-adv.conf
modprobe batman-adv

echo "[7/8] Unblocking Wi-Fi..."
rfkill unblock wifi || true

echo "[8/8] Showing system summary..."
echo ""
echo "Hostname:"
hostname

echo ""
echo "Network interfaces:"
ip -brief addr

echo ""
echo "Wi-Fi devices:"
iw dev || true

echo ""
echo "BATMAN module:"
lsmod | grep batman || echo "[WARN] batman-adv not loaded."

echo ""
echo "========================================"
echo " First boot setup complete."
echo "========================================"
echo "Next:"
echo "cd ~/HANSEL_MESH"
echo "sudo ./scripts/start_mesh.sh configs/head.env"
실행
chmod +x scripts/*.sh

git add .
git commit -m "Add Day1 OS flashing and first boot setup docs"
git push origin main
준비물 체크리스트
필수
- Raspberry Pi 4 1대
- Raspberry Pi 3 4대
- microSD 카드 5개
- Pi 전원 어댑터 또는 보조배터리 5개
- 노트북
- 같은 Wi-Fi 또는 핫스팟
- Raspberry Pi Imager

가능하면 있으면 좋은 것:

- HDMI 케이블
- 키보드
- 마우스
- LAN 케이블
- USB Wi-Fi 동글 1~2개

특히 HDMI/키보드 1세트는 꼭 준비하는 게 좋아. Mesh 시작하면 SSH가 끊길 수 있어서, 화면 없이 하면 막힐 수 있음.