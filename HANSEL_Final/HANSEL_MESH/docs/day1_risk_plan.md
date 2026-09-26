# Day1 Risk Plan

## 1. wlan0 Mesh 전환 시 SSH 끊김

### 원인

Raspberry Pi의 `wlan0`를 일반 Wi-Fi 연결에서 BATMAN Mesh용 인터페이스로 전환하면 기존 SSH 연결이 끊긴다.

### 대책

Day1에서는 Base Pi만 노트북과 관리망으로 연결한다.

권장 구조:

```text
노트북
  |
Base Pi 관리망: eth0
Base Pi Mesh망: wlan0 / bat0
  |
Node / Head Mesh망: wlan0 / bat0
```

## 2. 무선 모드 미지원

### 원인

Pi 또는 USB Wi-Fi 어댑터가 `mesh point`와 `IBSS`를 모두 지원하지 않으면 BATMAN-adv용 무선 링크를 만들 수 없다.

### 대책

각 Pi에서 아래 명령으로 지원 모드를 먼저 확인한다.

```bash
iw list | grep -A 40 "Supported interface modes"
```

`MESH_MODE=auto`는 `mesh point`를 먼저 시도하고, 실패하면 `IBSS`를 시도한다. 둘 다 안 되면 mesh 지원이 좋은 USB Wi-Fi 동글을 사용한다.

## 3. AP/client 서비스 충돌

### 원인

`hostapd`, `dnsmasq`, `wpa_supplicant`, NetworkManager가 `wlan0`를 잡고 있으면 mesh join이 실패할 수 있다.

### 대책

`start_mesh.sh`가 AP 관련 서비스를 정리하고, NetworkManager가 있으면 `wlan0`를 unmanaged로 바꾼다. Mesh 시작 후 일반 Wi-Fi SSH는 끊길 수 있으므로 Base Pi는 유선 LAN으로 관리한다.

## 4. traceroute 해석 주의

BATMAN-adv는 Layer 2 Mesh라서 `traceroute`가 실제 무선 relay 순서를 명확히 보여주지 않을 수 있다. Day1 성공 기준은 `ping`, `sudo batctl n`, `sudo batctl o`다.
