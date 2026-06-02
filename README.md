# HANSEL_MESH

HANSEL_GRETEL 다유닛 로봇을 위한 BATMAN-adv 기반 Wi-Fi Mesh 릴레이 통신 프로젝트입니다.

## 목표 구조

```text
구조자 PC
   |
Base Pi
   |
Relay Node Pi
   |
Relay Node Pi
   |
Head Pi / Robot
```

| Hostname | 역할                     | Config            | bat0 IP       |
| -------- | ------------------------ | ----------------- | ------------- |
| base     | 구조자 쪽 기준 노드       | configs/base.env  | 192.168.50.1  |
| head     | 로봇 헤드 / 카메라 / 제어 | configs/head.env  | 192.168.50.10 |
| node1    | 릴레이 유닛 1            | configs/node1.env | 192.168.50.11 |
| node2    | 릴레이 유닛 2            | configs/node2.env | 192.168.50.12 |
| node3    | 릴레이 유닛 3            | configs/node3.env | 192.168.50.13 |

## 기본 실행 순서

```bash
chmod +x scripts/*.sh
sudo ./scripts/install_mesh.sh
sudo ./scripts/start_mesh.sh configs/base.env
```

장치별로 configs/*.env만 바꿔서 실행합니다.

## 자동 실행

```bash
sudo systemctl enable hansel-mesh@base
sudo systemctl start hansel-mesh@base
```

다른 장치는 `base`를 `head`, `node1`, `node2`, `node3`로 바꿔서 사용합니다.

## 상태 확인

```bash
sudo ./scripts/check_mesh.sh
ping 192.168.50.1
sudo batctl n
sudo batctl o
```
