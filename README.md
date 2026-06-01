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

| 장치  | 역할                      | bat0 IP       |
| ----- | ----------------         | ------------- |
| Base  | 구조자 쪽 기준 노드        | 192.168.50.1  |
| Head  | 로봇 헤드 / 카메라 / 제어  | 192.168.50.10 |
| Node1 | 릴레이 유닛 1             | 192.168.50.11 |
| Node2 | 릴레이 유닛 2             | 192.168.50.12 |
| Node3 | 릴레이 유닛 3             | 192.168.50.13 |
 
 기본 실행 순서
 chmod +x scripts/*.sh
sudo ./scripts/install_mesh.sh
sudo ./scripts/start_mesh.sh configs/base.env
장치별로 configs/*.env만 바꿔서 실행합니다.

상태 확인
./scripts/check_mesh.sh
ping 192.168.50.1
sudo batctl n
sudo batctl o