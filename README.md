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
Head Pi / Robot
```

| Hostname | 역할                     | Config            | bat0 IP       |
| -------- | ------------------------ | ----------------- | ------------- |
| base     | 구조자 쪽 기준 노드       | configs/base.env  | 192.168.50.1  |
| head     | 로봇 헤드 / 카메라 / 제어 | configs/head.env  | 192.168.50.10 |
| node1    | 릴레이 유닛 1            | configs/node1.env | 192.168.50.11 |
| node2    | 릴레이 유닛 2            | configs/node2.env | 192.168.50.12 |

`node3`는 예비 Pi가 생겼을 때 추가할 optional 릴레이입니다.

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

다른 장치는 `base`를 `head`, `node1`, `node2`로 바꿔서 사용합니다.

## 상태 확인

```bash
sudo ./scripts/check_mesh.sh
ping 192.168.50.1
sudo batctl n
sudo batctl o
```

## 카메라 + 조종

동글 없는 현재 구성에서는 Head의 `wlan0`를 Mesh 전용으로 쓰고, 카메라 영상과 조종 명령을 먼저 검증합니다.

실행 순서는 [Camera And Control Quickstart](docs/camera_control_quickstart.md)를 따른다.

다음 현장 테스트 시작 순서는 [Next Field Test Checklist](docs/next_field_test_checklist.md)를 따른다.

실제 GPIO 모터/엔코더 제어는 [Motor Control Quickstart](docs/motor_control_quickstart.md)를 따른다.

내일 구동 + 카메라 테스트 전체 순서는 [Tomorrow Drive + Camera Runbook](docs/tomorrow_drive_camera_runbook.md)을 따른다.

팀원 설명용 코드/원리 문서는 [HANSEL_MESH Code And Network Explainer](docs/hansel_mesh_code_explainer.md)를 따른다.

## 통신 원칙

Base Pi가 명령 내용을 해석해 각 유닛으로 다시 보내는 dispatcher 구조가 아니라, 모든 유닛이 BATMAN-adv 릴레이 노드로 동작합니다.

데이터는 `bat0` 위에서 end-to-end로 흐르고, 중간 node는 BATMAN-adv next-hop으로 프레임을 포워딩합니다.
