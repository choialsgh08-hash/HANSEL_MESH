# 3-Day Implementation Plan

## Day 1: Mesh Network

목표:

```text
base Pi ↔ node Pi ↔ head Pi ping 성공
```

작업:

- Raspberry Pi OS Lite 64-bit 준비
- hostname을 `base`, `head`, `node1`, `node2`로 설정
- SSH 접속 확인
- GitHub repo clone
- `sudo ./scripts/pi_first_boot_setup.sh <role>` 실행
- `sudo ./scripts/install_mesh.sh` 실행
- `sudo ./scripts/start_mesh.sh configs/<role>.env` 실행
- `bat0` IP 확인
- `ping` 확인
- `sudo batctl n` / `sudo batctl o` 확인

성공 기준:

- Base Pi에서 Head Pi ping 성공
- Head Pi에서 Base Pi ping 성공
- Base Pi에서 node1/node2 ping 성공
- `batctl n`에서 neighbor 확인
- `batctl o`에서 originator 확인

Day1에서는 로봇 제어, 카메라, detach 로직은 건드리지 않는다.

## Day 2: Robot Control over Mesh

목표:

구조자 PC → Base Pi → Mesh → Head/Node 제어

작업:

- 기존 Head_control.py 이식
- 기존 Node 제어 코드 이식
- IP 대역 192.168.50.x로 수정
- keyboard.py 수정
- detach된 노드는 모터 명령 제외
- Mesh 릴레이는 유지

성공 기준:

Mesh 연결 상태에서 Head/Node 모터 제어 성공

## Day 3: Camera and Field Test

목표:

Head Pi 카메라 영상 → Mesh → 구조자 PC

작업:

- 저해상도 영상 스트리밍
- 640x480 10~15fps 테스트
- Relay 수 증가에 따른 지연 확인
- 통신 품질 기준 정리
- 최종 시연 시나리오 작성

성공 기준:

- 카메라 영상 확인
- 조종 명령 지연 허용 범위 확인
- 릴레이 노드 추가 시 통신 거리 증가 확인
