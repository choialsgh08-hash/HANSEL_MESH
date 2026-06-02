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

구조자 PC ↔ Base Pi ↔ node2 ↔ node1 ↔ Head Pi 간 투명한 multi-hop 데이터 통신

작업:

- BATMAN-adv가 중간 node를 통해 실제 next-hop을 잡는지 `batctl o`로 확인
- Head/Node 제어 서버는 각 대상 Pi에서 직접 실행
- 노트북/구조자 앱은 Base 관리망 또는 Base Pi에서 실행하되, 데이터는 `bat0` mesh 목적지 IP로 흐르게 구성
- Base가 명령 내용을 해석해 재전송하는 command dispatcher 구조는 사용하지 않음
- 조종 명령, 상태 피드백, 음성/영상 데이터 모두 BATMAN Mesh 위의 end-to-end 트래픽으로 검증
- detach된 노드는 모터 제어 대상에서는 제외되지만 Mesh 릴레이는 유지

성공 기준:

- Base에서 Head/Node로 ping 성공
- `batctl o`에서 Head 경로의 next-hop이 거리/배치에 따라 node1/node2로 바뀌는 것 확인
- 중간 Node가 분리되어도 전원과 mesh가 유지되어 Head 데이터 통신이 지속됨
- Mesh 연결 상태에서 Head/Node 모터 제어 성공

## Day 3: Camera and Field Test

목표:

Head Pi 카메라/상태 데이터 → node1/node2 릴레이 → Base Pi → 구조자 PC

작업:

- 저해상도 영상 스트리밍
- 640x480 10~15fps 테스트
- Relay 수 증가에 따른 지연 확인
- 통신 품질 기준 정리
- 최종 시연 시나리오 작성
- 동글 없는 현재 구성에서는 조난자 핸드폰 AP를 구현하지 않음

성공 기준:

- 카메라 영상 확인
- 조종 명령 지연 허용 범위 확인
- 릴레이 노드 추가 시 통신 거리 증가 확인

## Future: Victim Phone Access

목표:

조난자 핸드폰 → Head Pi AP → BATMAN Mesh → 구조자 PC 양방향 통신

전제:

- 스마트폰은 BATMAN/IBSS Mesh에 직접 붙지 않는다.
- Head Pi에 두 번째 Wi-Fi 인터페이스가 필요하다.
- 현재 동글 없는 구성에서는 Camera Module 3 영상 + 조종을 먼저 구현한다.
