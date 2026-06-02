# HANSEL_MESH Network Design

## 1. 목적

HANSEL_GRETEL 다유닛 로봇의 각 유닛을 Wi-Fi Mesh 릴레이 노드로 사용하여 구조자와 진입 로봇 사이의 통신 거리를 확장한다.

기존 AP 방식에서는 Head Pi가 직접 AP를 생성하여 모든 노드가 Head에 접속하는 스타형 구조가 되었다.

이 프로젝트에서는 모든 Pi가 동일한 BATMAN-adv Mesh에 참여하도록 하여, 분리된 유닛이 제어 대상에서는 제외되더라도 통신 릴레이 노드로 계속 동작하도록 한다.

## 2. 목표 구조

```text
구조자 PC
   |
base Pi (eth0: 구조자 PC, wlan0/bat0: Mesh)
   |
node3 Pi
   |
node2 Pi
   |
node1 Pi
   |
head Pi
```

| Hostname | 역할                          |
| -------- | ----------------------------- |
| base     | 구조자 측 Mesh 진입점          |
| head     | 로봇 헤드, 카메라, 제어 서버   |
| node1    | 릴레이 노드 및 유닛 제어       |
| node2    | 릴레이 노드 및 유닛 제어       |
| node3    | 릴레이 노드 및 유닛 제어       |

| Hostname | bat0 IP       |
| -------- | ------------- |
| base     | 192.168.50.1  |
| head     | 192.168.50.10 |
| node1    | 192.168.50.11 |
| node2    | 192.168.50.12 |
| node3    | 192.168.50.13 |

모든 IP는 `bat0` 인터페이스에 부여한다.

## 3. 중요 원칙

`wlan0`에는 IP를 부여하지 않는다.
`bat0`에만 고정 IP를 부여한다.
`head`는 AP를 열지 않는다.
detach된 Node도 Mesh는 계속 유지한다.
detach는 모터 제어 제외를 의미하며, 통신 릴레이 종료를 의미하지 않는다.

무선 모드는 `MESH_MODE=auto`를 기본값으로 두고, `802.11s mesh point`를 먼저 시도한 뒤 지원하지 않거나 join에 실패하면 `IBSS`로 fallback한다.

## 4. 통신 흐름

제어 명령:

```text
구조자 PC
→ Base Pi
→ BATMAN Mesh
→ Head/Node 제어 서버
```

카메라 영상:

```text
Head Pi Camera
→ Head Pi
→ BATMAN Mesh
→ Base Pi
→ 구조자 PC
```

## 5. Day1 성공 기준

Base Pi에서 Head Pi로 ping 성공
Head Pi에서 Base Pi로 ping 성공
Base Pi에서 node1/node2/node3 ping 성공
`batctl n`에서 주변 노드 확인
`batctl o`에서 originator 확인

`traceroute`는 보조 확인용이다. BATMAN-adv는 Layer 2 Mesh이므로 핵심 확인 도구는 `ping`, `batctl n`, `batctl o`다.

## 6. 추후 확장

GStreamer 기반 저지연 영상 전송
UDP 기반 조종 명령 전송
Node detach 후 제어 제외
통신 품질 기반 릴레이 투하 판단
