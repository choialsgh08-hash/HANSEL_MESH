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
node2 Pi
   |
node1 Pi
   |
head Pi
```

현재 구현 범위는 `base`, `node2`, `node1`, `head` 네 대다. `node3`는 장비가 추가될 때 사용할 optional 릴레이로 남겨둔다.

| Hostname | 역할                          |
| -------- | ----------------------------- |
| base     | 구조자 측 Mesh 진입점          |
| head     | 로봇 헤드, 카메라, 제어 서버   |
| node1    | 릴레이 노드 및 유닛 제어       |
| node2    | 릴레이 노드 및 유닛 제어       |

| Hostname | bat0 IP       |
| -------- | ------------- |
| base     | 192.168.50.1  |
| head     | 192.168.50.10 |
| node1    | 192.168.50.11 |
| node2    | 192.168.50.12 |

모든 Mesh IP는 `bat0` 인터페이스에 부여한다.

## 3. 중요 원칙

`wlan0`에는 IP를 부여하지 않는다.
`bat0`에만 고정 IP를 부여한다.
`head`는 AP를 열지 않는다.
detach된 Node도 Mesh는 계속 유지한다.
detach는 모터 제어 제외를 의미하며, 통신 릴레이 종료를 의미하지 않는다.
Base Pi가 명령을 해석해서 각 유닛으로 다시 보내는 command dispatcher 구조는 기본 통신 구조로 사용하지 않는다.
각 유닛은 BATMAN-adv Mesh 라우터로 동작하며, 모든 일반 데이터는 `bat0` 위에서 end-to-end로 흐른다.
현재 동글 없는 구성에서는 Head의 내장 Wi-Fi 하나를 Mesh에 사용하므로, 조난자 핸드폰용 AP를 동시에 열지 않는다.

무선 모드는 `MESH_MODE=auto`를 기본값으로 두고, `802.11s mesh point`를 먼저 시도한 뒤 지원하지 않거나 join에 실패하면 `IBSS`로 fallback한다.

## 4. 통신 흐름

제어 명령:

```text
구조자 PC
→ Base Pi 관리망
→ Base Pi bat0
→ BATMAN Mesh next-hop
→ Head/Node 제어 서버
```

카메라 영상:

```text
Head Pi Camera
→ Head Pi bat0
→ BATMAN Mesh next-hop
→ Base Pi bat0
→ 구조자 PC
```

중간 node1/node2는 애플리케이션 데이터를 열어서 다시 보내는 서버가 아니라, BATMAN-adv가 선택한 next-hop으로 Ethernet frame을 포워딩하는 릴레이 노드다.

## 5. 조난자 핸드폰 접속 방향

스마트폰은 BATMAN-adv/IBSS Mesh에 직접 참여하지 않는다. 조난자 핸드폰을 연결하려면 Head Pi가 일반 Wi-Fi AP를 제공하고, Head가 그 AP와 BATMAN Mesh 사이의 gateway 역할을 해야 한다.

```text
조난자 핸드폰
→ Head Pi 일반 Wi-Fi AP
→ Head Pi bat0
→ node1/node2 BATMAN 릴레이
→ Base Pi bat0
→ 구조자 PC
```

다만 현재 전제는 Wi-Fi 동글을 추가하지 않는 것이다. 따라서 현재 단계에서는 Head AP 기능을 구현하지 않고, Camera Module 3 영상과 조종을 먼저 BATMAN Mesh 위에서 검증한다. 조난자 핸드폰 AP는 Head에 두 번째 Wi-Fi 인터페이스가 생긴 뒤 추가한다.

## 6. Day1 성공 기준

Base Pi에서 Head Pi로 ping 성공
Head Pi에서 Base Pi로 ping 성공
Base Pi에서 node1/node2 ping 성공
`batctl n`에서 주변 노드 확인
`batctl o`에서 originator 확인
Head originator의 next-hop이 배치에 따라 node1/node2를 가리키는지 확인

`traceroute`는 보조 확인용이다. BATMAN-adv는 Layer 2 Mesh이므로 핵심 확인 도구는 `ping`, `batctl n`, `batctl o`다.

## 7. 추후 확장

GStreamer 기반 저지연 영상 전송
UDP 기반 조종 명령 전송
Head 두 번째 Wi-Fi 인터페이스 기반 조난자 핸드폰 AP
Node detach 후 제어 제외
통신 품질 기반 릴레이 투하 판단
