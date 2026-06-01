# HANSEL_MESH Network Design

## 1. 목적

HANSEL_GRETEL 다유닛 로봇의 각 유닛을 Wi-Fi Mesh 릴레이 노드로 사용하여 구조자와 진입 로봇 사이의 통신 거리를 확장한다.

기존 AP 방식에서는 Head Pi가 직접 AP를 생성하여 모든 노드가 Head에 접속하는 스타형 구조가 되었다.

이 프로젝트에서는 모든 Pi가 동일한 BATMAN-adv Mesh에 참여하도록 하여, 분리된 유닛이 제어 대상에서는 제외되더라도 통신 릴레이 노드로 계속 동작하도록 한다.

## 2. 목표 구조

```text
구조자 PC
   |
Base Pi
   |
Node3 Pi
   |
Node2 Pi
   |
Node1 Pi
   |
Head Pi

| 장치       | 역할                |
| -------- | ----------------- |
| Base Pi  | 구조자 측 Mesh 진입점    |
| Head Pi  | 로봇 헤드, 카메라, 제어 서버 |
| Node1 Pi | 릴레이 노드 및 유닛 제어    |
| Node2 Pi | 릴레이 노드 및 유닛 제어    |
| Node3 Pi | 릴레이 노드 및 유닛 제어    |

| 장치       | IP            |
| -------- | ------------- |
| Base Pi  | 192.168.50.1  |
| Head Pi  | 192.168.50.10 |
| Node1 Pi | 192.168.50.11 |
| Node2 Pi | 192.168.50.12 |
| Node3 Pi | 192.168.50.13 |

모든 IP는 bat0 인터페이스에 부여한다.

중요 원칙
wlan0에는 IP를 부여하지 않는다.
bat0에만 고정 IP를 부여한다.
Head Pi는 AP를 열지 않는다.
detach된 Node도 Mesh는 계속 유지한다.
detach는 모터 제어 제외를 의미하며, 통신 릴레이 종료를 의미하지 않는다.

통신 흐름
제어 명령
구조자 PC
→ Base Pi
→ BATMAN Mesh
→ Head/Node 제어 서버

카메라 영상
Head Pi Camera
→ Head Pi
→ BATMAN Mesh
→ Base Pi
→ 구조자 PC

1차 성공 기준
Base Pi에서 Head Pi로 ping 성공
Head Pi에서 Base Pi로 ping 성공
batctl n에서 주변 노드 확인
batctl o에서 originator 확인

추후 확장
GStreamer 기반 저지연 영상 전송
UDP 기반 조종 명령 전송
Node detach 후 제어 제외
통신 품질 기반 릴레이 투하 판단