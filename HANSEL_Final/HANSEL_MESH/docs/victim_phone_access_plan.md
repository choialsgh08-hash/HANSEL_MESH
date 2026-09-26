# Victim Phone Access Plan

## 1. 결론

조난자 핸드폰은 BATMAN-adv/IBSS Mesh에 직접 접속하지 않는다. 스마트폰 접속은 일반 Wi-Fi AP가 필요하다.

최종 구조는 아래와 같다.

```text
조난자 핸드폰
  |
Head Pi Wi-Fi AP
  |
Head Pi bat0
  |
node1/node2 BATMAN Mesh 릴레이
  |
Base Pi bat0
  |
구조자 PC
```

## 2. 현재 전제

현재는 Wi-Fi 동글을 추가하지 않는다.

따라서 Head Pi의 내장 Wi-Fi는 BATMAN Mesh용으로 사용하고, 조난자 핸드폰 AP는 현재 구현 범위에서 제외한다.

```text
head wlan0 = BATMAN Mesh
head AP    = 현재 구현하지 않음
```

## 3. 이유

Head Pi가 내장 Wi-Fi 하나로 Mesh와 일반 AP를 동시에 안정적으로 제공하기 어렵다.

로봇 내부 통신이 끊기면 구조자와 로봇 사이의 영상/조종/상태 데이터가 모두 끊긴다. 그래서 현재 단계에서는 Mesh 유지가 우선이다.

## 4. 구현 우선순위

1. Base/node2/node1/head BATMAN Mesh 안정화
2. Head 카메라 영상 → Base/구조자 PC 전송
3. 구조자 조종 명령 → Head/Node 제어
4. 통신 품질 기준에 따른 릴레이 분리 판단
5. 두 번째 Wi-Fi 인터페이스 추가 후 조난자 핸드폰 AP 구현

## 5. 핸드폰 AP 추가 시 방향

Head에 두 번째 Wi-Fi 인터페이스가 생기면 아래처럼 분리한다.

```text
head wlan0 = BATMAN Mesh
head wlan1 = 조난자 핸드폰 AP
```

초기 서비스는 브라우저 기반 구조 포털로 시작한다.

```text
SSID: HANSEL_RESCUE
Phone URL: http://10.70.0.1
기능: SOS, 텍스트 메시지, 상태 전달
```

음성 통화는 텍스트/상태 전달 이후에 추가한다.
