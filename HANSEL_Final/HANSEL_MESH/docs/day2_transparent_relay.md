# Day2 Transparent Relay Plan

## 1. 목표

Base Pi, node2, node1, Head Pi가 모두 BATMAN-adv Mesh 라우터로 동작하고, 조종/상태/영상/음성 데이터가 `bat0` 위에서 end-to-end로 흐르게 한다.

```text
구조자 PC
  |
Base Pi
  |
node2
  |
node1
  |
Head Pi
```

중간 node는 애플리케이션 명령을 해석해 재전송하지 않는다. 릴레이는 BATMAN-adv의 Layer 2 forwarding으로 수행한다.

## 2. 핵심 검증

Base에서 Head까지 통신:

```bash
ping -c 4 192.168.50.10
sudo batctl n
sudo batctl o
```

`batctl o`에서 `192.168.50.10`에 해당하는 originator의 next-hop이 물리 배치에 따라 `node1` 또는 `node2` 쪽 MAC으로 잡히는지 확인한다.

Head에서 Base까지 통신:

```bash
ping -c 4 192.168.50.1
sudo batctl n
sudo batctl o
```

## 3. 데이터 흐름

조종 명령:

```text
구조자 PC 또는 Base Pi 조종 앱
→ Base bat0
→ BATMAN next-hop
→ Head/Node 제어 서버
```

상태/영상/음성:

```text
Head/Node 앱
→ Head/Node bat0
→ BATMAN next-hop
→ Base bat0
→ 구조자 PC
```

## 4. 성공 기준

- Base에서 Head/node1/node2 ping 성공
- Head에서 Base ping 성공
- `batctl o`에서 originator와 next-hop 확인
- node1/node2를 물리적으로 릴레이 위치에 배치했을 때 Head 통신이 유지됨
- 분리된 node는 모터 제어 대상에서 제외되지만 mesh interface와 `bat0`는 계속 유지됨
