# Day1 Test Plan

## 1. 목표

Day1의 목표는 Base, Head, Node Pi들이 BATMAN-adv Mesh로 연결되고 `bat0` IP로 서로 `ping` 되는 것이다.

로봇 제어, 카메라, detach 로직은 Day1 범위가 아니다.

## 2. 장치 배치

| 역할  | Hostname | Config            | bat0 IP       |
| ----- | -------- | ----------------- | ------------- |
| Base  | base     | configs/base.env  | 192.168.50.1  |
| Head  | head     | configs/head.env  | 192.168.50.10 |
| Node1 | node1    | configs/node1.env | 192.168.50.11 |
| Node2 | node2    | configs/node2.env | 192.168.50.12 |

`node3`는 장비가 추가될 때 사용하는 optional 릴레이이며 Day1 필수 성공 기준에는 포함하지 않는다.

## 3. 테스트 순서

먼저 Base와 Head만 켠다.

Base:

```bash
sudo ./scripts/start_mesh.sh configs/base.env
ping -c 4 192.168.50.10
sudo batctl n
sudo batctl o
```

Head:

```bash
sudo ./scripts/start_mesh.sh configs/head.env
ping -c 4 192.168.50.1
sudo batctl n
sudo batctl o
```

Base와 Head ping이 성공하면 Node1, Node2를 하나씩 추가한다.

Node:

```bash
sudo ./scripts/start_mesh.sh configs/node1.env
sudo ./scripts/start_mesh.sh configs/node2.env
```

각 Node에서는 자기 config 하나만 실행한다.

Base에서 확인:

```bash
ping -c 4 192.168.50.11
ping -c 4 192.168.50.12
sudo batctl n
sudo batctl o
```

## 4. 성공 기준

- `bat0` 인터페이스가 생성된다.
- `wlan0`에는 IP가 없고 `bat0`에만 `192.168.50.x/24` IP가 있다.
- Base에서 Head와 Node1/Node2로 ping이 된다.
- Head에서 Base로 ping이 된다.
- `sudo batctl n`에서 neighbor가 보인다.
- `sudo batctl o`에서 originator가 보인다.

`traceroute`는 보조 확인용이다. BATMAN-adv는 Layer 2 Mesh이므로 Day1 성공 기준은 `ping`, `batctl n`, `batctl o`다.
