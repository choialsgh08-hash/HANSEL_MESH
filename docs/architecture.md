# HANSEL_MESH 아키텍처

## 1. 프로토콜 스택 (한 노드)

두 개의 **물리 박스**(AR9271 칩 / 라즈베리 파이)로 나뉘고, USB로 연결된다.
각 박스 안에서 도는 프로세스와, 파이 안의 계층(kernel space / user space)을
구분한다.

```mermaid
flowchart TD
    subgraph CHIP["① AR9271 동글 · 칩 내부"]
        RADIO["무선 PHY/MAC 하드웨어<br/>2.4GHz 802.11n 송수신"]
        FW["동글 내장 CPU + htc_9271.fw 펌웨어<br/>실시간 MAC: ACK·재전송·타이밍"]
        RADIO <--> FW
    end

    subgraph PI["② Raspberry Pi"]
        subgraph KSPACE["리눅스 커널 · kernel space"]
            DRV["ath9k_htc 드라이버<br/>펌웨어 업로드 · WMI/HTC 통신"]
            MAC["mac80211 / cfg80211 / nl80211<br/>무선 스택"]
            WLAN["wlan1 · netdev<br/>IBSS 모드"]
            BATMOD["batman-adv 모듈<br/>L2 메시 라우팅"]
            BAT0["bat0 · netdev<br/>IP 192.168.50.x"]
            DRV --> MAC
            MAC -->|인터페이스 생성| WLAN
            WLAN -->|hard interface 등록| BATMOD
            BATMOD -->|인터페이스 생성| BAT0
        end
        subgraph USPACE["사용자 공간 · user space"]
            APP["제어서버 :7000 · 카메라 스트림 · metrics_agent"]
        end
        BAT0 --> APP
    end

    FW <-->|USB| DRV
```

**박스별로 무슨 프로세스가 도나:**

| 물리 박스 | 계층 | 실행 주체 | 역할 |
| --- | --- | --- | --- |
| ① 칩(동글) | 무선 하드웨어 | AR9271 | 2.4GHz 송수신 |
| ① 칩(동글) | 펌웨어 | 동글 내장 CPU | **실시간** 802.11 MAC (호스트가 USB 너머로 실시간 제어 불가) |
| ② Pi 커널 | 드라이버 | ath9k_htc | 펌웨어 업로드, USB↔칩 통신 |
| ② Pi 커널 | 무선 스택 | mac80211/cfg80211 | `wlan1` netdev 생성 |
| ② Pi 커널 | netdev | wlan1 | IBSS 무선 링크 |
| ② Pi 커널 | 메시 라우팅 | batman-adv | `wlan1` 입력 → `bat0` 생성 |
| ② Pi 커널 | netdev | bat0 | end-to-end IP |
| ② Pi 유저 | 애플리케이션 | 제어/영상/모니터 | `bat0` 위에서 통신 |

> 핵심 layer 원칙:
> - **칩 안**은 펌웨어가 실시간 무선을 전담(USB 지연 때문). **파이 안**은 커널이
>   드라이버~batman까지, 유저공간이 앱.
> - batman-adv는 hard interface(`wlan1`)만 안다. AR9271/ath9k_htc를 직접 부르지 않는다.
> - IP는 `bat0`에만. `wlan1`(무선)에는 부여하지 않는다.

## 2. 물리 토폴로지

```mermaid
flowchart LR
    PC["구조자 PC<br/>192.168.60.2"] -->|eth 유선| BASE["base Pi<br/>eth0 .60.1 / bat0 .50.1<br/>gateway"]
    BASE -. mesh wlan1 .-> N2["node2 Pi<br/>bat0 .50.12"]
    N2 -. mesh .-> N1["node1 Pi<br/>bat0 .50.11"]
    N1 -. mesh .-> HEAD["head Pi<br/>bat0 .50.10<br/>카메라 / 제어"]
```

- **실선** = 유선(관리망), **점선** = BATMAN 무선 메시
- 모든 Pi가 릴레이 라우터. 중간 노드는 앱을 열지 않고 프레임만 포워딩
- 내장 `wlan0`은 mesh에서 빠져 있고, 향후 조난자 핸드폰 AP 등에 사용

## 3. 통신 흐름

- **제어**: PC → base `bat0` → 메시 next-hop → head/node 제어서버(:7000)
- **영상**: head 카메라 → `bat0` → 메시 → base → PC `video_probe`
- **모니터**: 각 노드 `metrics_agent` → base/PC `dashboard`(:7100)

## 4. 관련 문서

- 동글 도착 후 실행: [dongle_arrival_runbook.md](dongle_arrival_runbook.md)
- 재연결 감지/표시: [reconnect_detection.md](reconnect_detection.md)
- 재연결 튜닝: [reconnect_tuning.md](reconnect_tuning.md)
- 네트워크 설계: [network_design.md](network_design.md)
