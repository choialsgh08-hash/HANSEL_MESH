# 재연결 감지·표시 기준

대시보드의 "재연결 중" 배너와 이벤트 로그가 **어떤 기준으로 켜지는지** 정리한다.
끊김 자체를 줄이는 레버는 [reconnect_tuning.md](reconnect_tuning.md) 참고.

## 데이터 흐름

```
각 노드 metrics_agent (--loop)
  └ 연속 스냅샷 비교 → detect_events → snap["events"]
        │ (UDP :7100)
        ▼
laptop dashboard
  └ ingest: 이벤트에 수신시각 타임스탬프 부여 → EVENTS 버퍼
  └ merge_state: reconnect_active 계산 → /api/state
        ▼
  web: 배너 + 이벤트 로그 표시
```

## 1단계: 이벤트 생성 (metrics_agent detect_events)

연속된 두 스냅샷을 비교해 아래 이벤트를 만든다. **순수 함수**(prev, curr → events)라
하드웨어 없이 fixture로 검증된다. 이벤트는 `--loop` 모드에서만 생성된다(이전 스냅샷 필요).

| 이벤트 | 조건 | 의미 |
| --- | --- | --- |
| `route_changed` | 같은 peer(MAC)의 next-hop이 이전≠현재 | 경로가 다른 노드로 전환됨 |
| `neighbor_lost` | 직접 이웃이 이전엔 있었으나 현재 없음 | 직접 링크가 끊김 |
| `neighbor_gained` | 직접 이웃이 현재 새로 나타남 | 직접 링크가 (재)연결됨 |

- "직접 이웃" = `batctl n`에 나오는 direct neighbor (`link.direct == True`).
- next-hop 판정은 `batctl o`의 선택된 경로(`*` 표시 라인) 기준.
- peer 식별은 MAC 기준, 표시는 노드 이름(bat0 ARP로 매핑)으로.

## 2단계: 배너 on/off (dashboard)

- `ingest`가 각 이벤트에 수신 시각(`ts`)과 `HH:MM:SS`를 찍어 `EVENTS` 버퍼(최근 60개)에 쌓는다.
- `merge_state`가 매 요청마다 계산:
  - **`reconnect_active` = 최근 `RECONNECT_WINDOW_S`(기본 6.0초) 안에 이벤트가 하나라도 있으면 `True`**
  - `reconnect_latest` = 그 창 안의 가장 최근 이벤트
  - `events` = 최근 15개 (로그 패널용)
- 웹 페이지(`poll` → `updateReconnect`):
  - `reconnect_active`면 맥동 배너 표시, 문구 = 최근 이벤트 요약
    (예: `재연결 중 — 경로 변경 node2 → …00:0a · 영상이 잠시 끊길 수 있습니다`)
  - 6초간 새 이벤트가 없으면 배너 자동 소멸(이벤트 로그에는 계속 남음)

### 왜 6초 창인가

배너는 "지금 재연결이 진행/직후"임을 조종자에게 알리는 용도다. 이벤트는 순간적으로
찍히므로, 그 순간만 켜면 놓치기 쉽다. 그래서 이벤트 발생 후 일정 시간(기본 6초) 동안
배너를 유지해 조종자가 인지할 시간을 준다. 이 값은 `dashboard.py`의
`RECONNECT_WINDOW_S`로 조정한다(짧게=민감/깜빡임, 길게=둔감/오래 유지).

## 조정 지점

| 항목 | 위치 | 효과 |
| --- | --- | --- |
| 배너 유지 시간 | `dashboard.py` `RECONNECT_WINDOW_S` | 배너가 켜져 있는 시간 |
| 이벤트 로그 개수 | `dashboard.py` `EVENTS` maxlen(60) / `merge_state` `[-15:]` | 보관/표시 이벤트 수 |
| 샘플 주기 | `metrics_agent --interval` | 이벤트 감지 지연(주기가 길수록 감지 늦음) |

## 한계 (정직하게)

- 이벤트는 **샘플 주기 단위로만** 감지된다. `--interval 5`면 최대 5초 늦게 잡힐 수 있다.
- 아직 **실제 무선 데이터로 검증되지 않았다**(동글 없음). fixture로 로직만 검증됨.
- 배너는 "인지"용이지 끊김을 줄이지 않는다. 끊김 단축은 tuning 문서의 레버로 한다.
