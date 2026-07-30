# Rescue Network (구조 요청 메시 네트워크)

인터넷이 없는 재난 환경에서 조난자의 구조 요청을 수집하고, B.A.T.M.A.N.-adv
메시망을 통해 구조대 서버로 전달하기 위한 시스템입니다. `HANSEL_MESH` 저장소의
자가완결형 서브프로젝트(`rescue-network/`)로 존재합니다.

> **현재 상태: Phase 1–6 전체 구현됨** — 코어(구조 요청 앱·receiver·forwarder·대시보드),
> systemd 배포, 조난자 AP, **기존 HANSEL 메시 재사용**, 그리고 선택 기능(자동 롤백·모니터링·
> HMAC 인증·SSE·captive portal·유선 게이트웨이)까지 완료.
>
> 처음이라면 아래 **[동작 원리](#동작-원리-구조와-원리)** → **[실행 순서](#실행-순서-한눈에)** 순서로 읽으세요.

## 프로젝트 목적

1. 조난자가 스마트폰으로 현장 Raspberry Pi의 Wi-Fi AP에 접속한다.
2. 브라우저에서 `http://192.168.10.1/` 를 열어 구조 요청 양식을 작성한다.
3. 현장 노드가 요청을 **먼저 SQLite에 저장**한다.
4. 별도 전달 프로세스가 요청을 구조대 서버로 전송하고, 실패 시 재시도한다.
5. 구조대 서버는 요청을 중복 없이 저장하고 대시보드로 확인한다.

## 네트워크 구조 (전체 목표)

rescue-network는 **기존 HANSEL B.A.T.M.A.N.-adv 메시(bat0, 192.168.50.0/24)를 그대로
재사용**합니다. 별도의 메시를 만들지 않습니다 — 메시 구성은 저장소 루트의
`scripts/*mesh*.sh` + `configs/*.env`가 담당합니다.

```text
[스마트폰] --Wi-Fi AP(192.168.10.0/24)--> [field Pi]
                                              |  bat0 (기존 HANSEL 메시, 192.168.50.0/24)
                                          [relay Pi ...]  <-- B.A.T.M.A.N.-adv (Layer 2)
                                              |
                                          [receiver = base Pi] 192.168.50.1:8080
                                              |
                                          [구조대 대시보드]
```

**두 주소 대역을 혼동하지 마세요:**

| 대역 | 용도 | 예시 |
|---|---|---|
| **AP 네트워크** | 스마트폰 ↔ 현장 Pi | `http://192.168.10.1/` (스마트폰 접속 주소) |
| **bat0 메시 대역** | Pi ↔ Pi (기존 HANSEL 메시) | `192.168.50.1:8080` (내부 전달 주소) |

`192.168.50.1` 은 스마트폰 접속 주소가 **아닙니다** — 메시망 내부(구조대 노드) 주소입니다.

## 노드 역할 (`NODE_ROLE`)

| 역할 | 실행하는 것 |
|---|---|
| `field` | 조난자 AP · 구조 요청 웹/API · SQLite · forwarder · BATMAN-adv |
| `relay` | BATMAN-adv 중계만 (웹/HTTP 중계 서버 없음) |
| `receiver` | 수신 API · SQLite(중복 제거) · 대시보드 · BATMAN-adv |

중간 노드는 Layer 2 프레임만 전달하며, HTTP 중계 애플리케이션을 두지 않습니다.

---

## 동작 원리 (구조와 원리)

### 한 문장 요약

> 조난자는 **현장 노드(field)** 의 웹 폼에 구조 요청을 쓴다 → 현장 노드는 그것을
> **자기 SQLite에 먼저 저장**한다 → **별도의 forwarder 프로세스**가 그 요청을
> 메시망 너머 **구조대 노드(receiver)** 로 HTTP POST 한다 → receiver는 **중복 없이
> 저장**하고 ACK를 돌려준다 → 구조대는 **대시보드**로 실시간 확인한다.

핵심 설계 원칙은 **"저장 우선, 전달은 나중에"(store-and-forward)** 입니다. 재난
환경의 무선 링크는 자주 끊기므로, 요청을 받는 즉시 로컬에 안전하게 적어두고,
네트워크 전달은 실패해도 사라지지 않도록 재시도로 분리했습니다.

### 컴포넌트 지도

```text
         ┌─────────────── field 노드 (현장 Raspberry Pi) ───────────────┐
스마트폰 │   field_app.py (FastAPI)              forwarder.py (별도 프로세스) │
  ──POST─┼─▶ POST /api/rescue                         │  2s마다 폴링          │
 /api/   │      │ 검증(schemas)                        │                       │
 rescue  │      ▼                                      ▼                       │
         │   rescue_service ─▶ rescue_repository ─▶ [ field.db (SQLite/WAL) ]  │
         │                         ▲  같은 파일 공유  │                        │
         └─────────────────────────┼─────────────────┼────────────────────────┘
                                   │                  │ HTTP POST (bat0 경유)
                                   │                  │  X-Rescue-Token 헤더
                                   │                  ▼
         ┌──────────────── receiver 노드 (구조대) ─────────────────────┐
         │   receiver_app.py (FastAPI)                                  │
 구조대  │   POST /api/rescue/receive ─▶ receiver_service(멱등)         │
 브라우저─┼─▶ GET /dashboard  ◀─ GET /api/received ─▶ received_repository │
         │                                     │                        │
         │                                     ▼                        │
         │                            [ receiver.db (request_id UNIQUE) ]│
         └──────────────────────────────────────────────────────────────┘
```

`relay` 노드는 이 그림에 **애플리케이션이 없습니다**. 기존 HANSEL 메시가 올라와
있으면 Linux 라우팅 + B.A.T.M.A.N.-adv가 field↔receiver 사이의 Layer 2 프레임을 옮기고,
앱은 그저 `RECEIVER_URL`(`192.168.50.1:8080`)로 HTTP를 쏘면 커널이 경로를 정합니다.
**rescue-network는 메시를 만들지 않고, 이미 도는 메시 위에 얹힙니다.**

### 코드 계층 (책임 분리)

라우트가 DB를 직접 만지지 않도록 **route → service → repository → ORM** 4계층으로
나눴습니다.

| 계층 | 파일 | 책임 |
|---|---|---|
| 설정 | `config.py` | Pydantic Settings(환경변수). 토큰은 `SecretStr` |
| 검증 | `schemas.py` | 입력/출력 Pydantic 모델 + enum(`InjuryStatus`/`DeliveryStatus`) |
| 라우트 | `field_app.py` / `receiver_app.py` | HTTP 껍데기(FastAPI 앱 팩토리) |
| 서비스 | `services/*.py` | 비즈니스 로직(생성·멱등 저장·전달 1건) |
| 저장소 | `repositories/*.py` | SQL/ORM 접근만 |
| ORM | `models.py` | 테이블 정의(`rescue_requests`, `received_rescue_requests`) |
| DB 엔진 | `database.py` | SQLite 엔진(WAL·busy_timeout), 세션 팩토리 |
| 재시도 정책 | `retry.py` | 순수 함수: backoff 계산 + 오류 분류(IO 없음) |
| 전달 코어 | `services/delivery_service.py` | 전달 1건 시도 + 상태 전이(테스트 가능) |
| 전달 루프 | `forwarder.py` | 폴링·복구·프로세스 수명(`python -m rescue_network.forwarder`) |
| 인증 | `security.py` | 공유 토큰 상수시간 비교 |

### 구조 요청의 일생 (delivery_status 상태 기계)

한 요청은 현장 노드 DB 안에서 다음 상태를 오갑니다.

```text
  POST /api/rescue
        │  (로컬 저장 성공 = 조난자에게 즉시 "접수됨" 응답)
        ▼
    ┌─────────┐  forwarder가 집음      ┌─────────┐   유효한 ACK    ┌───────────┐
    │ pending │ ─────────────────────▶ │ sending │ ──────────────▶ │ delivered │ (끝)
    └─────────┘                        └─────────┘                 └───────────┘
        ▲                                   │
        │  네트워크·timeout·5xx·인증오류     │  잘못된 데이터로 인한 4xx(400/422 등)
        │  (retry_count++, backoff 후 재시도) │
        └───────────────────────────────────┤
                                            ▼
                                       ┌────────┐
                                       │ failed │ (영구 실패, 더 재시도 안 함)
                                       └────────┘

    프로세스가 sending 도중 죽으면? → stale_sending_seconds 경과 후 pending으로 자동 복구
```

- **pending → 즉시 응답:** 로컬 저장만 성공하면 전달 성공 여부와 무관하게
  조난자에게 접수 응답을 돌려줍니다. (재난 상황에서 사용자 대기 최소화)
- **재시도:** exponential backoff — 초기 5초, 2배씩 증가, 최대 5분, ±20% jitter,
  **무한 재시도**. 계산은 `retry.py`의 순수 함수라 단위 테스트가 결정적입니다.
- **영구 실패 판정:** "네트워크/일시적"인가 "데이터가 잘못됐나"로 나눕니다.
  네트워크·timeout·5xx·`401/403/408/429`는 재시도, 그 밖의 4xx(잘못된 본문)는
  `failed`. 재시도해도 소용없는 요청만 버려 구조 요청 유실을 최소화합니다.
- **크래시 복구:** `sending`은 전송 시작을 뜻하며 **HTTP 호출 전에 커밋**됩니다.
  forwarder가 그 사이 죽으면 행이 `sending`으로 남고, 다음 패스에서
  `stale_sending_seconds`(기본 120초)가 지난 것을 `pending`으로 되돌립니다.

### 중복 없이 저장 (멱등성)

같은 요청이 두 번 도착할 수 있습니다(예: ACK가 유실되어 forwarder가 재전송).
receiver는 이를 안전하게 처리합니다.

1. `received_rescue_requests.request_id`에 **UNIQUE 제약**을 겁니다(근본 방어선).
2. `receiver_service.store_received()`는 먼저 조회 → 있으면 `duplicate=true`로
   기존 행 반환, 없으면 삽입. 경합으로 삽입이 충돌하면 `IntegrityError`를 잡아
   역시 중복으로 처리합니다.
3. **이미 받은 요청에도 성공 ACK**(`accepted=true`)를 돌려주므로, forwarder는
   그 요청을 `delivered`로 마무리하고 더는 보내지 않습니다.

### 동시성 (웹서버 + forwarder가 같은 DB)

field 노드에서는 웹 서버와 forwarder **두 프로세스가 같은 `field.db`** 를 씁니다.
그래서 SQLite를 다음처럼 엽니다(`database.py`).

- **WAL 모드** — 읽기와 한 개의 쓰기가 공존.
- **busy_timeout=5s** — 잠금 충돌 시 즉시 실패 대신 대기.
- forwarder는 **요청 1건마다 트랜잭션을 커밋**하여, 중간에 죽어도 진행분이 남고
  재시작 시 이어서 처리합니다.
- 시간은 모두 **UTC**로 저장합니다.

### 노드 간 인증

- forwarder는 전달 시 `X-Rescue-Token`(공유 토큰)과 `X-Source-Node`(NODE_ID)
  헤더를 붙입니다.
- receiver는 `security.verify_token()`으로 **상수시간 비교**(`secrets.compare_digest`)
  하여 다르면 `401`. **토큰 값은 어떤 로그에도 출력하지 않습니다.**
- 토큰은 field·receiver에서 **동일**해야 하며, 코드/유닛 파일이 아니라
  `.env`/`EnvironmentFile`(권한 제한)에 둡니다.

## 기술 스택

Python 3.11+ (권장) · FastAPI · Uvicorn · SQLAlchemy 2.x · Pydantic 2.x ·
SQLite · httpx · Jinja2 · Vanilla JS · pytest · Ruff · systemd · Bash.

> 개발 박스에 3.10만 있는 경우에도 동작하도록 `requires-python`은 `>=3.10`으로
> 두었습니다. 운영은 3.11+를 권장합니다.

---

## 실행 순서 (한눈에)

**개발 PC에서 종단 간 확인** (인터넷·메시망 불필요, 창 3개):

```text
① receiver 서버 실행        → 구조대 수신 API + 대시보드 (:8080)
② field 웹 서버 실행         → 조난자 폼 (:8000)          ┐ 같은 field.db 공유
③ forwarder 실행            → 저장된 요청을 receiver로 전달 ┘
④ 폼에서 요청 제출 → forwarder가 전달 → 대시보드에 표시 확인
```

**운영(현장 배포)에서의 부팅 순서** (systemd가 자동 처리):

```text
network-online.target
   └▶ (field)    rescue-field-web.service  +  rescue-forwarder.service
   └▶ (receiver) rescue-receiver.service
   └▶ (relay)    rescue 서비스 없음 — 기존 HANSEL 메시(hansel-mesh@)만 실행
```

> 전제: 세 역할 모두 **기존 HANSEL 메시(bat0)가 이미 올라와 있어야** 전달이 됩니다.
> 메시 자체는 저장소 루트의 `scripts/*mesh*.sh`로 구성합니다(rescue-network 범위 밖).

구체적 명령은 아래 [개발 환경 실행](#개발-환경-실행-방법) /
[receiver·forwarder 실행](#receiver--forwarder-실행-방법-phase-2) /
[systemd 배포](#systemd-배포-phase-3)에 있습니다.

## 개발 환경 실행 방법

### Linux / macOS (Makefile)

```bash
cd rescue-network
make venv
make install
make test        # pytest
make lint        # ruff check
make run-field   # http://127.0.0.1:8000
```

### Windows (PowerShell)

```powershell
cd rescue-network
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -e ".[dev]"
pytest
ruff check .
uvicorn rescue_network.field_app:app --reload --port 8000
```

브라우저에서 `http://127.0.0.1:8000/` 를 열면 구조 요청 폼이 표시됩니다.
운영 환경(현장 Pi)에서는 `--host 0.0.0.0 --port 80` 으로 실행하며, 스마트폰은
`http://192.168.10.1/` 로 접속합니다(AP 구성은 Phase 4).

## 테스트 실행 방법

```bash
pytest                     # 전체
pytest tests/unit          # 검증/enum 단위 테스트
pytest tests/integration   # API 통합 테스트
```

모든 테스트는 **외부 인터넷 없이** 임시 디렉터리의 SQLite로 실행됩니다.

## API

### field 노드

| 메서드 | 경로 | 설명 |
|---|---|---|
| `GET` | `/` | 구조 요청 HTML 폼 |
| `POST` | `/api/rescue` | 요청 검증 후 SQLite 저장, 접수 응답(`pending`) |
| `GET` | `/api/rescue/{request_id}` | 해당 요청의 전달 상태 |
| `GET` | `/health` | 웹 서버 + DB 상태 |

`POST /api/rescue` 응답 예시:

```json
{ "request_id": "…UUID…", "delivery_status": "pending", "message": "구조 요청이 저장되었습니다." }
```

로컬 저장이 성공하면 네트워크 전달 여부와 무관하게 접수 응답을 반환합니다.

### receiver 노드 (구조대 서버)

| 메서드 | 경로 | 설명 |
|---|---|---|
| `POST` | `/api/rescue/receive` | 전달된 요청 수신(멱등). `X-Rescue-Token` 헤더 필요 |
| `GET` | `/api/received` | 수신 요청 목록(JSON, 대시보드 polling) |
| `GET` | `/dashboard` | 구조대 대시보드 HTML |
| `GET` | `/health` | 수신 서버 + DB 상태 |

`POST /api/rescue/receive` ACK 예시:

```json
{ "request_id": "…UUID…", "accepted": true, "duplicate": false }
```

같은 `request_id`가 다시 오면 새 레코드를 만들지 않고 `duplicate: true`로 성공 ACK를 반환합니다.
토큰이 없거나 틀리면 `401`을 반환하며, 토큰 값은 로그에 출력하지 않습니다.

## receiver / forwarder 실행 방법 (Phase 2)

구조대(receiver) 서버:

```bash
NODE_ROLE=receiver NODE_ID=receiver-01 RESCUE_SHARED_TOKEN=<shared> \
  uvicorn rescue_network.receiver_app:app --host 0.0.0.0 --port 8080
```

현장(field) 노드에서 전달 프로세스(웹 서버와 별도 프로세스로 실행):

```bash
NODE_ROLE=field NODE_ID=node-01 RESCUE_SHARED_TOKEN=<shared> \
  RECEIVER_URL=http://192.168.50.1:8080 \
  python -m rescue_network.forwarder
```

forwarder는 `pending` 요청을 폴링하여 `sending`으로 표시하고 receiver로 전송합니다.
성공 시 `delivered`, 실패 시 `pending`으로 되돌리고 exponential backoff(초기 5초·최대 5분·jitter)로
무한 재시도합니다. 잘못된 데이터로 인한 4xx는 `failed` 처리하고, 네트워크·timeout·5xx·인증
오류는 재시도합니다. 중간에 죽어 `sending`으로 남은 요청은 일정 시간 후 `pending`으로 복구합니다.
전달 헤더: `X-Rescue-Token`(공유 토큰), `X-Source-Node`(NODE_ID).

### 개발 PC 종단 간 예시 (창 3개, 메시망 없이)

field와 forwarder는 **같은 `DATA_DIR`** 를 써야 `field.db`를 공유합니다. receiver는
같은 `RESCUE_SHARED_TOKEN`을 써야 인증이 통과합니다. (Linux/macOS 예시)

```bash
# 창 1 — receiver
NODE_ROLE=receiver RESCUE_SHARED_TOKEN=dev-secret DATA_DIR=./data-receiver \
  ./.venv/bin/uvicorn rescue_network.receiver_app:app --port 8080
```

```bash
# 창 2 — field 웹 서버
NODE_ROLE=field NODE_ID=node-01 DATA_DIR=./data-field \
  ./.venv/bin/uvicorn rescue_network.field_app:app --port 8000
```

```bash
# 창 3 — forwarder (receiver를 localhost로 지정)
NODE_ROLE=field NODE_ID=node-01 DATA_DIR=./data-field \
  RESCUE_SHARED_TOKEN=dev-secret RECEIVER_URL=http://127.0.0.1:8080 \
  ./.venv/bin/python -m rescue_network.forwarder
```

이제 `http://127.0.0.1:8000/`에서 요청을 제출하면 몇 초 안에
`http://127.0.0.1:8080/dashboard`에 나타납니다.

## systemd 배포 (Phase 3)

운영 Raspberry Pi에서는 systemd로 서비스를 관리합니다. 서비스 파일은
`systemd/`에, 노드별 환경(비밀 토큰 포함)은 `EnvironmentFile`에 둡니다.

### 서비스 구성

| 역할 | 서비스 | 실행 |
|---|---|---|
| `field` | `rescue-field-web.service` | 구조 요청 웹/API (`uvicorn field_app`) |
| `field` | `rescue-forwarder.service` | 전달 프로세스 (`python -m …forwarder`) |
| `receiver` | `rescue-receiver.service` | 수신 API + 대시보드 (`uvicorn receiver_app`) |
| `relay` | *(없음)* | rescue 서비스 없음 — 기존 `hansel-mesh@`만 실행 |

공통 정책: 전용 시스템 사용자 `rescue`, `WorkingDirectory`/`EnvironmentFile` 명시,
`Restart=always` + `RestartSec`, 로그는 **journald**, 쓰기 데이터는
`StateDirectory=rescue-network`(→ `/var/lib/rescue-network`), 하드닝
(`NoNewPrivileges`, `ProtectSystem=strict`, `ProtectHome`, `PrivateTmp`).
비밀 토큰은 유닛 파일이 아니라 권한 제한된 `EnvironmentFile`에만 둡니다.

### 배포 전제 (경로는 조정 가능)

기본 경로는 코드 `/opt/rescue-network`, venv `/opt/rescue-network/.venv`,
환경파일 `/etc/rescue-network/<role>.env`, 데이터 `/var/lib/rescue-network`입니다.

```bash
# 1) 코드 배치 + venv 구성 (예: field/receiver Pi 공통)
sudo mkdir -p /opt/rescue-network
sudo rsync -a --exclude .venv --exclude data ./ /opt/rescue-network/
cd /opt/rescue-network
sudo python3 -m venv .venv
sudo ./.venv/bin/pip install -e .
```

### 서비스 설치 (네트워크는 건드리지 않음)

`install-services.sh`는 **애플리케이션 서비스만** 설치합니다. 네트워크 설정을
변경하지 않고, 서비스를 자동 enable/start 하지도 않으며, 실제 적용 전
`--dry-run`으로 계획을 먼저 볼 수 있습니다.

```bash
# 먼저 무엇을 할지 확인 (아무것도 바꾸지 않음)
sudo ./scripts/install-services.sh field --dry-run

# 실제 설치: 사용자 생성 + 데이터 디렉터리 + EnvironmentFile 예시 복사 + 유닛 설치
sudo ./scripts/install-services.sh field      # receiver / relay 도 동일
```

### 토큰 설정 후 시작 (순서 주의)

```bash
# 1) 비밀 토큰을 편집 (field·receiver 동일 값!)
sudo nano /etc/rescue-network/field.env       # RESCUE_SHARED_TOKEN=...

# 2) 부팅 시 자동 시작 + 지금 시작
sudo systemctl enable --now rescue-field-web.service
sudo systemctl enable --now rescue-forwarder.service
#   receiver 노드: sudo systemctl enable --now rescue-receiver.service

# 3) 상태·로그 확인
systemctl status rescue-field-web.service
journalctl -u rescue-forwarder.service -f
```

`network-online.target` 이후 실행되도록 순서가 지정되어 있고, 크래시 시
자동 재시작됩니다(`StartLimitIntervalSec=0`으로 무한 재시작).

## 조난자 AP 설정 (Phase 4)

현장 노드가 스마트폰을 받는 Wi-Fi AP(hostapd + dnsmasq)를 구성합니다. 스마트폰은
`http://192.168.10.1/` 로 접속해 구조 요청 폼을 엽니다. AP 인터페이스는 보통
**Pi 내장 Wi-Fi**(brcmfmac)이고, USB 동글은 기존 HANSEL 메시용으로 남겨둡니다.

> **captive portal 없음 / NAT·인터넷 공유 없음 / IP forwarding 켜지 않음.**
> 초기 단계는 "스마트폰에서 주소를 직접 입력하면 폼이 열린다"만 검증합니다.

### ⚠️ 네트워크 설정 안전 규칙

이 스크립트들은 SSH 연결을 끊을 수 있는 네트워크 변경을 다루므로 다음을 지킵니다.

- **기본은 dry-run** — 아무것도 바꾸지 않고 렌더링된 설정과 실행 계획만 출력.
- 실제 적용은 **명시적 플래그**(`--apply`, `--up`)가 있을 때만.
- 설정 파일 변경 **전에 백업**하고, `rollback-network.sh`로 복구 가능.
- 인터페이스 이름을 **하드코딩하지 않음**(항상 `ap.env`에서 읽음).
- hostapd/dnsmasq/NetworkManager를 **자동 재시작하지 않음**(`--up`은 별도·명시적).
- default route 삭제·iptables/nftables flush·IP forwarding·NAT를 **하지 않음**.
- 커밋된 `*.example` 파일로는 `--apply` 를 **거부**(플레이스홀더 암호 방지).

### 실행 순서 (현장 Pi에서)

```bash
# 1) 하드웨어가 AP를 지원하는지 확인 (아무것도 바꾸지 않음)
./scripts/check-capabilities.sh          # OS/Pi/무선모드/batman/드라이버/rfkill 점검
./scripts/detect-network.sh              # 내장 Wi-Fi=AP, USB 동글=mesh 제안

# 2) 설정값 준비
sudo cp config/examples/ap.env.example /etc/rescue-network/ap.env
sudo nano /etc/rescue-network/ap.env     # AP_INTERFACE / SSID / 국가 / 채널 / 암호 등

# 3) 먼저 dry-run 으로 렌더링 결과 확인 (변경 없음)
./scripts/configure-ap.sh --env /etc/rescue-network/ap.env

# 4) 설정 파일 기록 (백업 후, 서비스 재시작은 안 함)
sudo ./scripts/configure-ap.sh --env /etc/rescue-network/ap.env --apply

# 5) AP 올리기 (명시적 — 여기서 실제로 hostapd/dnsmasq 시작)
sudo ./scripts/configure-ap.sh --env /etc/rescue-network/ap.env --up

# 6) 검증 (읽기 전용): 인터페이스 UP, 192.168.10.1 할당, 폼 200 응답
./scripts/verify-network.sh

# 되돌리기: 설정 복구 + AP 내리기
sudo ./scripts/rollback-network.sh --apply
sudo ./scripts/configure-ap.sh --env /etc/rescue-network/ap.env --down
```

### 스마트폰 접속 검증

1. 스마트폰 Wi-Fi에서 `Rescue-Network`(SSID) 에 연결(암호는 `AP_PASSPHRASE`).
2. DHCP로 `192.168.10.100~200` 범위 주소를 받는지 확인.
3. 브라우저에서 **`http://192.168.10.1/`** 입력 → 구조 요청 폼이 열리면 성공.
   (`192.168.50.x` 는 메시 내부 주소이지 스마트폰 접속 주소가 아닙니다.)

### AP 환경 변수 (`ap.env`)

`AP_INTERFACE`, `AP_SSID`, `AP_COUNTRY`, `AP_BAND`(2.4/5), `AP_CHANNEL`,
`AP_ADDRESS`(CIDR), `DHCP_RANGE_START`/`DHCP_RANGE_END`/`DHCP_LEASE`,
`AP_SECURITY`(wpa2/open) + `AP_PASSPHRASE`, `HOSTAPD_CONF`/`DNSMASQ_CONF`(설치
경로). 템플릿은 `config/hostapd/` · `config/dnsmasq/` 에 있습니다.

## 메시망: 기존 HANSEL 메시 재사용 (Phase 5)

**rescue-network는 자체 메시를 구성하지 않습니다.** 이 로봇 Pi들은 이미 카메라·제어용
B.A.T.M.A.N.-adv 메시(`bat0`, `MESH_ID=HANSEL_MESH`, 192.168.50.0/24)를 형성하므로,
구조 요청 전달은 **그 메시 위에 얹힌 또 하나의 애플리케이션 트래픽**일 뿐입니다.

- 메시 구성/기동은 저장소 루트의 기존 도구가 담당합니다:
  `scripts/install_mesh.sh`, `scripts/start_role_network.sh`, `configs/*.env`,
  `services/hansel-mesh@.service`.
- 앱이 하는 일은 `RECEIVER_URL`을 **구조대 노드의 기존 bat0 IP**(관례상 base 노드
  `192.168.50.1`)로 두는 것뿐. Linux 라우팅 + B.A.T.M.A.N.-adv가 경로를 정합니다.
- 그래서 **forwarder/receiver 코드는 메시를 전혀 몰라도** 되고, 메시가 올라와 있으면
  종단 간 전달이 저절로 동작합니다.

### 종단 간 전달 검증 (기존 메시가 올라온 상태에서)

```bash
# 0) (선행) 기존 HANSEL 메시 기동 — rescue 범위 밖, 저장소 루트 도구 사용
sudo ./scripts/install_mesh.sh
sudo ./scripts/start_role_network.sh base      # 노드별로 base/head/node1 …

# 1) 메시가 살아있는지 확인 (읽기 전용)
ping 192.168.50.1                              # 상대 노드 bat0
sudo batctl if ; sudo batctl n ; sudo batctl o # 인터페이스 / 이웃 / 오리지네이터
./scripts/verify-network.sh                     # AP + bat0 존재 여부 점검

# 2) 앱 종단 간: field에서 receiver 헬스 확인 후 forwarder 가동
curl http://192.168.50.1:8080/health
NODE_ROLE=field RESCUE_SHARED_TOKEN=<shared> RECEIVER_URL=http://192.168.50.1:8080 \
  python -m rescue_network.forwarder
# → field 폼에 넣은 요청이 기존 메시를 건너 receiver 대시보드에 뜨면 성공
```

> 참고: rescue-network의 네트워크 스크립트는 **조난자 AP(`configure-ap.sh`)** 와
> **읽기 전용 진단·검증(`check-capabilities.sh`/`detect-network.sh`/`verify-network.sh`)**
> 만 제공합니다. 메시 bring-up 스크립트는 의도적으로 두지 않았습니다(중복 방지).

## 선택 기능 (Phase 6)

코어(Phase 1–5)를 건드리지 않고 붙는 독립 기능들입니다. 각각 환경변수/플래그로
켭니다.

### ⑥ 자동 롤백 (`--commit-timeout`)
원격에서 AP/메시를 잘못 켜 SSH가 끊기는 사고를 방지합니다. `--up` 뒤 제한 시간
안에 `--commit` 하지 않으면 워치독이 자동으로 이전 상태로 되돌립니다.

```bash
sudo ./scripts/configure-ap.sh --env /etc/rescue-network/ap.env --up --commit-timeout 90
#   연결이 살아있으면 90초 안에 확정:
sudo ./scripts/configure-ap.sh --env /etc/rescue-network/ap.env --commit
#   확정 안 하면 → 자동 --down (원상복구)
```
(AP 구성 스크립트에 적용되는 안전장치입니다.)

### ⑤ 모니터링·알림
두 앱에 Prometheus 형식 `GET /metrics`(대기/전송중/완료/실패 개수, 최고령 대기
나이)가 생깁니다. forwarder는 매 패스마다 헬스 요약을 로그로 남기고, **실패 발생**
또는 **오래 밀린 pending**이 있으면 경고합니다(옵션 `ALERT_WEBHOOK`로 POST).

```bash
curl http://127.0.0.1:8000/metrics    # field
curl http://127.0.0.1:8080/metrics    # receiver
```

### ③ 강화된 노드 인증 (HMAC 서명)
`RESCUE_REQUIRE_SIGNATURE=1` 이면 정적 토큰 대신 **요청마다 HMAC 서명 + 타임스탬프**
를 요구합니다. 본문 위변조와 재전송(replay)을 막습니다(field·receiver 양쪽에 설정).

```bash
NODE_ROLE=receiver RESCUE_REQUIRE_SIGNATURE=1 RESCUE_SHARED_TOKEN=<secret> \
  uvicorn rescue_network.receiver_app:app --port 8080
NODE_ROLE=field RESCUE_REQUIRE_SIGNATURE=1 RESCUE_SHARED_TOKEN=<secret> \
  RECEIVER_URL=http://192.168.50.1:8080 python -m rescue_network.forwarder
```
헤더: `X-Rescue-Signature`(HMAC-SHA256), `X-Rescue-Timestamp`. 서명이 틀리거나
타임스탬프가 허용 오차를 벗어나면 `401`.

### ④ 실시간 대시보드 갱신 (SSE)
대시보드가 `GET /api/received/stream`(Server-Sent Events)에 한 번 연결해, 새 요청이
도착하는 즉시 표시합니다. EventSource가 없거나 끊기면 자동으로 기존 5초 polling으로
폴백합니다. 서버/앱 변경 없이 대시보드를 열기만 하면 동작합니다.

### ① Captive Portal
`CAPTIVE_PORTAL=1`(앱) + `AP_CAPTIVE=1`(`ap.env`) 이면, 스마트폰이 AP에 접속할 때
OS의 인터넷 확인 요청을 가로채 **구조 폼이 자동으로 뜹니다**(주소 입력 불필요).

```bash
# AP: 와일드카드 DNS 활성화
sudo ./scripts/configure-ap.sh --env /etc/rescue-network/ap.env --apply   # AP_CAPTIVE=1
# 앱: 포털 라우트 활성화
NODE_ROLE=field CAPTIVE_PORTAL=1 uvicorn rescue_network.field_app:app --port 80
```

### ② 구조대 노트북 Ethernet 분리
receiver Pi의 유선 포트로 노트북을 연결해 대시보드를 봅니다. **IP forwarding은 이
스크립트에서만**(receiver 한정·백업·되돌리기) `GATEWAY_FORWARD=1`일 때 켜집니다.

```bash
./scripts/configure-gateway.sh --env /etc/rescue-network/gateway.env          # dry-run
sudo ./scripts/configure-gateway.sh --env /etc/rescue-network/gateway.env --apply
sudo ./scripts/configure-gateway.sh --env /etc/rescue-network/gateway.env --up
# 노트북을 유선 연결 → http://10.20.0.1:8080/dashboard
```

## 환경 변수

`.env.example` 를 `.env` 로 복사해 사용합니다 (`.env` 는 커밋하지 않습니다).

| 변수 | 기본값 | 단계 | 설명 |
|---|---|---|---|
| `NODE_ROLE` | `field` | 1 | `field` / `relay` / `receiver` |
| `NODE_ID` | `node-01` | 1 | 노드 식별자(로그·요청 출발지) |
| `DATA_DIR` | `./data` | 1 | SQLite 파일이 저장될 쓰기 가능 디렉터리 |
| `WEB_HOST` / `WEB_PORT` | `0.0.0.0` / `80` | 1 | 웹 서버 바인딩 |
| `AP_INTERFACE` / `AP_ADDRESS` | `wlan0` / `192.168.10.1/24` | 4 | 조난자 AP 인터페이스·주소 |
| `RECEIVER_URL` | `http://192.168.50.1:8080` | 2 | 구조대 수신 서버(기존 bat0 IP) |
| `RESCUE_SHARED_TOKEN` | `change-me` | 2 | 노드 간 공유 토큰(로그에 출력 금지) |
| `FORWARDER_POLL_INTERVAL_SECONDS` | `2.0` | 2 | forwarder 폴링 주기 |
| `DELIVERY_TIMEOUT_SECONDS` | `10.0` | 2 | 전달 HTTP 타임아웃 |
| `FORWARDER_BATCH_SIZE` | `20` | 2 | 한 패스에 처리할 최대 요청 수 |
| `STALE_SENDING_SECONDS` | `120.0` | 2 | 이 시간 넘게 `sending`이면 `pending`으로 복구 |
| `REQUIRE_SIGNATURE` | `false` | 6 | HMAC 서명 인증 요구(③) |
| `SIGNATURE_MAX_SKEW_SECONDS` | `300.0` | 6 | 서명 타임스탬프 허용 오차(replay 창) |
| `CAPTIVE_PORTAL` | `false` | 6 | captive portal 라우트 활성화(①) |
| `ALERT_PENDING_AGE_SECONDS` | `600.0` | 6 | 이 나이 넘는 pending이면 경고(⑤) |
| `ALERT_WEBHOOK` | `` | 6 | 경고를 POST할 웹훅(비면 로그만) |

개발 시에는 `.env` 파일을, 운영 시에는 systemd `EnvironmentFile`을 사용합니다.
두 경우 모두 값은 OS 환경변수로 주입되어 `config.py`의 Pydantic Settings가 읽습니다.

## 데이터 모델

두 노드는 **서로 다른 DB 파일**을 씁니다(field.db / receiver.db).

**field 노드 — `rescue_requests`** (전달 상태를 추적):
`request_id`(UUID, PK), `source_node_id`, `people_count`, `injury_status`,
`condition`, `message`, `latitude`/`longitude`/`location_accuracy`/`location_text`
(nullable), `delivery_status`(`pending`/`sending`/`delivered`/`failed`),
`retry_count`, `next_retry_at`, `last_attempt_at`, `last_error`, `created_at`,
`delivered_at`.

**receiver 노드 — `received_rescue_requests`** (중복 제거·수신 기록):
`id`(PK), `request_id`(**UNIQUE**), `source_node_id`, `people_count`,
`injury_status`, `condition`, `message`, 위치 필드, `original_created_at`(현장
생성 시각), `received_at`(수신 시각).

모든 시간은 UTC로 저장됩니다. SQLite는 WAL 모드 + `busy_timeout=5s`로 웹 서버와
forwarder의 동시 접근을 견디며, `request_id` UNIQUE 제약이 멱등성의 근본
방어선입니다.

## 프로젝트 구조

```text
rescue-network/
├── src/rescue_network/
│   ├── config.py          # Pydantic Settings (환경변수)
│   ├── database.py        # SQLite 엔진(WAL/busy_timeout), 세션
│   ├── models.py          # ORM: rescue_requests / received_rescue_requests
│   ├── schemas.py         # Pydantic 검증 + enum
│   ├── security.py        # 공유 토큰 + HMAC 서명 검증
│   ├── retry.py           # backoff 계산 + 오류 분류 (순수 함수)
│   ├── monitoring.py      # 메트릭 렌더 + 알림 판정 (순수 함수, Phase 6)
│   ├── captive.py         # captive portal 라우트 (Phase 6, opt-in)
│   ├── field_app.py       # field FastAPI 앱 (+/metrics, captive)
│   ├── receiver_app.py    # receiver FastAPI 앱 (+/metrics, SSE stream)
│   ├── forwarder.py       # 별도 전달 프로세스 (폴링 루프 + 헬스/알림)
│   ├── repositories/      # DB 접근 계층
│   ├── services/          # 비즈니스 로직(생성/멱등저장/전달코어)
│   ├── templates/         # rescue_form.html / dashboard.html
│   └── static/            # css / js (vanilla)
├── tests/{unit,integration}/
├── systemd/               # 3개 서비스 유닛 (Phase 3)
├── config/
│   ├── examples/          # field/receiver/relay/ap/gateway EnvironmentFile 예시
│   ├── hostapd/           # hostapd.conf 템플릿 (Phase 4)
│   └── dnsmasq/           # dnsmasq 템플릿 (Phase 4 / Phase 6 게이트웨이)
└── scripts/
    ├── install-services.sh       # 안전한 서비스 설치(dry-run)
    ├── lib/common.sh             # 공용 헬퍼(로그·dry-run·백업)
    ├── check-capabilities.sh     # 하드웨어/모드 점검(읽기전용)
    ├── detect-network.sh         # 인터페이스 탐지/제안(읽기전용)
    ├── configure-ap.sh           # 조난자 AP 구성(dry-run·백업·--up/--down·--commit-timeout)
    ├── configure-gateway.sh      # 유선 노트북 게이트웨이(Phase 6 ②)
    ├── verify-network.sh         # AP·(기존)메시·폼 도달 검증(읽기전용)
    └── rollback-network.sh       # 백업 복구
# 메시 bring-up은 rescue 범위 밖 — 저장소 루트의 scripts/*mesh*.sh 사용
```

## 로드맵

- **Phase 1 ✅** 로컬 구조 요청 앱
- **Phase 2 ✅** receiver API + 중복 제거 + forwarder(재시도) + 대시보드
- **Phase 3 ✅** systemd 배포 (field-web / forwarder / receiver)
- **Phase 4 ✅** 조난자 AP (hostapd/dnsmasq, dry-run·백업·rollback)
- **Phase 5 ✅** 기존 HANSEL B.A.T.M.A.N.-adv 메시 재사용 + 종단 간 전달 검증
- **Phase 6 ✅** 선택 기능: 자동 롤백·모니터링/알림·HMAC 인증·SSE 실시간·captive portal·유선 게이트웨이
