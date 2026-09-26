# delivery_status 상태 흐름

구조 요청 하나의 전달 상태(`delivery_status`)는 4개다.

| 상태 | 의미 |
|---|---|
| `pending` | 저장됨, 아직 안 보냄 |
| `sending` | 지금 보내는 중 |
| `delivered` | 성공 (구조대가 받음) |
| `failed` | 영구 실패 (재시도 무의미) |

## 흐름

```text
pending ──forwarder가 집음──▶ sending ──유효한 ACK──▶ delivered ✅ (끝)
   ▲                            │
   │                            ├─ 네트워크/타임아웃/5xx 등 일시적 오류 → pending (재시도)
   │                            └─ 잘못된 데이터(4xx) → failed (폐기)
   │
   └─ sending 도중 프로세스가 죽으면: 일정 시간 후 자동으로 pending 복구
```

## 요약
- 정상 경로: `pending → sending → delivered`
- 일시적 오류: `sending → pending` 으로 되돌려 재시도(지수 백오프, 무한)
- 데이터 오류(4xx): `sending → failed` 로 폐기
- 크래시 복구: `sending`에 갇힌 요청은 자동으로 `pending`으로 되돌려 유실 방지
