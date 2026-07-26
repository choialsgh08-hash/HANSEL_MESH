# 재연결 끊김 튜닝 가이드

메시 재연결 시 영상 끊김은 두 원인이 겹친다. 먼저 어느 쪽이 주범인지
**측정**한 뒤(추측 금지), 해당 원인의 레버만 조정한다.

```
링크 끊김 ─▶ neighbor_lost ─▶ route_changed ─▶ 영상 복구
                            └ 원인 A: 재수렴 ┘└ 원인 B: 키프레임 대기 ┘
```

## 0. 먼저 측정한다

`metrics_agent`(재연결 이벤트)와 `video_probe`(영상 복구) 로그를 모아
분석기에 넣으면 A/B 구간이 초 단위로 분해된다.

```bash
python3 monitor/calibrate_thresholds.py --events reconnect_events.jsonl
```

출력의 `reconverge_s`(A)와 `video_recovery_s`(B) 중 **p50/p95가 큰 쪽**이 주범이다.
- B가 크면 → 아래 "원인 B" 레버
- A가 크면 → 아래 "원인 A" 레버

측정 없이 커널/인코더를 바꾸지 않는다.

---

## 원인 B: 키프레임 대기 (카메라)

링크가 복구돼도 H.264는 다음 **키프레임(I-frame)** 이 와야 다시 그려진다.
GOP(키프레임 간격)가 길면 복구된 뒤에도 화면이 그만큼 멈춘다.

### 레버 ①: GOP 단축 (지금 가능, config 레벨)

```
옵션 이름   : CAMERA_INTRA  (configs/camera.env)
의미        : 키프레임 사이 프레임 수. rpicam-vid --intra 로 전달됨.
현재 기본값 : 비어있으면 프레임레이트(FPS)와 동일 = 약 1키프레임/초
확인 명령   : grep CAMERA_INTRA /etc/hansel-mesh/camera.env
              (실제 적용값은 head 카메라 로그의 --intra 확인)
설정 명령   : /etc/hansel-mesh/camera.env 에 CAMERA_INTRA=8  (15fps에서 ~0.5초)
              sudo systemctl restart hansel-camera
예상 효과   : 재연결 후 최대 키프레임 대기 = INTRA/FPS 초로 감소
부작용      : bitrate 상승(키프레임이 큼) → 링크 여유 없으면 오히려 손실↑
측정 지표   : calibrate_thresholds 의 video_recovery_s p50/p95
원복 명령   : CAMERA_INTRA="" 로 되돌리고 systemctl restart hansel-camera
```

> 주의: GOP를 너무 줄이면 bitrate가 올라 링크를 더 압박한다. 원인 B가
> 측정으로 주범일 때만, 그리고 링크 여유(RSSI/TQ) 범위에서 줄인다.

### 레버 ②: 진짜 on-demand 키프레임 (향후 과제, 미구현)

현재 인코더 `rpicam-vid`는 실행 중 "지금 키프레임 강제" 신호를 못 받는다.
route 변화 시 즉석 IDR을 쏘려면 **인코더를 GStreamer로 교체**해야 한다:

```
rpicam-vid(원본) → GStreamer(v4l2h264enc/x264enc) 인코딩
  → 파이프라인에 GstForceKeyUnit 이벤트 주입으로 IDR 강제 가능
  → metrics_agent 의 route_changed 이벤트를 트리거로 연결
```

미구현 이유:
- 파이프라인 재작성(인코더 계층 변경)으로 범위가 큼
- 실제 카메라 하드웨어 없이 검증 불가
- GOP 단축(레버 ①)으로 1초 이하까지 줄일 수 있어, B가 주범이라도 우선 ①로 대응 가능

측정에서 ①로도 부족하다고 확인되면 그때 착수한다. 착수 시 별도 변경 제안서로
작성한다(측정된 문제 / 재현 / 최소 변경안 / 부작용 / 회귀 / 원복).

---

## 원인 A: 경로 재수렴 (batman-adv)

링크가 끊기면 batman이 이웃 소실을 감지하고 originator/next-hop을 다시
계산하는 동안 패킷이 유실된다. 아래 값들로 재수렴 속도를 조절한다.

> 기본값은 커널 버전에 따라 다를 수 있으므로 **확인 명령으로 현재값을 먼저
> 읽고** 조정한다. 프로젝트는 `start_mesh.sh`가 sysfs에 기록한다
> (`BATMAN_ORIG_INTERVAL`, `BATMAN_HOP_PENALTY` in configs/*.env).

### orig_interval (OGM 송신 주기)

```
현재 기본값 : 1000 (ms) — 커널 기본
확인 명령   : cat /sys/class/net/bat0/mesh/orig_interval
              (또는 sudo batctl -m bat0 orig_interval)
설정 명령   : configs/<role>.env 에 BATMAN_ORIG_INTERVAL=500 후 재시작
              즉시: echo 500 | sudo tee /sys/class/net/bat0/mesh/orig_interval
예상 효과   : 이웃/경로 갱신이 빨라져 재수렴 시간 감소
부작용      : OGM 트래픽 증가(오버헤드) → 대역폭/혼잡 여유 소모
측정 지표   : calibrate_thresholds 의 reconverge_s p50/p95
원복 명령   : echo 1000 | sudo tee /sys/class/net/bat0/mesh/orig_interval
```

### hop_penalty (홉당 경로 페널티)

```
현재 기본값 : 15 (0-255) — 커널 기본
확인 명령   : cat /sys/class/net/bat0/mesh/hop_penalty
설정 명령   : configs/<role>.env 에 BATMAN_HOP_PENALTY=30 후 재시작
예상 효과   : 값↑ 이면 멀티홉보다 직접/짧은 경로 선호 → 경로 흔들림(flapping) 완화
부작용      : 과하면 우회 경로를 늦게 선택해 릴레이 활용도 저하
측정 지표   : route_changed 이벤트 빈도(경로 흔들림), reconverge_s
원복 명령   : echo 15 | sudo tee /sys/class/net/bat0/mesh/hop_penalty
```

### elp_interval (BATMAN_V ELP 링크 측정 주기, hard interface)

```
현재 기본값 : 500 (ms) — 커널 기본 (BATMAN_V)
확인 명령   : cat /sys/class/net/<MESH_IF>/batman_adv/elp_interval
              (또는 sudo batctl -m bat0 hardif <MESH_IF> elp_interval)
설정 명령   : echo 250 | sudo tee /sys/class/net/<MESH_IF>/batman_adv/elp_interval
예상 효과   : 링크 품질 변화를 더 빨리 반영 → 나빠진 링크를 더 빨리 이탈
부작용      : ELP 프로브 트래픽 증가
측정 지표   : detect_delay_s(강제 차단 시) / reconverge_s
원복 명령   : echo 500 | sudo tee /sys/class/net/<MESH_IF>/batman_adv/elp_interval
```

---

## 커널 소스 수정은 마지막 수단

위 설정값으로 해결되지 않고, 측정으로 다음 중 하나가 확인될 때만 커널
(batman-adv / ath9k_htc) 수정을 **제안**한다(바로 수정 금지):

- 링크가 끊겼는데 경로 제거가 지나치게 늦음
- 실제 처리량과 BATMAN_V metric이 크게 불일치
- 경로가 반복적으로 흔들림(설정으로 해결 불가)
- 이동 중 경로 전환이 요구 시간을 못 맞춤
- USB/드라이버 상태가 장시간 정지
- 필요한 정보가 Generic Netlink로 노출되지 않음

제안 시 형식: 측정된 문제 / 재현 절차 / 현재 동작 / 기대 동작 / 관련 파일·함수 /
최소 변경안 / 예상 부작용 / 회귀 테스트 / 원복 방법.
