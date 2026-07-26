# 동글 도착 후 실행 절차

AR9271 동글이 오면 각 Pi에 꽂고 아래를 순서대로 실행한다. 대부분 기존
스크립트를 그대로 쓴다 — 동글 때문에 새로 만든 건 `detect_ar9271.sh`뿐이다.

전제: 코드가 각 Pi의 `~/HANSEL_MESH`(또는 `/home/hansel/HANSEL_MESH`)에 배포돼 있음.

## 0. 각 Pi 공통 준비 (최초 1회)

```bash
cd ~/HANSEL_MESH
sudo ./scripts/install_mesh.sh
```

## 1. 동글 꽂고 인식 확인 (각 Pi)

```bash
./scripts/detect_ar9271.sh
```

5개 항목이 모두 `[ OK ]`면 준비 완료. `[MISS]`가 있으면:

- USB 안 보임 → 동글 재삽입 후 `lsusb`
- 펌웨어 없음 → `sudo apt install firmware-atheros` 후 재부팅
- 인터페이스가 `wlan1`이 아닌 다른 이름 → `iw dev`로 실제 이름 확인 후
  `configs/<role>.env`의 `MESH_IF`를 그 이름으로 수정

## 2. mesh 시작 (각 Pi, role만 바꿔서)

```bash
sudo ./scripts/enable_mesh_autostart.sh base   # head / node1 / node2 각자
sudo systemctl start hansel-mesh@base
```

## 3. 상태 확인 (각 Pi)

```bash
sudo ./scripts/check_mesh.sh base
sudo batctl n      # 직접 이웃
sudo batctl o      # originator / next-hop
```

## 4. 노트북 연결 + 도달 확인

노트북과 base를 랜선으로 연결한 뒤:

```bash
ping -c3 192.168.60.1     # base 관리망
ping -c3 192.168.50.10    # head (over mesh)
ping -c3 192.168.50.11    # node1
ping -c3 192.168.50.12    # node2
```

## 5. 로봇 제어 (모터 서버는 안전상 수동 시작)

각 노드:

```bash
sudo systemctl start hansel-control@head   # head / node1 / node2 각자
```

노트북:

```bash
python3 controller/mesh_control_client.py --live --target all
```

## 6. 영상 + 모니터 (노트북)

```bash
python3 monitor/video_probe.py --transport rtp --send 127.0.0.1:7100
python3 monitor/dashboard.py               # http://localhost:8080
```

## 성공 기준

- `detect_ar9271.sh`: 5/5 `[ OK ]`
- `batctl n` / `batctl o`에 다른 노드가 보임
- base ↔ head ping 성공
- 다중 홉 ping(중간 노드 경유) 성공, 중간 노드 제거 시 경로가 바뀜(재수렴)

## 다음: 측정과 튜닝

- 재연결 감지/표시 기준: [reconnect_detection.md](reconnect_detection.md)
- 임계값 산출: `python3 monitor/calibrate_thresholds.py` → `configs/quality.env`
- 재연결 끊김 튜닝(원인 A/B): [reconnect_tuning.md](reconnect_tuning.md)
