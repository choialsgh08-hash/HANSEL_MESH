# 영상 경로 UI

HANSEL_Final_radar_recovery_20261002 브랜치의
87add2cbf618f14ee6c7ecd8d98a2b55b44ec127을 기준으로 추가한 대시보드 기능이다.
Pi의 실행 파일과 설정을 자동으로 배포하거나 변경하지 않는다.

## 표시하는 정보

- 영상 방향 Head → Base의 선택 경로, 직결/중계, 무선 홉 수.
- Base → Head 경로는 별도로 계산한다. 반대 방향을 뒤집어 추정하지 않는다.
- 각 노드가 목적지까지 평가한 TQ와 정보 나이.
- 토폴로지 위 파란 방향 화살표. RSSI/TQ 링크가 아직 없으면 점선.
- 최근 경로 변경 이력과 변경 당시 영상 FPS. 화면 이력은 최근 약 6분이다.
  --log를 지정하면 전체 경로와 영상 품질을 JSONL에 함께 저장한다.
- 보고 누락, 식별 안 된 MAC, 오래된 경로, 순환 경로는 확인 불가로 표시한다.

이는 각 Pi의 selected originator table을 연결한 추정이다.
비동기 수집 때문에 전환 순간의 표가 일치하지 않을 수 있다.
개별 RTP 패킷을 추적한 결과도, 모든 패킷이 이 경로를 통과했다는 증명도 아니다.
확인 불가를 직접 연결로 표시하지 않는다.
TQ는 0–255 경로 평가값이고 Mbps나 실제 영상 손실률이 아니다.
현재 파서는 BATMAN IV의 (TQ) 형식을 사용한다. BATMAN V 표는 지원하지 않는다.
802.11s 자체 forwarding이 켜져 있으면 표시 홉과 실제 무선 홉이 다를 수 있다.

## 데모

이 폴더에서:

    python monitor/dashboard.py --demo --http-port 8086

http://127.0.0.1:8086 을 연다. 약 20초마다 직결/3홉 중계가 바뀌는 합성 데이터다.
화면 제목에 데모 데이터가 표시된다. 실제 영상과 실제 Pi 측정값이 아니다.

## 기존 장비에 적용

PC 또는 대시보드 수집기를 실행하는 장비의 기존 HANSEL_MESH에 다음 파일을 적용한다.
기존 파일은 먼저 백업한다.

    monitor/dashboard.py
    monitor/route_state.py        # 새 파일
    monitor/web/index.html

Head, Node1, Node2, Base 네 장비의 기존 HANSEL_MESH에는 다음 파일을 적용한다.

    monitor/metrics_agent.py

이 agent는 메쉬 무선 인터페이스의 MAC과 BATMAN 헤더의 primary MAC을 보고한다.
bat0 ARP MAC과 originator MAC이 다른 경우에도 노드 이름을 식별하기 위한 정보다.
기존 ROS2 쪽 agent가 별도로 돌아간다는 이유만으로 이 필드가 수집되지는 않는다.
아래 agent 또는 수정된 HANSEL_MESH의 기존 metrics 서비스를 사용해야 한다.
같은 노드의 기존 agent와 새 agent를 동시에 실행하지 않는다.

PC 수집기:

    python3 monitor/dashboard.py --http-port 8080 --log route_video_session.jsonl

각 Pi에서 하나씩 실행한다. --self는 실제 장비 역할에 맞춰 바꾼다.
예: Head=head, Base=base, Node1=node1, Node2=node2.
무선 인터페이스가 wlan1이 아니면 --mesh-if도 수정한다.

    cd ~/HANSEL_MESH
    sudo python3 monitor/metrics_agent.py --self head --mesh-if wlan1 --bat-if bat0 --ping base --loop --interval 5 --send 192.168.60.2:7100

수집기 주소가 다르면 --send의 주소를 바꾼다. PC에서 UDP 7100 수신이 가능해야 한다.
설정 파일이나 제어 경로를 바꾸는 작업은 없다. root 권한은 iw/batctl 조회를 위한 것이다.
네 노드의 보고가 도착한 뒤 현재 경로가 계산된다. 약 12초 이상 지난 보고/경로는 만료된다.
전환 기록은 2초 수집 주기보다 짧은 경로 변화를 놓칠 수 있다.
TQ 표시의 'Head의 목적지 TQ'는 Head→Base 전체 경로 평가값이며 Head→다음 홉만의 TQ가 아니다.

영상 품질을 함께 기록하려면 PC에서 video_probe를 수집기에 연결한다.
기존 ffplay와 같은 UDP 5600 포트를 동시에 수신하지 않는다.
최신 통합 실행기를 사용할 경우 --no-camera로 자동 송수신을 생략하고,
Head 송신과 아래 수신기를 따로 실행하는 테스트 구성을 사용한다.

    python3 monitor/video_probe.py --transport rtp --port 5600 --target-fps 15 --send 127.0.0.1:7100 --log video_quality.jsonl

목표 FPS는 실제 송신 프로파일에 맞춘다. video_probe.py는 기존 저장소의 파일을 사용한다.
ffplay만 실행하면 이 대시보드에 FPS/오류 샘플을 전송하지 않는다.
네트워크 경로 UI는 video_probe 실행 여부와 독립적으로 작동한다.

## ffplay와 압축

Head의 rpicam-vid/libcamera-vid가 --codec h264로 압축한다.
ffmpeg의 -c:v copy는 재압축 없이 RTP로 포장한다.
PC의 ffplay는 압축된 스트림을 디코딩해서 표시한다.
ffplay를 브라우저 플레이어로 바꾸는 것만으로 전송량이 감소하지 않는다.
전송량은 Head의 해상도/FPS/비트레이트 설정으로 조절한다.
현재 UI는 플레이어를 바꾸지 않으며, 라우팅 표만 읽는다.

## 검증

    python -m unittest tests.test_metrics_agent tests.test_dashboard_events tests.test_video_routes tests.test_quality_supervisor tests.test_quality_supervisor_async

기존 메트릭/이벤트/영상 품질 감시 테스트와 새 경로 테스트 44개 통과.
직결, 3홉, 비대칭 경로, 만료 보고, OGM 만료, 미식별 MAC,
누락 보고, 중복 MAC, 전환 중 순환과 API 노출을 확인했다.
브라우저에서 데모 경로 표시 및 JavaScript 구문을 확인했다.
실제 Pi의 BATMAN 출력, 경로 전환과 영상 통과는 현장 검증이 필요하다.
