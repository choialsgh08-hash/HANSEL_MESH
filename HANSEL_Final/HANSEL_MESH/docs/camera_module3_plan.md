# Raspberry Pi Camera Module 3 Plan

## 1. 사용할 카메라

사용 카메라:

```text
Raspberry Pi Camera Module 3

Camera Module 3는 Sony IMX708 기반 12MP 카메라이며 powered autofocus를 지원한다.

2. 소프트웨어 방향

Raspberry Pi OS Bookworm 기준으로 카메라는 libcamera / rpicam 계열 도구와 Picamera2 Python 라이브러리를 기준으로 사용한다.

기존 구형 picamera 라이브러리는 사용하지 않는다.

3. Day3 목표

Day3에서는 카메라 영상을 구조자 노트북에 띄워 실시간 조종에 사용한다.

초기 목표는 고화질이 아니라 저지연이다.

초기 설정:

resolution: 640x480
fps: 10~15
encoding: H.264 또는 MJPEG
transport: UDP 또는 TCP
4. 네트워크 구조
Head Pi + Camera Module 3
  |
BATMAN Mesh over node1/node2 relay
  |
Base Pi
  |
구조자 노트북
5. 테스트 순서
Step 1. Head Pi에서 카메라 인식 확인
rpicam-hello --list-cameras
Step 2. Head Pi에서 짧은 영상 테스트
rpicam-vid -t 5000 -o test.h264
Step 3. 저해상도 실시간 송출 테스트

Day3에서 GStreamer 또는 Python 기반 스트리밍 코드로 진행한다.

6. 주의사항
Camera Module 3는 CSI 케이블 방향을 잘못 꽂으면 인식되지 않는다.
Pi 4에 연결할 때는 카메라 포트 잠금 플라스틱을 확실히 고정한다.
자동초점 때문에 처음 프레임이 약간 흔들릴 수 있다.
실시간 조종 목적에서는 고화질보다 지연 시간이 중요하다.
Mesh 홉 수가 늘어나면 영상 품질을 낮춰야 한다.

7. 조난자 핸드폰과의 관계

조난자 핸드폰 접속은 Head Pi가 별도 Wi-Fi AP를 열 수 있을 때 추가한다.

현재 동글 없는 구성에서는 Head의 `wlan0`를 BATMAN Mesh에 사용하므로, 핸드폰 AP와 Mesh를 동시에 제공하지 않는다. 따라서 먼저 Head 카메라 영상과 조종 명령을 BATMAN Mesh 위에서 안정화한다.
