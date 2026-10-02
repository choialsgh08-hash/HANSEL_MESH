# HANSEL Final

구동, 카메라, 레이더, 오도메트리를 PC 터미널 하나에서 시작하는 버전이다.
키보드 제어는 기존대로 BASE에서 실행하고, Head에서 구동 노드와 센서를 실행한다.
PC에는 통합 지도와 카메라 창이 열린다. `--with-nodes`를 붙이면 node1, node2도 함께 구동한다.

처음에는 아래 순서대로 Nano 업로드와 각 Pi의 빌드까지 진행한다. 그다음부터는
장비를 연결하고 [9번 통합 실행](#9-통합-실행)만 하면 된다.
`source`는 빌드 명령이 아니라 현재 터미널에 ROS 환경을 불러오는 명령이다.
통합 실행기가 각 프로세스의 환경을 불러오므로 매번 SSH 창을 열어 따로 입력할 필요는 없다.

## 1. 폴더와 장비 구성

```text
HANSEL_Final/
  HANSEL_MESH_ROS2/    구동, 분리, 오도메트리, 지도, PC 통합 실행기
  HANSEL_MESH/         레이더, 카메라 송신, 센서 처리
```

이 브랜치에서는 위 두 폴더를 사용한다. 저장소 루트와 `260926` 폴더의 코드는 이전 기록이다.
레이더 소스도 일반 파일로 포함했으므로 별도 submodule이나 Git bundle을 복원하지 않는다.

| 장비 | 기본 주소 | 실행되는 것 |
| --- | --- | --- |
| PC | 192.168.60.2 | 통합 실행기, SSH 연결, 지도 브라우저, ffplay |
| BASE | 192.168.60.1 | operator stack, 키보드, 명령 라우팅 |
| Head | 192.168.50.10 | Nano 구동, 레이더, IMU, 오도메트리, 지도 서버, 카메라 송신 |
| node1 | 192.168.50.11 | Pi GPIO 구동, node2 분리용 마이크로 서보 |
| node2 | 192.168.50.12 | Pi GPIO 구동 |

이 문서는 기존 HANSEL 장비를 업데이트하는 순서다. 빈 SD 카드에 OS부터 설치하는 절차는 아니다.
Pi의 BATMAN/AP, SSH, 시간 동기화와 소스 빌드한 ROS 2 Lyrical은 이미 구성되어 있어야 한다.

- PC: Python 3.8 이상, Git, OpenSSH, 브라우저, ffplay, Nano 업로드용 Arduino IDE.
- BASE/Head/node1/node2: `~/ros2_lyrical/install/local_setup.bash`와 ROS 빌드 도구.
- Head: 기존 레이더 Python 의존성, `rpicam-vid` 또는 `libcamera-vid`, `ffmpeg`.
- node1/node2: 기존 GPIO 배선과 pigpiod. 후방 노드는 Nano 방식으로 바꾸지 않는다.

PC에는 ROS를 설치하지 않아도 된다. Ubuntu PC에 ffplay가 없으면 `sudo apt install ffmpeg`로 설치한다.

## 2. 기존 실행 종료와 코드 다운로드

업데이트 중에는 로봇을 정지해 둔다. 기존 키보드 창에서 X로 정지한 뒤 P로 종료하고,
별도로 띄운 구동·레이더·오도메트리·카메라 실행 창도 종료한다.
예전에 열어 둔 `ssh -N` 지도 터널과 ffplay도 닫는다. BATMAN/AP와 SSH는 그대로 둔다.

Head에서 남아 있는 프로세스를 확인할 때는 다음을 사용한다.

```bash
pgrep -af '[u]nit_controller|[h]ead_odometry|[l]ocal_inputs|[r]un_head_map|[r]un_radar_stack|[r]adar_front|[s]tart_camera_stream|[r]picam-vid|[f]fmpeg'
systemctl is-active hansel-camera.service hansel-radar-head.service
```

카메라나 레이더 서비스가 `active`라면 해당 서비스만 중지한다. 서비스가 없는 장비에서는 이 단계는 건너뛴다.
기존 수동 실행과 통합 실행을 동시에 띄우면 같은 장치나 포트를 중복 사용하게 된다.

**PC 터미널**에서 내려받는다.

```bash
mkdir -p ~/Projects
cd ~/Projects
git clone --branch HANSEL_Final --single-branch https://github.com/choialsgh08-hash/HANSEL_MESH.git HANSEL_Final_checkout
cd ~/Projects/HANSEL_Final_checkout/HANSEL_Final
```

이미 같은 이름의 폴더가 있으면 덮어쓰지 말고 다른 새 폴더에 clone한다.
아래 명령의 PC 경로는 `HANSEL_Final_checkout`을 기준으로 적었다.

## 3. Head Arduino Nano 업로드

업로드할 스케치는 다음 파일이다.

```text
HANSEL_MESH_ROS2/robot/firmware/hansel_nano_bridge/hansel_nano_bridge.ino
```

같은 폴더의 `servo_motion.h`도 필요하다. `.ino` 하나만 따로 옮기지 않는다.
**Head의 Nano에만 업로드한다.** node1/node2는 Pi GPIO 구동이므로 이 스케치를 올리지 않는다.

1. 구동 노드를 종료하고 모터·서보 전원을 끈다. Head Nano의 USB를 PC에 연결한다.
2. Arduino IDE에서 위 `.ino`를 연다. `servo_motion.h`가 같은 스케치 폴더에 있는지 확인한다.
3. 보드는 기존에 사용한 classic Arduino Nano / ATmega328P를 선택한다.
4. 프로세서는 이전에 업로드가 성공했던 설정을 쓴다. 구형 부트로더 보드라면 `ATmega328P (Old Bootloader)`다. CH340 USB라는 이유만으로 부트로더를 단정하지 않는다.
5. PC에서 실제로 연결된 Nano 포트를 선택한다. Head의 `/dev/ttyUSB1` 번호를 그대로 입력하는 것이 아니다.
6. 먼저 컴파일하고, 성공하면 업로드한다. `Servo.h`가 없다는 오류라면 Arduino Servo 라이브러리를 설치한다. 이 작업은 스케치 업로드이며 부트로더 굽기가 아니다.
7. 업로드 완료를 확인한 뒤 시리얼 모니터를 닫고 USB를 Head에 다시 연결한다.

보드 설정은 [Arduino Nano 프로세서 선택 안내](https://support.arduino.cc/hc/en-us/articles/4401874304274-Select-the-right-processor-for-Arduino-Nano)를 참고하면 된다.

새 펌웨어는 누적 엔코더 카운트와 부팅 후 시간을 함께 보낸다. 115200 baud의 출력 형식은 아래와 같다.
숫자는 예시이고, `B,boot`는 부팅 시에만 나온다. 출력을 보기 위해 모터를 돌릴 필요는 없다.

```text
B,boot
T,0.000,0.000,0.000,0,0,0,2500
```

`T` 뒤에는 좌/우 CPS, 서보 명령 각도, 서보 이동 상태, 좌/우 누적 카운트, uptime(ms)가 온다.
구형 펌웨어를 그대로 쓰면 기존 PWM 제한이 남고, 누적 카운트 오도메트리와 분리 완료 응답도 동작하지 않는다.

## 4. PC에서 Pi로 파일 전송

이번 변경에는 `WheelState.msg` 변경이 포함된다. Head만 바꾸지 말고 BASE와 사용하는 후방 노드까지
동일한 소스를 전송하고 빌드한다. 빌드 결과물은 장비마다 다르므로 PC의 `build/install/log`는 보내지 않는다.

**PC**에서 압축한다.

```bash
cd ~/Projects/HANSEL_Final_checkout/HANSEL_Final
tar --exclude='*/__pycache__' --exclude='*/.pytest_cache' --exclude='*/runtime' --exclude='*/ros2_ws/build' --exclude='*/ros2_ws/install' --exclude='*/ros2_ws/log' -czf /tmp/HANSEL_Final_ros.tar.gz HANSEL_MESH_ROS2
tar --exclude='*/__pycache__' --exclude='*/.pytest_cache' --exclude='*/runtime' -czf /tmp/HANSEL_Final_radar.tar.gz HANSEL_MESH

scp /tmp/HANSEL_Final_ros.tar.gz hansel@192.168.60.1:/tmp/
scp -o ProxyJump=hansel@192.168.60.1 /tmp/HANSEL_Final_ros.tar.gz /tmp/HANSEL_Final_radar.tar.gz hansel@192.168.50.10:/tmp/
```

node1/node2도 사용할 때는 이어서 전송한다.

```bash
scp -o ProxyJump=hansel@192.168.60.1 /tmp/HANSEL_Final_ros.tar.gz hansel@192.168.50.11:/tmp/
scp -o ProxyJump=hansel@192.168.60.1 /tmp/HANSEL_Final_ros.tar.gz hansel@192.168.50.12:/tmp/
```

비밀번호는 SSH가 요구할 때 입력한다. 설정 파일에 저장하지 않는다.
이후 Pi를 재부팅해서 `/tmp`의 압축 파일이 사라졌다면 다시 전송한다.

## 5. BASE 적용과 빌드

**PC**에서 BASE로 접속한다.

```bash
ssh hansel@192.168.60.1
```

**BASE**에서 실행한다. 프롬프트 또는 `hostname`이 `base`인지 먼저 확인한다.

```bash
hostname
cd ~
if [ -d HANSEL_MESH_ROS2 ]; then
  tar --exclude='*/ros2_ws/build' --exclude='*/ros2_ws/install' --exclude='*/ros2_ws/log' --exclude='*/runtime' -czf "hansel-base-backup-$(date +%Y%m%d-%H%M%S).tar.gz" HANSEL_MESH_ROS2
fi
tar -xzf /tmp/HANSEL_Final_ros.tar.gz -C "$HOME"
cd ~/HANSEL_MESH_ROS2
bash scripts/build_ros2.sh
```

빌드 마지막의 `[OK] HANSEL ROS 2 workspace build and overlay validation passed.`를 확인한다.
에러가 있으면 다음 장비 실행으로 넘어가지 말고 해당 빌드를 먼저 마친다.
지금은 `run_operator_stack.sh`나 키보드를 따로 실행하지 않는다. 통합 실행기가 시작한다.

작업을 마치면 `exit`로 PC에 돌아온다.

## 6. Head 적용과 빌드

**PC**에서 접속한다.

```bash
ssh -J hansel@192.168.60.1 hansel@192.168.50.10
```

**Head**에서 기존 소스와 보정 파일을 백업한 뒤 두 압축 파일을 푼다.

```bash
hostname
cd ~
if [ -d HANSEL_MESH_ROS2 ]; then
  tar --exclude='*/ros2_ws/build' --exclude='*/ros2_ws/install' --exclude='*/ros2_ws/log' --exclude='*/runtime' -czf "hansel-head-ros-backup-$(date +%Y%m%d-%H%M%S).tar.gz" HANSEL_MESH_ROS2
fi
if [ -d HANSEL_MESH ]; then
  tar --exclude='*/runtime' --exclude='*/__pycache__' --exclude='*/.git' -czf "hansel-head-sensors-backup-$(date +%Y%m%d-%H%M%S).tar.gz" HANSEL_MESH
fi
tar -xzf /tmp/HANSEL_Final_ros.tar.gz -C "$HOME"
tar -xzf /tmp/HANSEL_Final_radar.tar.gz -C "$HOME"
cd ~/HANSEL_MESH_ROS2
bash scripts/build_ros2.sh
```

Head에서 나중에 수정한 서보 한계나 보정 파일이 따로 있다면 백업본과 비교해 반영한다.
새 소스의 기본값이 현장에서 다시 측정한 값보다 최신이라고 가정하지 않는다.

장치 연결과 카메라 실행 파일도 확인한다.

```bash
ls -l /dev/serial0 /dev/serial/by-id/
command -v rpicam-vid libcamera-vid ffmpeg
```

카메라 명령은 `rpicam-vid`와 `libcamera-vid` 중 하나가 있으면 되고, RTP 송신에는 `ffmpeg`도 필요하다.
Nano와 레이더는 by-id 경로를 사용한다. USB를 다시 꽂아 `ttyUSB0/1`이나 `ttyACM0/1` 번호가 바뀌어도
같은 장치를 찾도록 한 것이다. 다른 보드로 교체해서 ID 자체가 바뀌면 8번 설정을 바꿔야 한다.
IMU는 BNO085 UART-RVC이며 `/dev/serial0`을 사용한다.

확인을 마치면 `exit`로 PC에 돌아온다.

## 7. node1과 node2 적용

Head만 구동할 때는 이 단계를 건너뛴다. 후방 노드를 함께 쓸 때는 **두 장비 모두** 업데이트한다.

PC에서 node1에 접속한다.

```bash
ssh -J hansel@192.168.60.1 hansel@192.168.50.11
```

**node1**에서 실행한다.

```bash
hostname
cd ~
if [ -d HANSEL_MESH_ROS2 ]; then
  tar --exclude='*/ros2_ws/build' --exclude='*/ros2_ws/install' --exclude='*/ros2_ws/log' --exclude='*/runtime' -czf "hansel-node1-backup-$(date +%Y%m%d-%H%M%S).tar.gz" HANSEL_MESH_ROS2
fi
tar -xzf /tmp/HANSEL_Final_ros.tar.gz -C "$HOME"
cd ~/HANSEL_MESH_ROS2
bash scripts/build_ros2.sh
bash robot/run_node.sh node1 --check
```

`exit`로 PC에 돌아온 뒤 node2로 접속한다.

```bash
ssh -J hansel@192.168.60.1 hansel@192.168.50.12
```

**node2**에서 실행한다.

```bash
hostname
cd ~
if [ -d HANSEL_MESH_ROS2 ]; then
  tar --exclude='*/ros2_ws/build' --exclude='*/ros2_ws/install' --exclude='*/ros2_ws/log' --exclude='*/runtime' -czf "hansel-node2-backup-$(date +%Y%m%d-%H%M%S).tar.gz" HANSEL_MESH_ROS2
fi
tar -xzf /tmp/HANSEL_Final_ros.tar.gz -C "$HOME"
cd ~/HANSEL_MESH_ROS2
bash scripts/build_ros2.sh
bash robot/run_node.sh node2 --check
```

`--check`는 GPIO 연결 상태를 점검하는 용도다. pigpiod 연결 오류가 나면 해당 장비의 기존 pigpiod 서비스를 확인한다.
후방 노드의 BATMAN/AP 설정은 바꾸지 않는다. 끝나면 `exit`로 PC에 돌아온다.

인터페이스 빌드는 이번 변경을 받는 장비마다 한 번씩 필요하다. 이후 SSH를 다시 연결하거나 통합 실행기를
재시작할 때마다 빌드할 필요는 없다. 메시지나 코드가 다시 바뀌면 영향을 받는 장비를 다시 빌드한다.

## 8. 실행 전 주소와 시간 확인

PC의 [configs/head_session.json](HANSEL_MESH_ROS2/configs/head_session.json)을 확인한다.
IP와 계정, Pi의 코드 경로, Nano/레이더 by-id가 실제 장비와 맞아야 한다.

```bash
ip -brief addr
```

PC 영상 수신 주소가 `192.168.60.2`가 아니면 JSON의 `pc_ip`를 실제 주소로 수정한다.
카메라 UDP는 Head에서 PC로 직접 전달된다. 지도용 SSH 터널이 카메라 RTP까지 운반하는 것은 아니다.

기존 팀원 가이드의 시간 동기화도 유지한다. BASE를 제외한 장비에서 기존 Makefile이 있는 위치의
`make timereset`, `make timecheck`를 사용하거나 `chronyc sources -v`로 선택된 시간 소스의 `^*`를 확인한다.
이 배포본에는 해당 시간 관리 Makefile을 새로 넣지 않았다. 프로젝트 어디에서나 `make timereset`이 된다는 뜻은 아니다.
시간 변경은 주행 전에 끝낸다. 통합 실행기는 시각 차이를 경고만 하고 실행 중 시간을 강제로 바꾸지 않는다.

전체 실행은 새 주행 세션으로 시작한다. `--with-nodes`를 사용할 때는 Head-node1-node2가 모두 물리적으로
연결된 상태여야 한다. 지난 세션에서 떨어뜨린 상태를 다음 세션까지 유지하지 않는다.

## 9. 통합 실행

**PC**에서 다음 위치로 이동한다.

```bash
cd ~/Projects/HANSEL_Final_checkout/HANSEL_Final/HANSEL_MESH_ROS2
```

Head, node1, node2를 모두 쓸 때:

```bash
python3 controller/run_head_session.py --with-nodes
```

Head만 쓸 때는 위 명령 대신 다음을 실행한다.

```bash
python3 controller/run_head_session.py
```

두 명령 모두 **구동 + 카메라 송수신 + 레이더 + 오도메트리 + 통합 지도**를 실행한다.
두 명령을 동시에 실행하는 것이 아니다. 실행 전 명령 구성만 보려면 같은 명령 끝에 `--dry-run`을 붙인다.

SSH 인증을 마치면 각 장비에서 프로세스가 시작되고 잠시 후 지도와 카메라 창이 열린다.
카메라는 Head가 이번 실행에서 새로 만든 SDP를 PC로 가져와 ffplay로 연다.
이전 `/tmp/hansel-camera.sdp`를 재사용하거나 ffplay를 따로 실행할 필요가 없다.

- 통합 지도: `http://127.0.0.1:8083/`
- 지도 왼쪽: 레이더 경로. 오른쪽: 누적 오도메트리와 분리 위치.
- 지도 방향: +X가 위쪽.
- 카메라: 별도 ffplay 창.
- 조종: 실행한 PC 터미널. 키보드 프로세스 자체는 BASE에서 동작한다.
- 로그: `HANSEL_MESH_ROS2/runtime/sessions/<실행시각-PID>/`

레이더 초기 설정은 다른 프로세스보다 늦게 끝날 수 있다. 프로세스의 `running` 표시는 데이터 수신 성공과는
다르므로 지도에 최신 데이터가 들어오는지도 확인한다. 레이더나 IMU 오류만으로 구동을 잠그지는 않는다.

## 10. 첫 실행 확인과 원점 설정

처음 업데이트한 뒤에는 정지 상태에서 아래를 확인한다. 진단용 SSH 창은 필요할 때만 열면 된다.

**Head의 별도 SSH 창**, 구동 상태 확인:

```bash
cd ~/HANSEL_MESH_ROS2
source scripts/ros_env.sh
timeout 10s ros2 param get /hansel/head/unit_controller maximum_pwm_percent
timeout 10s ros2 topic echo /hansel/head/state/wheels --once
```

PWM 상한은 `75.0`이어야 한다. 정지 중 CPS와 PWM이 0인 것은 정상이다.
새 펌웨어에서는 누적 카운트가 유효하게 표시되어야 한다. 회전시켰을 때 실제 누적 카운트가 바뀌는지는
첫 짧은 주행에서 확인한다. 목표 CPS나 목표 RPM이 바뀌는 것만으로 엔코더 수신을 판단하지 않는다.

**같은 Head 진단 창**, 로컬 오도메트리 확인:

```bash
source scripts/ros_local_env.sh
timeout 10s ros2 topic echo /hansel/head/odometry --once --field pose.pose
```

구동은 mesh domain 0, 오도메트리와 지도는 Head 내부 domain 42를 쓴다.
IMU·누적 경로를 BATMAN에 ROS 토픽으로 계속 보내지 않고, Head 내부에서 처리한 뒤 PC에는 HTTP로 전달한다.
domain 0의 `ros2 node list`에서 오도메트리가 안 보이는 것은 이 구성에서는 정상이다.
오도메트리를 조회하는 창에서는 `ros_local_env.sh` 이후 `ros_env.sh`를 다시 불러오지 않는다.

출발 위치에 내려놓고 완전히 정지한 뒤 원점을 잡는다.

```bash
timeout 10s ros2 service call /hansel/head/localization/zero std_srvs/srv/Trigger '{}'
```

성공 응답을 확인하고 약 2초 기다린 뒤 위치를 다시 조회한다. 초기 heading을 모으는 동안은 움직이지 않는다.
원점 재설정은 현재 경로와 분리 마커도 지운다. 이전 기록을 남겨야 한다면 먼저 보관한다.

거리 계산은 **뒤쪽 좌우 엔코더의 실제 누적 카운트 차이**를 사용한다. CPS는 PID와 표시용으로 남아 있다.
현재 거리 보정값 `0.07073229`는 대략 1 m 주행에서 얻은 임시값이다. 카운트 방식으로 바꾼 뒤의 정확도까지
입증한 값은 아니므로 처음에는 유지해서 비교하고, 이후 실측 거리로 다시 보정한다.

## 11. 조종과 유닛 분리

| 키 | 동작 |
| --- | --- |
| W / S | 전진 / 후진. 기존 속도 목표와 PID 사용 |
| Q | 왼쪽 바퀴만 전진, 최대 PWM 적용 |
| E | 오른쪽 바퀴만 전진, 최대 PWM 적용 |
| A / D | 양쪽 바퀴를 반대로 돌려 좌/우 제자리 회전 |
| U / J | 뿔 서보 목표 각도 +5 / -5도. 목표까지 부드럽게 이동 |
| K | 설정된 기본 각도로 복귀 |
| F / V | 앞 모터만 전진 / 정지 |
| X / Space | 정지 |
| 2 | node1의 마이크로 서보로 node2 분리 |
| 1 | Head의 마이크로 서보로 node1 분리 |
| P | 키보드 종료와 통합 세션 종료 |

Q/E는 회전 방향이 아니라 **구동할 바퀴 이름** 기준이다. Q가 왼쪽으로 꺾인다는 뜻은 아니다.
최대 PWM은 세 유닛 모두 75%다. A/D/Q/E도 기존 출력 상승 제한을 거쳐 약 0.75초에 75%에 도달한다.
W/S 기본값은 `speed=0.75`다. 속도 목표 비율이며, PWM을 항상 75%로 고정한다는 뜻은 아니다.
A/D/Q/E는 Head만 조향하고 node1·node2에는 정지를 전달한다. W/S는 분리되지 않은 구동 유닛에 함께 전달한다.
F의 앞 모터 단독 모드는 V/X 또는 일반 주행 키를 누르면 끝난다.

명령이 잠시 끊기면 모터는 정지하고, 새 명령을 받으면 다시 진행한다. 통신 끊김마다 G를 누르는 방식은 없앴다.
다만 수동으로 구동을 껐거나 기존 비상정지 상태를 걸어 둔 경우, 이미 분리된 유닛인 경우까지 자동 해제하지는 않는다.

유닛을 하나씩 남길 때는 **2 → 1** 순서다. node2를 분리하면 node2는 구동 대상에서 빠지고,
다음에 node1을 분리하면 node1도 빠진다. 분리된 유닛의 BATMAN/AP는 그대로 유지한다.
1을 먼저 누르면 node1에 붙어 있는 node2까지 Head에서 떨어지므로 두 유닛 모두 구동을 중지한다.
분리된 유닛은 G를 눌러도 다시 구동되지 않는다.

분리 완료 이벤트는 해당 시점의 위치를 지도에 남긴다. 마커는 Head 궤적과 유닛 간격 0.20/0.40 m로 계산한
추정 위치이며, 연결부가 실제로 열렸는지를 감지하는 센서는 없다. 서보 명령 완료와 기계적 분리는 구분해야 한다.
완료 응답이 없으면 성공 마커를 만들지 않고, 이미 떨어졌을 수 있는 후방 유닛을 정지 상태로 유지한다.
최초 사용 시 정지 상태에서 서보 방향과 걸쇠 이동을 확인한다.

## 12. 종료와 다음 실행

조종 터미널에서 **X로 정지한 다음 P**를 누른다. 실행기가 자신이 띄운 구동·센서·카메라·지도 프로세스와
SSH 터널, ffplay를 정리한다. BATMAN/AP와 기존 네트워크 서비스는 건드리지 않는다.

다음에는 장비와 전원을 연결하고 9번 명령만 실행하면 된다. 매번 다시 빌드하거나 Nano를 업로드하지 않는다.
분리 시연 뒤 전체 실행을 다시 시작할 때는 유닛을 다시 연결한다.

## 문제가 생겼을 때

먼저 PC의 해당 실행 로그를 본다. 같은 프로그램을 여러 창에서 다시 띄우지 않는다.

| 증상 | 확인할 것 |
| --- | --- |
| 구동 시작 실패 | `head-unit.log`, `base-operator.log`, 사용 중이면 `node1-unit.log` / `node2-unit.log`. 기존 실행, Nano by-id, pigpiod 확인 |
| 레이더 화면 없음 | `head-radar.log`. CONFIGURE 단계의 명령과 응답, by-id, 포트 점유 확인 |
| IMU 값 없음 | `head-odometry.log`. raw UART 0 bytes는 정상 수신이 아님. 전원·공통 GND·UART 배선·RVC 모드 확인 |
| 지도만 연결 안 됨 | `head-map.log`, PC 8083 포트를 쓰는 이전 SSH 터널 확인 |
| 카메라 창 없음 | `head-camera.log`, `ffplay.log`, PC의 ffplay 설치 확인 |
| 카메라 창은 있지만 영상 없음 | JSON의 `pc_ip`, Head→PC UDP 5600 경로·방화벽, 카메라 송신 로그 확인 |
| 오도메트리 zero 서비스 없음 | Head에서 `ros_local_env.sh`를 불렀는지, `head-odometry.log`에 오류가 있는지 확인 |

레이더는 이전 고속 스트리밍 상태를 확인하고 가능한 baud에서 정지 후 프로파일을 다시 적용하도록 수정했다.
IMU는 무수신과 잘못된 프레임을 나누어 기록하고 재연결한다. by-id는 USB 번호 변경 문제를 해결하지만,
빠진 선이나 전원 불량, 센서 자체 멈춤까지 해결하는 기능은 아니다.

## 확인 범위

로컬 자동 테스트, 구문 검사, Nano 컴파일·링크와 합성 데이터 지도 화면 확인을 진행했다.
Pi의 실제 빌드, Nano 업로드, 실물 서보·분리, 레이더 재연결, 세 유닛 주행과 메쉬 대역폭은 현장에서 확인해야 한다.
테스트 통과를 실물 검증 완료나 위치 정확도 보증으로 취급하지 않는다.

구현과 변경 근거는 [통합 실행 구조](HANSEL_MESH_ROS2/docs/HEAD_SESSION.md),
[구동·서보·분리](HANSEL_MESH_ROS2/docs/DRIVE_DETACH.md)에 정리했다.
