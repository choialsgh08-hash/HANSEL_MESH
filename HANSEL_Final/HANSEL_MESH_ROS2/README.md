# HANSEL ROS 2

구동, 분리 제어, 실제 엔코더 카운트 오도메트리와 통합 실행기 코드다.
현재 장비는 소스 빌드한 ROS 2 Lyrical을 사용한다.

**[설치와 전체 실행 순서](../README.md)**

설치와 Nano 업로드를 마쳤다면 PC에서 다음 명령으로 구동·카메라·레이더·오도메트리를
함께 시작한다. Head만 사용할 때는 `--with-nodes`를 뺀다.

```bash
python3 controller/run_head_session.py --with-nodes
```

- BASE: `controller/run_operator_stack.sh`, `controller/run_keyboard_control.sh`
- Head: `robot/run_unit.sh`와 로컬 센서/지도 프로세스
- node1/node2: `robot/run_node.sh`, Pi GPIO/pigpiod 방식
- Head Nano: `robot/firmware/hansel_nano_bridge/`의 `.ino`와 `servo_motion.h`

펌웨어는 Head Nano에만 올린다. 후방 노드에 같은 Nano 스케치를 올리지 않는다.
오도메트리·지도 점검은 Head에서 `scripts/ros_local_env.sh`를 사용한다.
구동 점검은 `scripts/ros_env.sh`를 사용한다.

기술 문서: [통합 실행](docs/HEAD_SESSION.md), [구동과 분리](docs/DRIVE_DETACH.md).
