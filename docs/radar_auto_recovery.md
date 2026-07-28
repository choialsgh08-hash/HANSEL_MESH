# IWRL6432BOOST 자동 복구 운용 절차 (Windows)

이 문서는 IWRL6432BOOST 캡처, 상태 감시, 자동 복구, 조종 화면을 한 프로세스가
관리하는 정상 Windows 운용 절차다. 저장소 루트에서 명령을 실행하고, 조종 화면은
`http://127.0.0.1:8081/`에서 연다.

> **주행 안전 규칙**
>
> supervisor가 `RUNNING`이고 화면 입력도 정상일 때만 센서 데이터가 주행 판단에
> 사용 가능하다. `WAIT_PORT`를 포함한 모든 non-running 상태와 화면의
> `SENSOR_FAULT`는 movement-blocking이다. 이 프로그램은 모터 정지 명령을 직접
> 보내지 않으므로, 로봇 제어 계층이 이 조건에서 반드시 구동을 차단해야 한다.

## 1. 사전 준비

### Python과 pyserial

현재 검증 환경은 Python 3.12와 pyserial 3.5다. 프로젝트 요구 범위는
`pyserial>=3.5,<4`이며 다음 순서로 확인한다.

```powershell
python --version
python -m pip install -r requirements-sensors.txt
python -c "import serial; print(serial.__version__)"
```

### TI UniFlash와 XDS110 reset 도구

자동 복구는 가능하면 TI `xds110reset.exe`로 선택한 XDS110 보드만 reset한다.
실행기는 먼저 `PATH`를 확인하고, 없으면 `C:\ti\uniflash_*` 아래의 지원 경로에서
가장 최신 UniFlash 설치를 찾는다. 현재 벤치 PC의 대표 경로는 다음과 같이 확인할
수 있다.

```powershell
$radarReset = 'C:\ti\uniflash_9.6.0\deskdb\content\TICloudAgent\win\ccs_base\common\uscif\xds110\xds110reset.exe'
Test-Path -LiteralPath $radarReset -PathType Leaf
```

결과가 `True`여야 한다. 자동 탐색 범위 밖에 설치했다면 이처럼 확인한 정확한
파일만 `--reset-executable "$radarReset"`로 지정한다. 존재하지 않는 명시 경로는
사용할 수 없다. 실제 지원 옵션과 기본값은 언제든 다음 명령으로 확인한다.

```powershell
python scripts\run_radar_stack.py --help
```

## 2. 영구 캘리브레이션 확인

정상 운용에는 다음 영구 클러터 캘리브레이션이 반드시 있어야 한다.

```powershell
Test-Path -LiteralPath `
  configs\radar\calibrations\head-near.json `
  -PathType Leaf
```

`False`이면 여기서 멈춘다. 임시 파일이나 캘리브레이션 없는 화면으로 실제 로봇을
운용하지 않는다. [레이더 화면 가이드의 빈 장면 캘리브레이션](radar_front_view.md#3-빈-장면-캘리브레이션)에
있는 통제된 빈 장면 `radar-live` 캡처와
`python -m sensors radar-calibrate` 절차를 먼저 수행한다.

## 3. 정상 Windows 원클릭 실행

보드를 USB로 연결한 뒤 저장소 루트의 PowerShell에서 다음 명령을 그대로 실행한다.
정상 명령은 COM 번호를 고정하지 않는다.

```powershell
python scripts\run_radar_stack.py `
  --xds-serial RI32 `
  --cfg configs\radar\iwrl6432_3d_operator_near_10hz.cfg `
  --clutter-calibration `
    configs\radar\calibrations\head-near.json
```

supervisor가 `RUNNING`이 되면 브라우저에서
`http://127.0.0.1:8081/`을 연다. 실행 중인 PowerShell은 닫지 않는다.

### 현재 production 기본값

| 항목 | 기본값 |
| --- | --- |
| active profile | `configs\radar\iwrl6432_3d_operator_near_10hz.cfg` |
| profile ID | `lsdk-05.05.04.02-presence-near-heatmap16-elev8-cfar15-10hz-v1` |
| 첫 프레임 제한 | `--first-frame-timeout 3.0`초 |
| RUNNING 프레임 제한 | `--frame-timeout 2.5`초 |
| 검증 제한 | `--verification-timeout 3.0`초 |
| 연속 검증 프레임 | `--verify-frames 5` |
| 재시도 backoff | `--retry-initial 0.5`초, 최대 `--retry-max 5.0`초 |
| HTTP | `--http-port 8081`, bind `127.0.0.1` |
| 출력 root | `--output-root` 생략 시 저장소 루트 |
| run ID | `--run-id` 생략 시 UTC `YYYYMMDD-HHMMSS` |

backoff는 실패할 때마다 두 배로 늘어나 최대 5초가 되고, 다시 `RUNNING`에
도달하면 0.5초로 초기화된다. `--clutter-calibration`은 기본 파일을 암묵적으로
선택하지 않으며, 존재하는 JSON 경로를 반드시 명시해야 한다.

## 4. 보드와 COM 식별

supervisor는 다음 조건을 모두 만족하는 포트 하나만 선택한다.

- TI XDS110 VID:PID `0451:BEF3`
- description에 `Application/User UART` 포함
- XDS serial `RI32`
- description에 `Auxiliary`가 없음

`Auxiliary Data Port`는 레이더 CLI/data 포트가 아니므로 절대 선택하지 않는다.
정상 명령에는 `--port`가 없기 때문에 reset 또는 USB 재연결 뒤 `COM3`이 다른
번호로 바뀌어도 같은 XDS serial의 Application/User UART를 자동으로 다시 찾는다.
`--port COM3`처럼 번호를 명시하면 해당 번호로 고정되므로 진단할 때만 사용한다.
조건을 만족하는 포트가 없거나 둘 이상이라 하나를 확정할 수 없으면
`WAIT_PORT`에서 주행을 차단한 채 기다린다.

## 5. supervisor 상태

| 상태 | 의미와 운용 |
| --- | --- |
| `WAIT_PORT` | 선택한 Application/User UART가 정확히 하나가 될 때까지 기다린다. reset 후 COM 재열거도 여기서 처리한다. |
| `RESET_TARGET` | 발견한 reset 도구로 XDS serial이 일치하는 보드만 reset한다. 실패하면 backoff 후 다시 시도한다. |
| `CONFIGURE` | 115200 baud에서 profile을 적용하고 1250000 baud 전환 결과를 검증한다. |
| `START_CAPTURE` | 새 epoch의 mission, raw, raw index 경로를 할당하고 `radar-live`를 시작한다. |
| `VERIFY_FRAMES` | watchdog가 요구하는 연속 5개 정상 프레임을 기다린다. 검증 전 데이터는 주행 가능 상태가 아니다. |
| `SWITCH_VIEWER` | 검증된 새 epoch를 따라가도록 viewer를 교체한다. |
| `RUNNING` | 캡처와 viewer가 살아 있고 프레임 watchdog가 정상인 유일한 운용 상태다. viewer만 종료되면 같은 epoch에서 다시 시작한다. |
| `RECOVERING` | UART 손실, 캡처 종료, stale/assert 등 fault 뒤 캡처를 닫고 다음 복구 시도를 준비한다. 주행은 차단한다. |
| `STOPPED` | `Ctrl+C`, `SIGTERM`, 치명 오류 또는 종료 정리가 끝난 상태다. 주행은 차단한다. |

화면이 아직 첫 프레임을 기다리거나 재연결 중이거나 입력이 stale/fault인 경우에도
이전 프레임을 안전 정보로 재사용하지 않는다. 이때 충돌 상태와 장면 hazard는
`SENSOR_FAULT`이며, 거리 안내와 구역 정보는 무효다.

## 6. 자동 복구와 USB cycle gate

`RUNNING` 중 Application/User UART가 사라지거나 캡처가 종료되거나 watchdog가
프레임 fault를 검출하면 supervisor는 다음 순서로 복구한다.

1. `recovery_count`를 1 증가시키고 `RECOVERING`으로 전환한다.
2. 해당 capture를 종료하고 현재 epoch의 종료 시각과 이유를 manifest에 기록한다.
3. 같은 XDS serial의 Application/User UART를 다시 찾는다.
4. 가능하면 선택한 보드만 reset하고 profile을 다시 적용한다.
5. 기존 파일을 재사용하지 않고 다음 epoch를 만든다.
6. 연속 5개 정상 프레임을 검증한 뒤 viewer를 바꾸고 `RUNNING`으로 복귀한다.

`recovery_count`는 `RUNNING`에서 감지한 runtime fault 횟수다. 시작 단계의 profile,
capture 시작 또는 초기 프레임 검증 재시도 횟수와 동일하지 않다.

reset 실행 파일을 찾지 못해도 최초 시작에서 사용 가능한 포트의 profile 적용은
시도한다. 그러나 이미 연결된 채 발생한 fault를 복구할 때 reset 도구가 없고
포트 이탈을 아직 관측하지 않았다면, 상태 이유
`reset_tool_unavailable_waiting_for_usb_cycle`로 남아 실제 USB 분리 후 같은 보드의
재연결을 모두 확인할 때까지 재구성하지 않는다. 단순히 COM 이름이 존재하거나
동일 포트를 다시 읽는 것은 이 gate를 통과하지 못한다. UART 손실 자체를 이미
관측한 복구에서는 같은 보드의 포트가 다시 나타나는 것이 재연결 증거다.

## 7. 기록 파일과 immutable epoch

기본 output root는 저장소 루트이고, `<run-id>`는 기본적으로 시작 UTC 시각이다.
각 capture 시도는 증가하는 `<epoch>` (`e001`, `e002`, ...)를 사용한다.

```text
missions\radar-board-live-<run-id>-<epoch>.jsonl
captures\radar-board-live-<run-id>-<epoch>.bin
captures\radar-board-live-<run-id>-<epoch>.bin.chunks.jsonl
runtime\radar-board-live-<run-id>\<epoch>-capture.stdout.log
runtime\radar-board-live-<run-id>\<epoch>-capture.stderr.log
runtime\radar-board-live-<run-id>\<epoch>-viewer.stdout.log
runtime\radar-board-live-<run-id>\<epoch>-viewer.stderr.log
runtime\radar-supervisor-<run-id>.json
```

마지막 JSON은 현재 상태, `epoch`, `recovery_count`, 선택 포트와 XDS serial,
reset 가능 여부, 각 epoch의 시작·종료·종료 이유·capture exit code, owned child
시작/종료와 escalation을 담는 manifest다.

epoch 파일은 immutable이다. 경로 중 하나라도 이미 존재하면 덮어쓰지 않고
실패하며, 복구는 기존 mission/raw/index를 이어 쓰지 않고 새 epoch를 할당한다.
따라서 장애 전후 자료는 각각 보존된다.

## 8. 정상 종료

launcher PowerShell에서 `Ctrl+C`를 누르거나 프로세스에 `SIGTERM`을 보내면 새 복구를
시작하지 않고 owned capture와 viewer만 종료한 뒤 `STOPPED`를 기록한다. Windows
child에는 먼저 `CTRL_BREAK`를 보내고 각 child가 종료할 시간을 2초 준다. 응답하지
않으면 `terminate` 후 다시 2초, 마지막으로 `kill` 순서로 제한된 escalation을
수행한다. supervisor가 시작하지 않은 다른 프로세스에는 신호를 보내지 않는다.

capture가 graceful signal을 처리하면 raw index에 `capture_end` footer를 쓰고
raw/index를 flush 및 `fsync`한 뒤 mission log의 final health를 기록하고 파일을
닫는다. 강제 `terminate` 또는 `kill`까지 진행되면 이 graceful footer가 완성되지
않을 수 있으므로 manifest의 `process_events.escalation`과
`.bin.chunks.jsonl` footer를 함께 확인한다.

## 9. 벤치 acceptance 전용 임시 캘리브레이션

다음 파일은 현재 PC에서 짧은 벤치 acceptance에만 사용할 수 있는 fixture다.

```text
C:\Users\minho\AppData\Local\Temp\hansel-r9-fixture-calibration.json
```

이 파일은 production/default 캘리브레이션이 아니다. 로봇 장착 상태의 정상 운용,
실측 평가 또는 현장 주행에는
`configs\radar\calibrations\head-near.json`을 새로 생성해 사용한다.

## 10. Raspberry Pi 배포

`xds110reset.exe`는 Windows 실행 파일이므로 Raspberry Pi에서 동작하지 않는다.
Pi 배포 전 supervisor에 해당 환경에서 검증된 XDS reset 명령을 주입하거나,
보드 reset 선을 제어하는 GPIO 회로와 안전한 pulse/boot 대기 전략으로 대체해야
한다. 이 대체 reset 경로가 준비되지 않은 Pi는 자동 reset이 불가능하며, 복구 중
위 USB disconnect/reconnect gate가 적용된다.
