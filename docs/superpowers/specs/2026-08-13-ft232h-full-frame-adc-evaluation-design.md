# FT232H 전체 프레임 ADC 평가 설계

## 목적

IWRL6432BOOST의 현재 UART 기반 주행 보조 화면을 기준선으로 보존하면서,
FTDI FT232H SPI를 통해 전체 프레임 ADC를 노트북에서 수집하고 오프라인으로
재처리한다. 동일 장면에서 현재 UART 처리보다 통로 방향, 벽 위치, 경로 안정성이
정량적으로 향상된 경우에만 전체 ADC 결과를 실시간 주행 입력으로 승격한다.

이 설계의 1차 산출물은 새로운 실시간 주행 화면이 아니라 재현 가능한 전체 ADC
기록, 파서, 신호처리 결과 및 UART 대비 A/B 평가 보고서다. 1차 평가가 통과하면
노트북 실시간 처리, 그 다음에 Raspberry Pi 4 처리로 순서대로 확장한다.

## 범위와 비범위

### 포함

- C232HM-DDHSL-0을 이용한 Windows 노트북 직접 연결
- 현재 UART/XDS110 연결과 SPI 전체 ADC 수집의 동시 또는 동기화 기록
- 전 프레임 ADC의 길이, 배열 순서, 프레임 경계 및 손실 검증
- PC에서 Range FFT, BPM 복원, Doppler 처리, 안테나 보정 및 방위각 추정
- 0-30 cm 근거리 반사면과 통과 가능 공간 추정
- 현재 UART 방식과 전체 ADC 방식의 동일 장면 A/B 평가
- 합격 후 같은 FT232H 케이블을 Raspberry Pi 4 USB에 연결하는 이식 경로
- 메쉬망에는 전체 ADC 대신 압축 지도, 경로, 상태만 전송하는 구조

### 제외

- DCA1000 구매 또는 지원
- 1차 평가 전에 현재 UART 경로 생성기를 삭제하거나 교체하는 작업
- Raspberry Pi GPIO SPI와 레이더를 직접 연결하는 구성
- 전체 ADC를 BATMAN 메쉬망으로 상시 전송하는 구성
- IWRL6432BOOST의 안테나 aperture를 넘어서는 LiDAR급 윤곽 복원 주장
- FT232H 연결만으로 거리 및 각도 분해능이 향상된다는 주장

## 구매 품목

### 필수

1. **FTDI C232HM-DDHSL-0** 한 개
   - FT232H 기반 USB 2.0 Hi-Speed MPSSE 케이블
   - 3.3 V 디지털 I/O
   - USB-A 플러그와 2.54 mm 암 소켓 개별선 10개
   - TI가 xWRL6432 FCCSP SPI ADC 절차에서 직접 사용하는 모델
   - 제조사 제품 페이지: https://ftdichip.com/products/c232hm-ddhsl-0-2/

### 조건부

- 노트북에 USB-A 포트가 없으면 데이터 통신을 지원하는 USB-C to USB-A 어댑터
- 보드 J2가 미실장인 경우에만 2.54 mm 1x7 수 핀헤더와 납땜 작업
- 배선 고정용 열수축 튜브 또는 절연 캡

### 구매하지 않을 품목

- `C232HM-EDHSL-0`: I/O는 3.3 V지만 빨간 VCC 선이 5 V 출력인 변형이라
  현장 배선 혼동 가능성이 더 크다.
- UART 전용 FTDI 케이블: MPSSE SPI master가 아니므로 사용할 수 없다.
- CH341 계열 및 출처 불명 FT232H 보드: TI 도구와 BUSY GPIO 동기화의 재현성이
  검증되지 않았다.

공식 FTDI 판매가는 작성 시점 기준 약 USD 38.80이며, 국내 유통 가격과 재고는
변동한다. 부품 번호가 정확히 `C232HM-DDHSL-0`인지 확인하고 공인 유통망 또는
FTDI에서 구입한다.

## 노트북 직접 연결

노트북 연결이 기본 경로다. IWRL6432BOOST의 기존 마이크로 USB J5는 그대로
유지하여 보드 전원, XDS110 UART 설정 및 처리 결과를 담당한다. C232HM 케이블의
USB-A는 노트북의 두 번째 USB 포트에 연결하고 개별 신호선은 레이더 보드 J2 SPI
헤더에 연결한다. FTDI는 SPI master, IWRL6432는 SPI slave로 동작한다.

| IWRL6432BOOST J2 | 신호 | C232HM-DDHSL-0 |
|---:|---|---|
| 1 | VCC_3V3 | 연결하지 않음 |
| 2 | DCA_LP_SPI_MOSI | 노란색 |
| 3 | DCA_LP_SPI_MISO | 초록색 |
| 4 | DCA_LP_SPI_CS | 갈색 |
| 5 | DCA_LP_SPI_CLK | 주황색 |
| 6 | DCA_LP_HOST_INTR_1 / SPI BUSY | 회색 |
| 7 | GND | 검정색 |

J2의 사각 패드가 1번이다. C232HM의 빨간 VCC 선과 사용하지 않는 모든 선은 서로
닿지 않도록 개별 절연한다. 레이더는 기존 USB로 전원을 공급하므로 FTDI VCC로
레이더를 급전하지 않는다.

### 전원 및 연결 순서

1. 레이더와 FTDI USB를 모두 분리한다.
2. J2 핀 번호를 육안과 연속성 측정으로 확인한다.
3. MOSI, MISO, CS, CLK, BUSY, GND 여섯 선만 연결한다.
4. TI 절차에 따라 S1.1과 S1.6을 ON으로 둔다.
5. 기존 마이크로 USB로 레이더 보드 전원을 먼저 공급한다.
6. 보드가 켜진 뒤 C232HM USB를 노트북에 연결한다.
7. 종료할 때는 C232HM USB를 먼저 분리하고 레이더 전원을 끈다.

IWRL6432BOOST의 디지털 I/O는 전원 비인가 상태에서 외부 구동에 안전한 핀이
아니므로 이 순서를 유지한다.

## 비교한 접근 방식

### A. Windows 노트북 + C232HM 전체 ADC - 채택

TI가 제공하는 `adcDataSPIFTDI.exe`, Windows D2XX 드라이버 및 현재 설치된
MMWAVE-L-SDK를 그대로 사용한다. 하드웨어, 펌웨어, 파서와 알고리즘 문제를 가장
빨리 분리할 수 있고 현재 화면과 동시에 관찰할 수 있다.

### B. Raspberry Pi 4 + 같은 C232HM - 2차 채택

노트북에서 개선이 증명된 뒤 같은 케이블의 USB-A를 Pi 4에 연결한다. FTDI는
Linux ARM용 D2XX와 LibMPSSE-SPI를 제공하지만 TI SDK의
`adcDataSPIFTDI.exe`는 Windows 도구다. 따라서 Pi에서는 TI 도구의 프레임/BUSY
프로토콜을 재현하는 작은 native capture service를 구현하고, Python/NumPy 처리기는
그 서비스가 만든 binary frame을 소비한다.

### C. Raspberry Pi GPIO SPI 직접 연결 - 제외

전용 FTDI 없이 Pi SPI controller를 사용하는 방법은 가능성을 연구할 수 있으나,
28 MHz 전송, SPI BUSY 동기화, 프레임 경계 복구, Linux scheduling jitter 및 전기적
보호를 동시에 해결해야 한다. 1차 목표의 위험과 작업량을 불필요하게 키우므로 사용하지
않는다.

## 펌웨어와 프로파일

현재 UART 주행용 appimage와 SPI ADC 평가용 appimage를 별도 산출물로 관리한다.
SPI 평가용 demo에는 다음 조건을 적용한다.

- SysConfig의 `ADC STREAMING via SPI` 활성화
- `lowPowerCfg 0`
- `adcLogging 2`
- 현재 근거리 프로파일 기준 ADC sample 256, RX 3, chirp/burst 2,
  burst/frame 16, frame period 200 ms에서 시작
- TI 식 `RX x frame chirps x ADC samples x bytes/sample`에 맞춰
  `ADC_DATA_BUFF_MAX_SIZE`와 linker memory 검증
- 모든 ADC가 frame idle 안에 전송되는지 측정하고 부족하면 먼저 프레임 주기를
  늘린다. 샘플을 임의로 버려 전송 시간을 맞추지 않는다.
- 기존 UART 처리 결과도 가능한 한 함께 기록한다. 메모리 또는 idle budget 때문에
  동시 기록이 불가능하면 센서와 장면을 움직이지 않은 채 UART와 SPI를 연속 기록한다.

현재 설정에서 SPI에 노출되는 chirp가 `2 x 16 = 32`개라면 예상 binary payload는
프레임당 `3 x 32 x 256 x 2 = 49,152 bytes`다. 이 수치는 구현의 가정이 아니라
첫 캡처에서 실제 파일 길이와 대조해야 하는 검증값이다. `NumOfChirpsAccum`이 SPI
배열에 반영되는 방식이 예상과 다르면 파서는 즉시 실패하고 실측 형상에 맞춰 계약을
수정한다.

## 데이터 흐름

```text
IWRL6432 RF
  +-- XDS110 UART --> 기존 heatmap + point + 마지막 2 chirp --> UART 기준선
  `-- J2 SPI ------> FT232H USB --> 전체 ADC binary + capture manifest
                                         |
                                         v
                                  엄격한 frame parser
                                         |
                           Range FFT + BPM + Doppler + calibration
                                         |
                           0-30 cm range-angle evidence surface
                                         |
                              기존 확률 지도와 경로기의 별도 입력
                                         |
                                   UART/SPI A/B 보고서
```

각 기록 세션은 raw 파일과 함께 다음 manifest를 보관한다.

- profile 파일의 SHA-256
- appimage SHA-256 및 SDK 버전
- ADC samples, RX, chirp/burst, burst/frame, frame period
- FTDI serial number
- 캡처 시작 host monotonic time과 UART 첫 frame number
- 장면 ID, 실측 벽 좌표, 센서 높이와 자세
- 예상/수신 frame 수, 손실 및 길이 오류 수

## 전체 ADC 처리

### 1. 수집 무결성

- 각 프레임의 예상 sample 수와 파일 길이를 정확히 검증한다.
- 프레임 누락, 중복, 잘린 프레임, 포화 sample 및 상수 sample을 구분한다.
- 잘못된 프레임을 0으로 채우거나 이전 프레임으로 대체하지 않는다.
- 원본 raw 파일은 수정하지 않고 파생 산출물을 별도 저장한다.

### 2. 신호처리

- signed REAL16 변환
- RX/DC offset 제거와 Hann window
- RX별 Range FFT
- BPM 두 TX 복원
- frame 전체 chirp를 이용한 Doppler FFT와 coherent/noncoherent 누적 비교
- zero-Doppler 정지 반사를 장애물 후보로 유지
- 빈 장면 clutter는 별도 calibration으로 관리하고 현재 장면을 자동 배경으로 학습하지 않음
- corner reflector를 이용한 range bias 및 RX gain/phase 보정
- 6 virtual antenna conventional beamforming을 기본 각도 추정기로 사용
- covariance가 충분한 경우에만 MVDR을 실험군으로 평가하며 기본 경로에 바로 사용하지 않음
- 신호 세기는 hard admission threshold가 아니라 확률과 불확실성에 반영

### 3. 주행 정보

- 0-30 cm를 주 화면 범위로 유지한다.
- 기하학적 바닥 제거만 적용하고 약한 신호라는 이유로 측정을 삭제하지 않는다.
- 벽 모양을 특정 직선, V자 또는 ㅅ자로 가정하지 않는다.
- 관측된 반사면과 미관측 영역을 구분하며 UNKNOWN을 FREE로 바꾸지 않는다.
- 경로는 로봇 폭 15 cm와 양쪽 3 cm margin을 사용한다.
- 전체 ADC 결과는 A/B 합격 전까지 화면의 진단 layer일 뿐 제어 권한이 없다.

## A/B 실측 평가

모든 장면은 바닥에 센서 원점과 벽의 두 끝점을 표시하고 줄자로 좌표를 기록한다.
UART와 SPI 기록 사이에는 센서, 벽, 케이블을 움직이지 않는다.

### 장면 세트

1. 빈 장면 및 센서 자체 결합 측정
2. 정면 단일 평면을 10, 20, 30 cm에 배치
3. 단일 벽을 좌우 각각 -30, -15, 0, +15, +30도로 배치
4. 평행한 양쪽 벽
5. 넓다가 좁아지는 통로와 좁다가 넓어지는 통로
6. 한쪽은 직선, 다른 쪽은 경사진 비대칭 통로
7. 한쪽만 열린 장면과 완전히 막힌 장면
8. 크기와 재질이 다른 상자 및 불규칙 잔해 모사 장면
9. 센서를 직진 및 좌우 회전시키는 저속 이동 장면

각 정적 장면은 최소 60초, 이동 장면은 동일 동작을 최소 5회 기록한다.

### 합격 기준

전체 ADC 방식은 아래 조건을 모두 만족해야 한다.

- 프레임 완전성 99% 이상
- 통과 가능한 양벽 장면의 유효 경로 제공률 90% 이상
- 막힌 쪽을 경로로 선택한 프레임 0개
- 열린 쪽 좌우 선택 정확도 95% 이상
- 벽 거리 median absolute error 5 cm 이하, p95 10 cm 이하
- 통로 중심 heading median error 10도 이하, p95 20도 이하
- 정적 장면 경로 lateral jitter p95 5 cm 이하
- 현재 UART 기준선보다 벽 위치 jitter 또는 잘못된 경로 선택률 중 하나가 25% 이상
  개선되고, 다른 하나가 악화되지 않음
- 노트북 오프라인 처리 p95가 프레임당 125 ms 이하

정확한 벽 윤곽보다 잘못된 통로로 안내하지 않는 것이 우선한다. 절대 기준을 통과하지
못하면 UART보다 일부 수치가 좋아도 실시간 주행 입력으로 승격하지 않는다.

## 실패 처리와 롤백

- FTDI 연결 끊김, BUSY timeout, 길이 오류 또는 calibration mismatch가 발생하면 해당
  SPI 프레임을 폐기한다.
- SPI 처리 상태는 `AVAILABLE`, `DEGRADED`, `UNAVAILABLE`로 구분한다.
- A/B 통과 후에도 실시간 SPI가 `DEGRADED`가 되면 현재 UART R70 경로로 자동 복귀한다.
- 두 소스가 모두 불완전하면 경로를 추측하지 않고 정지/UNKNOWN을 출력한다.
- 기존 UART appimage, profile과 실행 명령은 항상 복구 가능한 산출물로 유지한다.

## Raspberry Pi 4 이식

노트북 A/B가 통과한 뒤에만 진행한다.

### 물리 연결

- C232HM USB-A를 Raspberry Pi 4 USB 3.0 또는 USB 2.0 포트에 연결한다.
- 개별선과 J2 배선은 노트북 구성과 동일하다.
- 레이더 전원은 별도 안정화 5 V 공급 또는 기존 USB 전원으로 유지한다.
- FTDI 빨간 VCC 선은 Pi에서도 연결하지 않는다.

### 소프트웨어

- Pi OS 아키텍처를 `uname -m`으로 확인한다.
- ARMv7 또는 ARMv8 FTDI D2XX와 LibMPSSE-SPI를 설치한다.
- native capture service가 SPI Mode 0, FTDI MPSSE clock, active-low CS 및 BUSY를
  처리하고 프레임을 binary ring buffer에 기록한다.
- Python 처리 프로세스는 shared-memory 또는 bounded queue로 완전한 프레임만 받는다.
- Pi의 연산량이 부족하면 beamforming 각도 bin과 Doppler output을 줄이되 raw 수집
  형식과 안전 판정은 변경하지 않는다.

### 네트워크 출력

BATMAN 메쉬에는 다음만 전송한다.

- 0-30 cm local probability grid
- 경계 evidence와 uncertainty
- 폭을 고려한 후보 경로
- hazard와 데이터 건강 상태
- 필요한 경우 낮은 주기의 진단 snapshot

전체 ADC의 상시 전송은 금지한다. 문제 분석을 위한 원본 기록은 Pi 로컬 저장장치에
순환 보관하고 명시적인 회수 명령이 있을 때만 전송한다.

## 구현 순서

1. 구매품과 J2 배선 검사표를 확정한다.
2. 현재 UART appimage/profile과 실측 결과를 기준선으로 동결한다.
3. Windows D2XX 및 C232HM 열거 테스트를 만든다.
4. SPI streaming demo를 별도 appimage로 빌드하고 flash한다.
5. TI 도구로 짧은 전체 ADC 파일을 수집하고 길이/FFT peak를 검증한다.
6. binary canonical format, manifest 및 엄격한 parser를 구현한다.
7. 전체 프레임 신호처리와 range-angle evidence 생성을 구현한다.
8. 고정된 장면 세트를 UART와 SPI로 기록한다.
9. 자동 A/B 보고서로 합격 기준을 평가한다.
10. 통과한 경우에만 노트북 실시간 화면에 shadow mode로 연결한다.
11. shadow mode 안정성 검증 후 SPI 경로를 우선 입력으로 승격한다.
12. 같은 케이블을 Pi 4에 연결하고 native capture service를 이식한다.
13. Pi 처리 시간, 온도, 저장장치와 메쉬 대역폭을 검증한다.

## 공식 근거

- TI xWRL6432 MmWave Demo SPI ADC Streaming:
  https://software-dl.ti.com/ra-processors/esd/MMWAVE-L-SDK/05_05_00_02/exports/api_guide_xwrL64xx/MMWAVE_DEMO.html
- TI IWRL6432BOOST User Guide:
  https://www.ti.com/lit/ug/swru596/swru596.pdf
- FTDI C232HM-DDHSL-0:
  https://ftdichip.com/products/c232hm-ddhsl-0-2/
- FTDI LibMPSSE-SPI Linux ARM packages:
  https://ftdichip.com/software-examples/mpsse-projects/libmpsse-spi-examples/
- FTDI D2XX drivers:
  https://ftdichip.com/drivers/d2xx-drivers/
