# Robo Control Lab

## 실제 카메라 코드: 영상 입력과 경기장 좌표 보정

`robo_control.vision`은 실제 USB 카메라 또는 녹화 파일을 읽어 경기장 평면을
원근 보정하고, 영상 픽셀을 **좌하단 원점의 mm 좌표**로 변환합니다.
보정값 JSON 저장, 별도 실측 기준점 오차 검사, 프레임 시각·순서·해상도 검사와
처리 로그를 제공합니다. 다중 프레임 추적·ESP32 주행 연동은 다음 단계입니다.

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[vision]"
.\.venv\Scripts\python.exe -m robo_control.vision calibrate --camera 0 --output recordings/camera.json
.\.venv\Scripts\python.exe -m robo_control.vision run --camera 0 --calibration recordings/camera.json --preview --frames 300 --report recordings/camera-frames.jsonl
```

처음 실행할 때 카메라가 연결되어 있어야 합니다. 네 모서리를 경기장 기준
좌상→우상→우하→좌하 순서로 클릭하고 Enter를 누릅니다.
설치·녹화 파일 사용·검증 방법은 [카메라 보정 사용 설명](docs/CAMERA_CALIBRATION_KO.md)을 참고하세요.

### 실제 로봇 4대 AprilTag 검출

기본 태그 설정은 `DICT_APRILTAG_36h11`의 ID 0~3을 각각
`H1`, `H2`, `B1`, `B2`로 지정한다. 카메라 보정 후 다음 명령으로 실제 영상의
로봇 회전중심(mm)과 방향(rad)을 JSONL로 기록할 수 있다.

```powershell
.\.venv\Scripts\python.exe tools/generate_robot_tags.py --output-dir recordings/printable-tags
.\.venv\Scripts\python.exe -m robo_control.vision detect --camera 0 --calibration recordings/camera.json --tags config/robot_tags.json --frames 300 --preview --report recordings/tag-live.jsonl
```

미등록 태그, 중복 ID, 경기장 밖 좌표는 로봇 관측으로 사용하지 않는다.
`observation_complete`는 한 프레임의 태그 구성이 완전하다는 의미이며 모터 제어
허가가 아니다. 인쇄 방향·장착 오프셋·태그 높이와 렌즈 오차를 실물로 측정해야 한다.
자세한 계약과 시험법은 [AprilTag 검출 설명](docs/APRILTAG_TRACKING_KO.md)에 있다.

현재 실제 비전 CLI는 실행당 카메라 한 대를 처리한다. 아래의 고정 카메라 2대는
시뮬레이터의 관측 가정이며, 두 실영상의 시간 동기화·중복 관측 융합은 아직
구현하지 않았다. 카메라별 보정과 단일 영상 검출을 먼저 실측한 뒤 연결한다.

## 현재 기본 구성: 햄스터 2 · 비버 2 · 고정 카메라 2

**[새 예선 임무 시뮬레이터 열기](https://robo-six-arena-simulator.magic-shark-7297.chatgpt.site)**

지금 실행되는 사이트는 한국 Senior 예선 초안을 기준으로 물체를 **집기·운반·해제·최종 배치 채점**합니다.
기존 6대 범용 경로·차동/메카넘 비교 코드는 `기존 구동 시험` 탭으로 보존했습니다.
현재 기본 팀은 지상로봇 4대와 고정 카메라 2대이며 드론 없이 작동합니다.
박쥐 드론 1대는 화면에서 선택하는 비교용 관측 방식이고, 더 이상 지상로봇 6대가 아닙니다.

- 햄스터 2대: 디스크 3개를 서로 다른 LAB 원으로. 게이트·광센서 모의.
- 비버 2대: 빨강 3개→H, 초록 3개→RZ, 노랑 3개→양쪽 PCC. 집게 서보 모의.
- 비버의 큐브: 2개씩 사전 적재하여 H 2개/PCC 각각1개 배분. 별도 배출 서보 모의.
- 고정 카메라 2대: 기본 관측 모드. 두 영상을 합친 좌표 입력으로 지상 임무 수행.
- 박쥐: 선택 모드. 시작 구역 이륙·상공 관측 모의.
- 최종 160점, 경계 접촉·물체 미해제·혼색·노랑 한쪽 몰림·120초 종료를 판정.

**디스크에는 색별 목적지 규칙이 없고, 원기둥은 색에 맞게 분류합니다.**
햄스터도 왼쪽 아래 격리구역으로 이동하므로 지상 4대 전부의 충돌을 검사합니다.
외곽은 1,143×1,181mm, 출발구역은 오른쪽 아래 480×280mm입니다. B-1/B-4 세부 배치 공개 전이라
LAB·RZ·색 순서·샘플 좌표는 잠정값입니다. 126×100mm 명목 차체와 150mm 가상 드론은 실측 보정이 필요합니다.
사이트는 공개 상태라 로그인 없이 누구나 열 수 있습니다.

### 새 Python 제어 코어 실행

Python 3.11 이상, 추가 패키지 없이 실행됩니다.

```bash
python -m robo_control.qualifier
python -m robo_control.qualifier --fail-signal optical_present
python -m unittest discover -s tests -v
```

정상 합성센서 데모는 160점 JSON을 출력합니다. 센서 실패를 넣으면 확인 없이 조작을 성공 처리하지 않습니다.
웹의 약 89초는 가상 주행 시간이며 Python의 단순 합성 시간과 성능 비교하면 안 됩니다.
웹·Python은 같은 좌하단 원점/물체/목적지/작업 장면을 쓰지만 서로 실시간 연결된 것은 아닙니다.
위 실제 영상 관측 결과와 모터·드론·무선통신은 제어 코어에 아직 미연결입니다.
이 코드는 실물 완주시간이나 비행 허용을 보장하지 않습니다.

- [웹 규칙·동선·기구·시험 상세](web_simulator/README.md)
- [Python 제어 설계 상세](docs/QUALIFIER_CONTROL_KO.md)
- [예선 장면 설정](config/qualifier_senior.json)
- [예선 제어 코어](robo_control/qualifier.py)

2026-09-05 현재 vision 의존성 환경에서 Python 172개·웹 26개 회귀시험을 통과했습니다.

---

## 아래는 이전 6대 범용 경로 실험의 보존 문서

`python -m robo_control`과 `START_SIMULATOR.bat`는 아래의 **이전 실험**을 엽니다.
최신 팀/규칙은 위 예선 모듈과 웹 기본 화면을 사용하세요. 아래 6대 수치와 시간은 이전 실험에만 해당합니다.

6대의 소형 지상 로봇을 중앙 컴퓨터에서 추적하고, 임무를 배정하고, 충돌하지
않는 경로를 계산하기 위한 **하드웨어 독립형 1차 소프트웨어**입니다. 지금은
부품이 하나도 없어도 실행되는 결정론적 2D 시뮬레이터와 웹 관제 화면을
제공합니다.

> **중요:** 이 버전은 실제 로봇·카메라·드론으로 검증되지 않았습니다.
> 드론, 외부 서버, 무선통신 및 조종기 자동화가 대회에서 허용되는지도 아직
> 운영진의 서면 답변으로 확정되지 않았습니다. 기본 실행에는 장치 명령 경로가
> 연결·활성화되어 있지 않아 실제 장치로 어떤 명령도 전송하지 않습니다.

## 바로 실행하는 웹 시뮬레이터

설치 없이 다음 주소에서 6대 로봇의 출발·이동·충돌 여유를 확인할 수 있습니다.

**[Robo 6대 경기 시뮬레이터 열기](https://robo-six-arena-simulator.magic-shark-7297.chatgpt.site)**

이 웹 버전은 `hamster_print_ready_v0.7.scad`의 명목 외형을 기준으로 계산한
보수적 8각형 발자국(`126 × 100mm`, 회전 포락 반경 `81mm`)과
국제 룰북의 한 팀용 `1,143 × 1,181mm` 경기장과 아래쪽 오른편
`480 × 280mm` 출발구역, 공식 20mm 테이프 선형과 주요 구역을 사용합니다. 6대를
한꺼번에 회전시키기 어려운 출발구역 조건을 확인할 수 있도록 순차 출발 경로를
제공하며, 현재 차동구동과 가상의 4륜 메카넘 구동을 비교할 수 있습니다.
메카넘 모드에서는 앞뒤 바퀴 4개까지 감싸는 별도 8각형 외곽을 사용합니다.
현재 사이트는 소유자 계정으로 로그인해야 열리는 접근 설정입니다.

공식 한국 예선 B-1/B-2/B-4 고정 배치가 아직 공개되지 않아 LAB의 정확한 좌표,
로봇 시작점과 자동 경로는 **임시 검증용**이고, 실제 AI 최적화나 로봇 명령
송신은 연결하지 않았습니다. RC카 분석 결과와
전후·좌우·회전 바퀴 명령, 개조안, 검증 순서는
[`docs/RC_MECANUM_ADAPTATION_KO.md`](docs/RC_MECANUM_ADAPTATION_KO.md)에
정리했습니다.

## 지금 구현된 것

- 국제 한 팀용 필드 `1.143 × 1.181m`, 경기시간 120초, 가상 로봇 6대
- 우선순위 기반 임무 배정과 6대 기본 시나리오의 정확한 최소비용 일대일 매칭
- 20mm 격자와 시간 예약을 사용하는 다중 로봇 A*
- 같은 시간 같은 위치·정면 자리 교환·tick 중간 연속 최소거리 충돌 방지
- 정적 연결 성분 사전검사, 정지 로봇 예약, 도달 불가 시 재계획 폭주 방지
- 반경 63mm의 단순 원형 모델과 138mm 최소 중심거리로 가상 충돌 감시
- 공식 테이프·의료구역·환자 위치와 잠정 경로·로봇 상태를 표시하는 실시간 웹 대시보드
- 시작, 일시정지, 초기화, 재계획, 전체 긴급정지
- USB/RTSP 카메라, ESP32 UDP 통신, 드론 조종기용 독립 어댑터 경계 클래스
- 외부 패키지 없이 실행되는 자동 테스트와 headless 시뮬레이션

이전 데모는 당시 구상에 맞춰 6대로 구성했지만, 실제 제작은
1대→2대→지상 4대 순서로 안정성을 확인합니다. 480×280mm 출발구역에
6대를 넣으면 기구 오차와 경로 혼잡에 매우 민감합니다.

설정과 사용자 시나리오는 1~8대를 지원합니다. 기본 시나리오는 시작점 6개와
임무 6개를 가지며, 로봇 수를 줄여도 임무는 유지되어 한 라운드가 끝날 때마다
자동 재배정·재계획합니다. 7~8대는 시작점을 추가한 별도 `--scenario` 파일이
필요합니다. 남은 임무가 모두 도달 불가이면 반복 계획하지 않고 일시정지합니다.

현재 반경 63mm 원은 126×100mm 직사각형 차체를 임의 방향에서 감싸는
보수적 모델이 아닙니다. 해당 직사각형의 외접반경은 약 80.4mm이므로,
138mm 중심거리와 현재 자동시험만으로 실물 차체 비접촉을 보장할 수 없습니다.
실물 투입 전에는 회전을 포함한 외형 모델과 실측 안전여유를 다시 정해야 합니다.

## 3분 안에 실행하기

Python 3.11 이상만 있으면 됩니다. OpenCV나 GPU는 기본 시뮬레이터에
필요하지 않습니다.

Windows에서는 `START_SIMULATOR.bat`를 더블클릭해도 됩니다. 터미널에서는:

```bash
python -m robo_control
```

브라우저가 자동으로 열리지 않으면 다음 주소를 엽니다.

```text
http://127.0.0.1:8080
```

브라우저 없이 기본 시나리오를 최대 속도로 smoke test하려면:

```bash
python -m robo_control --headless
```

120초 제한은 설정되어 있지만 기본 6대 임무는 시뮬레이션 시각 약 8.05초에
끝나므로, 이 명령만으로 정확한 120초 timeout 동작까지 검증되는 것은 아닙니다.

자동 테스트:

```bash
python -m unittest discover -s tests -v
```

2026-09-05 재검증에서 Python 35개, 웹 시뮬레이터 11개 테스트가 모두
통과했습니다. Python 기본 미션은 가상시간 8.05초에 6/6 완료했고, 웹의
메카넘 임시 경로는 약 31.99초에 6/6 완료했습니다. 서로 다른 경로·충돌 모델을
쓰므로 두 시간은 성능 비교값이 아닙니다.

기본 시나리오를 100회 반복하여 완료율·충돌·계획 시간을 확인하려면:

```bash
python tools/benchmark.py --runs 100
```

현재 측정 기준은 100/100 완료, 100/100 충돌 기록 0, 계획시간 평균
61.328ms·p95 71.633ms·최대 75.243ms입니다. 같은 결정적 기본 시나리오를
반복한 결과이며 무작위 배치 검증이 아닙니다. 시간 수치는 컴퓨터와 부하에 따라
달라집니다.

다른 시나리오 파일을 실행하려면:

```bash
python -m robo_control --scenario config/scenario_demo.json
```

현재 CLI는 `--scenario`와 `--config`를 지원하지만 `--robots`, `--seed`,
`replay` 하위 명령은 아직 지원하지 않습니다. 로봇 수는 설정 파일의
`robot_count`로 바꿉니다.

패키지 명령으로 설치하고 싶다면:

```bash
python -m pip install -e .
robo-control
```

## 화면에서 보는 정보

- 색이 있는 사각형: 로봇 본체와 진행 방향
- 점선: 시간 순서가 포함된 예약 경로
- 색 원: 각 로봇에 배정된 가상 임무
- 붉은 직사각형: 시나리오에 적힌 장애물 원본 외형. 계획기는 화면에 따로
  칠하지 않은 반경·경계 여유만큼 더 팽창시켜 우회
- 녹색 영역: 480×280mm 출발구역
- 오른쪽 패널: 남은 시간, 완료 수, 계획 시간, 충돌 및 안전 이벤트

`긴급정지`는 모든 로봇을 즉시 `stopped` 상태로 만들며 자동으로 다시
움직이지 않습니다. 다시 시험하려면 `초기화`를 먼저 누릅니다.

## 프로젝트 구조

```text
Robo/
├─ .github/workflows/ci.yml          테스트·패키징·Windows 설치 smoke
├─ START_SIMULATOR.bat              Windows용 더블클릭 실행 파일
├─ MANIFEST.in                      설치 패키지에 기본 데이터를 포함하는 목록
├─ config/default.json              경기장·속도·안전 설정
├─ config/scenario_demo.json        로봇·임무·장애물 예시 좌표
├─ robo_control/
│  ├─ config.py                     설정·패키지 데이터 로드와 검증
│  ├─ models.py                     공통 데이터 모델
│  ├─ planner.py                    임무 배정·시간예약 A*
│  ├─ simulation.py                 6대 결정론적 시뮬레이션
│  ├─ server.py                     REST API·웹 관제 화면
│  ├─ adapters.py                   카메라·UDP·드론 교체 경계
│  ├─ __main__.py                   실행 명령
│  └─ data/                         설치본용 기본 설정·시나리오 사본
├─ tests/                            하드웨어 없는 자동 검증
├─ tools/benchmark.py                반복 성능·안전 벤치마크
├─ docs/IMPLEMENTATION_GUIDE_KO.md   상세 구현·실물 전환 설명서
├─ docs/RC_MECANUM_ADAPTATION_KO.md  전방향 RC카 원리·개조·검증 설명서
├─ docs/TEST_PLAN_KO.md              자동·실물 시험 항목과 합격 기준
├─ web_simulator/                    공개 웹 시뮬레이터 소스·자동시험
└─ ROBOTICS_PROJECT_FULL_RECORD_KO.txt  전체 기획 기록
```

소스 트리에서는 `config/`를 사용하고, wheel/sdist 설치본에서는 패키지에
포함된 `robo_control/data/`로 fallback합니다. CI와 패키징 회귀시험이 두
기본 데이터 사본의 일치와 설치본 실행을 확인합니다.

## 핵심 API

서버가 실행 중일 때:

| 메서드 | 주소 | 기능 |
|---|---|---|
| `GET` | `/api/state` | 전체 세계상태와 경로 조회 |
| `GET` | `/healthz` | 서버 생존 확인 |
| `POST` | `/api/mission/start` | 경기 시작 |
| `POST` | `/api/mission/pause` | 일시정지 |
| `POST` | `/api/mission/reset` | 데모 초기화 및 재계획 |
| `POST` | `/api/mission/replan` | 현재 위치에서 다시 계획 |
| `POST` | `/api/mission/stop` | 전체 안전정지 |

## 실제 장비를 연결할 때

현재 `SyntheticCameraSource`, `OpenCVCameraSource`, `DryRunTransport`,
`UdpRobotTransport`는 독립 경계 클래스일 뿐 `SimulationEngine`의 관측→계획→
명령 폐루프에 연결되어 있지 않습니다. 따라서 클래스를 단순 교체하는 것만으로
실제 로봇이 움직이지 않습니다. 다음 통합 작업과 시험이 먼저 필요합니다.

1. 카메라 프레임에서 AprilTag/ArUco를 검출하고, 4개 이상의 경기장 기준점으로
   픽셀 좌표를 미터 좌표로 변환하는 관측 파이프라인을 구현합니다.
2. 관측 시각·신뢰도·로봇 ID를 검증해 `RobotState`를 갱신하고, 0.5초 이상
   오래된 추적은 안전정지로 연결합니다.
3. 계획 경로를 `RobotCommand`로 변환하는 추종 제어와 sequence·TTL 검사를
   구현한 뒤, 먼저 `DryRunTransport`로 명령 내용을 폐루프 검증합니다.
4. ESP32가 목표좌표, 속도 제한, 시퀀스 번호, TTL을 받고 자체 엔코더 PID를
   수행하도록 만든 뒤 loopback·1대 HIL 시험을 통과시킵니다.
5. 새 유효 명령이 300~500ms 안에 도착하지 않으면 펌웨어가 독립적으로 모터를
   정지하게 하고, 이 watchdog을 검증한 뒤에만 `UdpRobotTransport` 출력을
   명시적으로 활성화합니다.

드론은 순정 조종기를 별도 ESP32가 전기적으로 조작하는 방안을 가정하지만,
조종기 전압·축 방향·Failsafe가 측정되기 전에는 출력 코드를 만들거나 연결하면
안 됩니다. 현재 `DisabledDroneController`가 의도적으로 모든 드론 출력을
거부합니다.

## 설정에서 먼저 바꿀 값

`config/scenario_demo.json`의 좌표는 소프트웨어 검증용 예시이며 공식 배치가
아닙니다. 실물 치수를 재면 `default.json`과 시나리오 파일을 다음 순서로
수정합니다.

1. 경기장과 출발구역의 실제 치수
2. 바퀴를 포함한 로봇 외형과 안전여유
3. 실제 직진속도 및 회전속도
4. 공식 물체·장애물·목적지 좌표
5. 카메라 프레임률과 추적 타임아웃
6. 로봇별 IP, 포트 및 통신 watchdog

전체 설계, 단계별 실물 전환, 테스트 기준과 알려진 한계는
[`docs/IMPLEMENTATION_GUIDE_KO.md`](docs/IMPLEMENTATION_GUIDE_KO.md)를
참고하세요.
