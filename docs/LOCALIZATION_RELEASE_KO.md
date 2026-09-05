# 2026-09-05 코드 감사와 실시간 좌표 추적 배포

## 무엇을 실제로 확인했나

GitHub main을 새로 가져와 기존 배포 a93376b 이후 **ef5e020 (실제 비전 구현), ac51e89 (팀 인계 문서)** 두 변경을 검토했다. 다른 팀원이 만든 코드를 보존하고 그 위에 수정했다.

| 요청한 기능 | 검토한 기존 상태 | 이번 변경 |
|---|---|---|
| USB·웹캠·녹화 입력 | 구현됨 | 기존 입력 경로 유지 |
| 네 모서리, 원근, 픽셀↔mm, 저장·기준점 오차 | 구현됨 | 유지, 움직이는 카메라에 그대로 사용 금지 |
| AprilTag ID·위치·방향·장착 오프셋 | 구현됨 | 인쇄 ID 유지, 역할 분리 |
| 인쇄 PNG 생성 | 구현됨 | 재생성 불필요. 설치 위치 변경 시 오프셋 재실측 |
| 미등록·중복·누락·크기 거부 | 구현됨, 크기 기본값 null로 비활성 | 평균 변 길이 대신 각 변 검사; 실측 없이는 준비 완료로 표시하지 않음 |
| 지연·순서·해상도 | 구현됨 | 검출기 해상도 오류도 프레임별 거부 JSONL로 남기고 다음 입력 처리 |
| 연속 위치 추적 | 없었음 | --track: 위치 이력·속도·각속도·누락·500ms 만료·점프 검사 |
| 색 물체 검출 | 없었음 | --colors: 빨강·노랑·초록 원기둥 후보, HSV+mm 면적+원형도 |
| 5대 이상 미션 구조 | 검출기만 일반화, 제어 코어는 4대 제한 | 역할 registry로 작업 배정·소유권·충돌·정지 목록 일반화 |
| 실제 모터·드론 제어 | 미연결 | 계속 미연결. 코드만으로 장비 검증을 주장하지 않음 |

## 팀과 임무

| 내부 ID | 태그 번호 | 현재 역할 | 임무 |
|---|---:|---|---|
| H1 | 0 | 햄스터 1대 | D1·D2·D3 → LAB |
| H2 | 1 | 세 번째 비버, 화면 이름 B3 | R2·Y2·G2 |
| B1 | 2 | 한가한 비버 | C1·C2·R1·Y1·G1 |
| B2 | 3 | 바쁜 비버 | C3·C4·R3·Y3·G3 |

H2라는 문자열이 남아 있다고 두 번째 햄스터인 것이 아니다. `ground_robots[].role`이 기능을 결정한다. 인쇄된 태그·통신 식별자를 임의로 바꾸지 않기 위해 H2를 유지했다. 실물 기구가 바뀌면 태그 장착 오프셋은 다시 측정해야 한다. B1·B2에만 큐브 두 개씩을 적재한다.

## 영상은 입력, 운영 화면은 좌표 지도

카메라/녹화 → 프레임 안전 검사 → 보정된 mm 좌표 → AprilTag 로봇 관측 + 색 물체 후보 → ID별 위치 이력 → 안전 상태 → 향후 경로 제어기 순서다. 카메라 두 대 설치를 전제로 묶지 않는다. 다만 영상으로 절대 위치를 측정하려면 실제 영상 입력은 여전히 필요하다. 영상 없는 자가 위치 추정을 원한다면 엔코더·IMU·거리/기준점 센서용 별도 어댑터가 필요하며 이번에 구현한 것으로 취급하지 않는다.

`--track --preview`는 원본 영상 대신 경기장 좌표 지도를 보여 준다. 로봇 위치·방향, 색 원기둥 후보, 관측/누락 상태를 실제 프레임이 들어올 때마다 갱신한다. 원본 영상 주석 확인은 `--track` 없이 `--preview`를 실행하면 된다.

공개 웹 시뮬레이터는 이상적 센서를 10Hz로 모의한다. 물리 상태와 별도 측정 시각을 가진 좌표 목록을 표시하며, 전체 입력 단절(500ms 초과) 또는 특정 태그 누락(즉시) 시 지상팀을 대기시킨다. 오차·블러·미끄러짐은 없는 모델이다. 물리 이동과 집기 제어는 여전히 모의이며, 화면에 좌표가 나온다고 OpenCV나 실제 로봇이 연결된 것은 아니다.

웹의 `검출 JSONL 확인`은 Python이 만든 로그를 브라우저 안에서 읽는다. 프레임 슬라이더, 측정 지도, mm·각도·나이, 색 후보를 확인할 수 있다. 파일은 서버에 업로드하지 않는다. 이 기능은 로그 재생이지 장비의 실시간 네트워크 연결은 아니다. 로봇 대수는 로그에서 읽으므로 5·6·12대도 표시한다.

## 실행 방법 (Windows, 저장소 루트)

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[vision]"
.\.venv\Scripts\python.exe -m robo_control.vision calibrate --camera 0 --output recordings/camera.json
.\.venv\Scripts\python.exe -m robo_control.vision detect --camera 0 --calibration recordings/camera.json --tags config/robot_tags.json --fleet config/qualifier_senior.json --track --colors config/object_colors.json --frames 100000 --preview --report recordings/live-positions.jsonl
```

Q/Esc 또는 창 닫기로 종료한다. 카메라가 없으면 마지막 명령의 `--camera 0`을 `--video recordings/run.mp4`로 바꾼다. 첫 보정도 `calibrate --video ...`나 `--image ...`를 지원한다. 다른 카메라 위치·해상도에서 만든 보정값을 재사용하면 안 된다.

먼저 인쇄물의 **검은 사각형 한 변**을 mm로 실측해 `config/robot_tags.json`의 `tag_size_mm`에 넣는다. 임의의 숫자를 넣어 오류 검사를 통과시키지 않는다. 보정점과 태그 높이 시차·렌즈 왜곡 검증 전에는 `hardware_verified=false`를 유지한다. 태그 생성기는 기존 `tools/generate_robot_tags.py`를 그대로 사용한다.

색 임계값은 `config/object_colors.json`에서 조정한다. OpenCV HSV의 H는 0~179이며 빨강은 양 끝 구간 두 개를 합친다. 기본값은 연습용이다. 색만으로 같은 색 물체를 구분할 수 없으므로 `identity=null`과 `classification=color_geometry_candidate`를 출력한다. 원기둥 외 디스크·큐브는 임의로 색을 가정하지 않았으며 기본 프로필로 검출된다고 보장하지 않는다. 배경·바닥 표식과 같은 색, 가림, 붙은 물체, 그림자에서 별도 검증해야 한다.

## 관측 계약과 안전

- `tracks`: 등록된 모든 ID의 마지막 위치·방향, observed/missing/stale 상태, 마지막 관측으로부터의 age_ms.
- `--fleet`은 미션의 ID·tag_id 목록과 실제 태그 설정을 교차 검증한다. 한쪽만 수정하면 시작 시 거부한다. 일반 검출 실험에는 생략할 수 있지만 미션 연결 전에는 반드시 사용한다.
- 누락된 로봇의 옛 좌표는 지도 참고용일 뿐 `valid_for_control=false`다. 가림 동안 좌표를 추측해 이동시키지 않는다.
- `observation_usable`: 현재 프레임에 전체 등록 로봇이 모두 정상이고 미등록·중복/점프가 없는지. 위치 정확도·기계 안전 인증이 아니다.
- `stop_required`: 관측 계층의 대기 요청. 실제 정지 명령이 전송되었다는 뜻이 아니다.
- `hardware_ready`, `motion_permitted`, `device_io`: 모두 false. 실측 표시가 있더라도 모터 출력은 자동으로 열리지 않는다.
- EOF·정상 종료·읽기 실패 시 마지막 `source_closed` 레코드로 대기를 표시한다. 프로세스 강제 종료/카메라 read() 정체는 이 레코드를 보낼 수 없으므로 소비자와 로봇 펌웨어에 독립 watchdog이 반드시 필요하다.
- 카메라 지연은 호스트 read 시작 기준이다. 카메라 내부 버퍼에 이미 오래된 영상인지까지는 알 수 없다.
- 녹화의 속도 계산은 media_time_s, 지연 검사는 호스트 단조 시각으로 분리한다. 녹화를 빠르게 디코딩했다고 로봇이 순간 이동한 것으로 판정하지 않는다.
- 다른 호스트의 monotonic 시각은 직접 비교할 수 없다. 소스 재접속은 새 FrameProcessor/PoseTracker 세션으로 시작한다. 여러 카메라 융합은 별도 구현이다.
- 실제 출력 heading_rad와 시나리오 ground_robots의 yaw_rad는 **+X=0, 반시계 양수**, mm 좌표는 좌하단 원점이다. 웹 물리 엔진의 옛 heading은 +Y=0이므로 연결 시 `normalize(real_heading - π/2)`로 바꾸고 mm를 1000으로 나눈다. 측정 지도는 +X 방향을 직접 그린다.

## 드론 있음/없음

드론 선택은 웹에 유지했다. 그러나 실제 드론이 이동하면 고정 시점에서 저장한 호모그래피가 무효가 된다. 해상도가 같다는 이유로 유효한 보정이 아니다. 매 프레임 경기장 기준 태그/코너를 재검출하거나 카메라 자세와 보정 유효성을 검증하는 어댑터가 필요하다. 현재 CLI는 `--moving-camera`를 지정하면 실행을 거부한다. 이 옵션을 빼서 안전 검사를 우회해서는 안 된다. 실제 드론 영상·비행 제어·렌즈 왜곡·태그 높이 보정은 이번 구현 범위에서 완성되지 않았다.

## 시험 결과와 5대 이상 제한

기존 코드: OpenCV 5.0.0 / NumPy 2.5.2 설치 환경에서 172개 기존 Python 시험이 모두 통과했다. 이번 변경 후 Python 183개, 웹 30개 자동 시험이 통과했다. 장비 없이 합성 영상·파일 디코딩·단위/통합 시험으로 검증했다.

- 기본 **1햄스터+3비버**: 가상 73초, 16개/160점, 전체 이동에서 12mm 모델 간격 및 로봇/물체 충돌 검사 통과. 드론 선택 모드도 같은 지상 결과.
- 5·6·12개의 실제 OpenCV 생성 태그 이미지 검출: ID·회전·좌표 검사 통과.
- 5·6·12대 역할 registry, 작업 배정, 소유권, 모든 로봇 쌍 충돌 검사, 전체 정지 의도 목록: 통과.
- 5대 웹 초기 배치/좌표 갱신/추가 로봇 누락 시 전체 대기: 통과.
- **5대 동시 주행 혼잡 실험은 완주 실패**: 120초, 100점, 충돌 검사 위반 0회, 통로 및 구역 대기가 남음. 4대에서 통과한 동선을 단순 분할하면 교착이 생긴다. 즉 “데이터 구조 확장 가능”과 “5대 주행 안정화 완료”는 다르다.

재현:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
node --experimental-strip-types --test web_simulator/tests/*.test.ts
node --experimental-strip-types web_simulator/tools/fleet-expansion.ts
```

실제 5대 이상을 늘리기 전 시간 기반 통로 예약·교착 탐지/양보 지점·우선순위 재조정, 출발구역과 기구 실측, 무선 지연·비상정지를 검증해야 한다. 12개 태그 검출이 12대 로봇을 작은 경기장에 넣어 안정 주행할 수 있다는 의미는 아니다.

## 다음 팀원이 수정할 위치

- `config/qualifier_senior.json`: 로봇 role/id/name, 실제 기구·초기 위치, 명시 임무. 패키지용 data 사본도 동기화.
- `config/robot_tags.json`: 태그 대응·실측 크기·장착 오프셋. 같은 ID 중복 금지.
- `config/object_colors.json`: 조명에 맞춘 HSV/면적/원형도.
- `robo_control/fleet.py`, `qualifier.py`: 임의 대수 역할 registry와 미션 코어.
- `robo_control/vision/tracking.py`: 시간축 관측과 대기 게이트. 모터 어댑터는 별도.
- `robo_control/vision/colors.py`, `map_view.py`: 색 후보와 실시간 좌표 지도.
- `web_simulator/lib/mission.ts`: 역할·작업·출발·이탈·주차를 갖는 FleetSpec. createWorld(mode, fleet)로 추가 대수 주입. 인원만 늘리고 동선을 검증하지 않는 것은 금지.
- `web_simulator/lib/localization.ts`: JSONL 검증과 좌표 계약.

API 참고: [OpenCV 태그 검출과 코너 규약](https://docs.opencv.org/5.0/main_modules/objdetect_aruco.html), [OpenCV HSV 범위 분리](https://docs.opencv.org/4.13.0/da/d97/tutorial_threshold_inRange.html). 생성기와 검출기의 태그 사전·전방 규약을 함께 유지한다.
