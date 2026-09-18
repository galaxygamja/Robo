# 구매 부품으로 실제 로봇 시작하기

이 안내서는 햄스터 1대와 비버 3대의 **ESP32-C3 SuperMini + DRV8833 + N20 6 V 100 RPM 2륜 차동구동**을 대상으로 한다. 코드 빌드·업로드, 통신, 짧은 모터 시험, 실제 영상 기반 주행을 차례로 진행한다. 실물 조립과 측정은 사용자가 수행하며, 저장소의 자동 시험 결과는 실물 주행·집기·대회 완주를 증명하지 않는다.

현재 하드웨어 코드의 출발점은 `python -m robo_control.hardware`다. 기존 웹 시뮬레이터의 시작 버튼을 눌러도 실제 로봇이 움직이지 않는다. 실제 출력은 별도 장치 설정과 명시적인 실행 옵션을 사용한다.

먼저 전원 모듈의 모델명을 확인한다. 구매품이 **XL4015라면 제조사 입력 범위가 8–36 V이므로 2S 배터리의 정상 방전 구간 전체를 지원하지 않는다.** 이 부분을 확인하는 동안에도 USB로 펌웨어 빌드·업로드·네트워크 확인을 진행할 수 있다. 모터·서보 출력은 아래 전원 설명에 맞춰 확인한 뒤 시험한다.

## 1. 로봇 이름과 보드 4개를 먼저 대응시킨다

| 실물 표시 | 내부 ID / 펌웨어 환경 | AprilTag ID | 서보 | 장착한 물체 센서 |
|---|---|---:|---|---|
| 햄스터 | `H1` | 0 | 채널 0: 디스크 게이트 | TCRT5000 1개 |
| 비버 B3 | `H2` | 1 | 채널 0: 앞 집게, 채널 1: 뒤 투하 게이트 | 없음 |
| 한가한 비버 B1 | `B1` | 2 | 채널 0: 앞 집게, 채널 1: 뒤 투하 게이트 | 없음 |
| 바쁜 비버 B2 | `B2` | 3 | 채널 0: 앞 집게, 채널 1: 뒤 투하 게이트 | 없음 |

`H2`는 예전 이름을 유지한 세 번째 비버다. 화면에 B3라고 써도 펌웨어·설정·명령에는 `H2`를 쓴다. 기존 태그와 임무 배정을 유지하기 위한 것이므로 임의로 B3로 바꾸지 않는다. 의료키트 적재는 기존 임무의 `B1=1`, `B2=2`, `H2=1`을 유지한다.

각 보드·배선·태그에 같은 ID를 붙인다. 아래 예시의 `H1` 시험이 끝난 다음 해당 명령의 ID를 바꿔 한 대씩 반복한다.

## 2. 전원과 배선

ESP32의 GPIO 번호 기준이다. 보드 위에 인쇄된 위치와 핀 이름을 먼저 대조한다.

| ESP32-C3 GPIO | 연결 대상 | 의미 |
|---:|---|---|
| 3 | DRV8833 AIN1 | 왼쪽 모터 입력 1 |
| 4 | DRV8833 AIN2 | 왼쪽 모터 입력 2 |
| 5 | DRV8833 BIN1 | 오른쪽 모터 입력 1 |
| 6 | DRV8833 BIN2 | 오른쪽 모터 입력 2 |
| 7 | 서보 채널 0 신호 | 햄스터 게이트 / 비버 앞 집게 |
| 10 | 서보 채널 1 신호 | 비버 뒤 투하 게이트; H1은 연결하지 않음 |
| 0 | H1 TCRT5000 디지털 출력 D0 | 디스크 유무; 다른 로봇은 연결하지 않음 |
| GND | 드라이버·서보·센서·전원 모듈 GND | 공통 기준 전위 |

왼쪽/오른쪽은 로봇 뒤에서 전방을 바라보는 기준이다. DRV8833 AOUT1/AOUT2에는 왼쪽 모터, BOUT1/BOUT2에는 오른쪽 모터를 연결한다. `nSLEEP`이 노출된 모듈은 사용 모듈의 회로도에 맞춰 활성 상태로 둬야 한다. 현재 펌웨어에는 별도 nSLEEP 제어 핀이 없다.

- 모터 전원: 검증한 6 V 전원을 DRV8833 VM에 공급한다.
- 서보 전원: 검증한 별도 5 V 전원을 사용한다. 서보를 ESP32 3.3 V 핀에서 구동하지 않는다.
- 보드 전원: 실제 SuperMini 보드의 5 V 입력 규격을 확인한다. USB와 외부 5 V를 동시에 연결할 때는 해당 보드의 역급전 방지 여부를 확인한다.
- TCRT5000 모듈의 D0가 GPIO에 들어가는 전압은 3.3 V 범위여야 한다. 5 V로 구동되는 모듈 출력을 확인 없이 직결하지 않는다. 공급전압과 출력회로에 따라 레벨 변환이 필요할 수 있다.
- 현재 구성에는 배터리 전압 측정선과 엔코더가 없다. 화면의 명령 속도는 실측 휠 속도가 아니며, 소프트웨어가 LiPo 저전압을 감시한다고 가정하지 않는다.

배터리는 최근 변경한 FMS 2S 7.4 V 350 mAh 팩을 기준으로 하지만, 커넥터 모양만 보고 극성을 판단하지 않는다. 2S 완충 전압은 8.4 V이므로 6 V 모터·5 V 서보에 직접 연결하지 않는다.

**구매한 강압 모듈이 실제로 XL4015라면 전원부 재확인이 필요하다.** 제조사가 명시한 입력 범위는 8–36 V여서, 2S 배터리가 8 V 아래로 내려가는 정상 사용 구간을 보장하지 않는다. 기존 문서의 “XL4015를 2S에 연결하면 된다”는 설명만으로 사용 승인을 내리지 않는다. USB로 보드 업로드·통신 확인을 먼저 진행하고, 모터 시험은 사용할 배터리 전압 범위를 지원하는 전원부 또는 전류 제한 시험 전원으로 진행한다. 근거: [XLSEMI XL4015 데이터시트](https://www.xlsemi.com/datasheet/XL4015-EN.pdf).

## 3. 서버 설치

다음 PowerShell 명령은 저장소 최상위 폴더에서 실행한다. Python 3.11 이상이 필요하다.

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[vision]" platformio
.\.venv\Scripts\python.exe -m robo_control.hardware --help
.\.venv\Scripts\python.exe -m robo_control.hardware init --output local_state/hardware.json
```

`local_state/hardware.json`은 실제 로봇 IP·보정값을 적을 개인 설정이다. `local_state/`는 Git에서 제외된다. 이미 설정 파일이 있다면 덮어쓰지 말고 새 경로로 초기화하거나 기존 파일을 편집한다.

생성 직후에는 실측값과 확인 항목이 채워지지 않았으므로 실제 주행 검증이 통과하지 않는 것이 정상이다. 축 간격과 충돌 반경은 `null`, 속도 곡선은 빈 배열로 생성된다. IP 192.168.4.101~104도 예시이므로 실제 주소로 바꾼다.

프로필 구조는 [hardware.example.json](../config/hardware.example.json)과 같다. 확인 단계에 따라 아래 값을 채운다.

| 설정 | 채우는 값 |
|---|---|
| 최상위 `network_confirmed` | 네 대의 실제 개별 IP·포트를 확인했을 때 `true` |
| 로봇별 `wiring_verified` | 핀, 극성, 부하를 연결한 전원 전압을 확인했을 때 `true` |
| `track_width_mm` | 좌우 바퀴 접지 중심 간 실측 간격 |
| `envelope_radius_mm` | 축 중점에서 적재물·열린 기구까지 포함한 최대 회전 반경 |
| `left_forward`, `left_reverse`, `right_forward`, `right_reverse` | `[PWM, 실측 mm/s]` 점들의 배열; 모두 `[0,0]`으로 시작 |
| `motion_calibrated` | 네 곡선·치수·주행 방향을 실물 확인했을 때 `true` |
| `servo_presets` | 이름별 `[채널0 펄스µs, 채널1 펄스µs]` |
| `sensor_verified` | H1 센서로 실제 디스크 유무와 출력 극성을 확인했을 때 `true` |
| `sensor_active_low` | 펌웨어의 `discActiveLow` 설정과 같은 값 |

속도 곡선은 PWM과 속도가 모두 엄격히 증가해야 한다. 예를 들어 `[[0,0],[30,20],[50,45]]`는 **파일 형식만 설명하는 가상 숫자**이며 로봇에 복사할 보정값이 아니다. 후진 곡선에도 PWM과 속도의 크기를 양수로 기록한다. 실제 명령의 부호가 전후진을 선택한다.

## 4. 펌웨어 빌드와 업로드

`firmware/include/secrets.example.h`를 `firmware/include/secrets.h`로 복사한 뒤 다음 값을 적는다.

- `WIFI_SSID`, `WIFI_PASSWORD`: 서버와 로봇이 사용할 로컬 2.4 GHz Wi-Fi.
- `ROBOT_SHARED_TOKEN`: 팀 로컬 장치 인증용 임의 문자열. 영문자·숫자 24~64자로 생성한다. 저장소에 올리지 않는다.
- `HARDWARE_OUTPUT_ENABLED`: 첫 USB·통신 확인은 `0`; 배선 확인과 실제 시험을 진행할 때만 `1`로 변경하여 다시 업로드한다.

호스트 실행 PowerShell의 `ROBO_HW_TOKEN` 환경 변수에는 `ROBOT_SHARED_TOKEN`과 동일한 값을 넣는다. 실제 비밀값을 문서·스크린샷·공개 로그에 남기지 않는다. 다른 PowerShell 창에서는 환경 변수를 다시 설정해야 한다.

```powershell
.\.venv\Scripts\python.exe -m platformio run --project-dir firmware -e H1
.\.venv\Scripts\python.exe -m platformio device list
.\.venv\Scripts\python.exe -m platformio run --project-dir firmware -e H1 -t upload --upload-port COM5
.\.venv\Scripts\python.exe -m platformio device monitor --project-dir firmware --port COM5 --baud 115200
```

`COM5`는 예시다. `device list`의 실제 포트를 사용한다. H2/B1/B2는 각각 `-e H2`, `-e B1`, `-e B2`로 빌드·업로드한다. 한 보드에 H1 펌웨어를 네 번 복제하지 않는다.

빌드 대상은 ESP32-C3 Arduino 보드 프로필을 사용한다. SuperMini의 실제 플래시·USB 설정과 일치하는지 업로드 로그로 확인한다. USB 포트가 안 보이면 데이터 케이블과 실제 부트 절차를 확인한다. 무선망이 AP 격리 기능으로 로봇끼리/서버와의 통신을 차단하지 않는지도 확인한다.

`firmware/include/robot_config.h`의 핀 표, 모터 극성, 서보 펄스 허용 범위가 실물과 맞아야 한다. 기본 모터 출력 상한은 8비트 PWM의 96이다. 96이라는 값이 96 mm/s나 96%를 뜻하지 않는다.

## 5. 한 대씩 짧은 시험

실제 IP를 개인 설정에 기입하고 확인한 항목만 `wiring_verified`, `network_confirmed`로 표시한다. 보드를 재부팅하거나 통신이 끊기면 이전 출력 권한이 자동으로 이어지지 않도록 구성되어 있다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.hardware validate --config local_state/hardware.json
.\.venv\Scripts\python.exe -m robo_control.hardware bench --config local_state/hardware.json --robot H1 --left-pwm 30 --right-pwm 30 --duration-s 0.5 --enable-hardware --wheels-raised
```

첫 펄스는 바퀴를 바닥에서 띄운 상태에서 실행한다. 아직 속도 보정을 하지 않았어도 짧은 개별 시험은 가능하지만, 미확인 배선·망·인증 정보로 출력을 켜지는 않는다. PWM 30에서 모터가 돌지 않을 수도 있다. 저속 기동 한계는 부하와 기어박스에 따라 다르므로 “0.5초 지났다”는 사실을 이동거리로 변환하지 않는다.

왼쪽만 `30/0`, 오른쪽만 `0/30`, 양쪽 `30/30` 순으로 방향을 확인한다. 둘 다 양수일 때 로봇 전방으로 굴러야 한다. 잘못된 극성은 배선을 수정하거나 펌웨어 설정에서 수정한 다음 다시 확인한다. 지속 반복 명령으로 시험을 생략하지 않는다.

서보는 각도 대신 펄스 폭을 사용한다. `servo_presets`의 값은 `[채널0, 채널1]`이고 `0`은 해당 채널의 펄스 출력 해제다. H1의 두 번째 값은 항상 0이다. 허용 범위는 900~2100 µs지만 이 전체 범위가 장착 기구의 허용 범위라는 뜻은 아니다. 링크를 분리한 초기 중립 확인 후 실제 기구에서 작은 변화로 범위를 정한다. 예를 들어 `"test_neutral": [1500,0]`은 형식 설명이며, 1500 µs도 장착 위치에 따라 중립이 아닐 수 있다.

실측해 등록한 프리셋 이름이 `disc_latch_open`일 때 시험 명령은 다음과 같다. 서보만 시험할 때 모터 PWM은 기본값 0으로 둔다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.hardware bench --config local_state/hardware.json --robot H1 --servo-preset disc_latch_open --duration-s 0.5 --enable-hardware --wheels-raised
```

H1은 `disc_latch_open`, `disc_latch_close`, 비버는 `gripper_open`, `gripper_close`, `hopper_gate_open_one` 등 실제 사용할 의도에 맞는 이름으로 프리셋을 등록한다. 비버 두 채널은 배열로 함께 지정하므로 뒤 게이트만 바꿀 때도 앞 집게를 어떻게 유지할지 실측한 값으로 명시한다. 아직 존재하지 않는 `arm_retract` 동작을 이름만 등록하여 성공처럼 넘기지 않는다.

네트워크 명령에 응답했다는 ACK는 수신 확인이다. 바퀴 회전, 집기 성공, 서보 도착을 확인한 센서값이 아니다.

## 6. CAD 숫자와 실제 주행 숫자를 구분한다

| 항목 | 햄스터 H1 v0.9 | 비버 H2/B1/B2 v0.7 | 사용 방법 |
|---|---:|---:|---|
| 바퀴 공칭 지름 | 43 mm | 43 mm | 타이어 눌림과 실제 원주를 측정 |
| CAD 바퀴 중심 간격 | 120.9 mm | 115.5 mm | 실물 좌우 접지 중심 간격을 측정 |
| CAD 축 중심 Y | −15 mm | −23 mm | 차체 CAD 원점과 구분 |
| 닫힘 기준 전체 외형 | 138.4 × 112 × 100 mm | 133 × 98.2875 × 94 mm | 태그 위치·적재물·열린 기구까지 포함해 확인 |

43 mm 바퀴가 실제로 100 RPM일 때 기하학적 속도는 약 225 mm/s다. 이 숫자는 무부하 공칭값으로 계산한 값이며, PWM→속도 곡선에 넣을 실측값이 아니다.

차동 주행의 위치 기준은 **좌우 바퀴 축의 중점**이다. AprilTag 중심과 이 점이 다르면 `robot_center_from_tag_mm`에 실측한 전방/좌측 오프셋을 넣는다. 전진축은 로봇 전방이며, CAD에서 전방은 +Y로 그려져 있어도 영상 좌표계의 각도 0은 +X다.

회전 충돌 반경은 축 중점에서 가장 멀리 나온 부분까지의 거리다. 차체 폭의 절반만 넣으면 앞 집게나 열린 뒤 게이트가 빠진다. 비버는 뒤 게이트를 열면 Y 외형이 약 144.92 mm까지 바뀐다. 적재물과 케이블도 포함해 측정한다.

배터리 CAD 참조는 이전 HJ/SCX24 가정인 56 × 28 × 13 mm이고 FMS 제품 표기는 40 × 22 × 14 mm다. 높이 파라미터의 1 mm 차이만으로 장착 불가를 단정하지 않는다. 실제 트레이·주변 모터·덮개 여유가 중요하다. 비버의 기존 배터리 위쪽 모터 여유는 작으므로 실물 시험 장착을 하고, 셀이나 전선이 눌리는 경우 공간을 수정한다.

## 7. 엔코더 없는 모터 보정

현재 엔코더가 없으므로 모터 PWM은 개방루프이고, 실제 로봇 위치·방향은 카메라로 보정한다. 전압·바닥·적재물에 따라 PWM 대비 속도가 달라진다.

각 로봇에서 `왼쪽 전진 / 왼쪽 후진 / 오른쪽 전진 / 오른쪽 후진` 네 곡선을 측정한다. 시험자가 관리하는 낮은 출력부터 여러 PWM 점을 사용하고, 동일한 배터리 상태와 적재 조건에서 태그 위치·방향의 시간 변화를 기록한다. 띄운 바퀴 시험은 극성과 회전 여부 확인용이고, 바닥 부하의 속도 보정을 대체하지 않는다.

극성 시험 후 바닥의 시험 구간을 비우고 한 대만 둔다. `--clear-test-lane`은 시험자가 빈 이동 구간을 확보했다는 명시적 확인이며, 카메라가 장애물을 감시한다는 뜻은 아니다. 바닥 보정 펄스는 1회 최대 0.5초이고 자동 장시간 반복은 하지 않는다. 아래 예시의 PWM도 실물에 맞춰 낮게 시작한다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.hardware bench --config local_state/hardware.json --robot H1 --left-pwm 30 --right-pwm 30 --duration-s 0.5 --enable-hardware --clear-test-lane
```

영상 또는 실측 표식으로 이동량과 회전량을 기록한다. 명령 시작/종료 전후의 가속·감속 구간을 구분하고, 일정한 속도 구간이 너무 짧아 값을 신뢰할 수 없으면 그 점을 보정값으로 넣지 않는다. 통신 유실 시 보드의 출력 만료는 최대 250 ms 범위지만 바퀴의 관성 이동까지 즉시 사라지지는 않는다. 이 시험에는 카메라 충돌 회피나 자동 저전압 보호가 없다.

영상으로 기록하려면 아래 8절의 카메라 보정을 먼저 마치고 첫 번째 PowerShell 창에서 `vision detect --track --report ...`를 실행한다. 두 번째 창에서 짧은 바닥 펄스를 실행한다. 검출 JSONL의 시간·축 중심 위치·방향으로 속도를 계산한다. 이때 카메라는 측정 기록용이며 bench 명령의 충돌 감시기가 아니다.

차체 전진 속도를 v, 반시계 방향 각속도를 ω, 실측 바퀴 중심 간격을 L이라 하면 왼쪽 바퀴 접선속도는 `v − ωL/2`, 오른쪽은 `v + ωL/2`다. mm와 초 단위를 통일하고 각속도는 rad/s로 계산한다. 각 방향의 기동 PWM 이하 영역과 정지점을 함께 기록한다. 지원하지 않는 속도를 높은 PWM으로 추정해서 채우지 않는다.

기록에서 좌우 속도 후보를 계산하는 도구도 포함되어 있다. `--start-s`와 `--end-s`는 영상 재생 초가 아니라 JSONL의 `captured_at_s` 값이다. 아래 바퀴 간격·시간·PWM은 형식 예시이며 본인이 측정한 값과 일정한 PWM이 적용된 구간으로 바꾼다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.hardware_calibration --log recordings/tags-first.jsonl --robot H1 --track-width-mm 120.9 --start-s 1234.50 --end-s 1234.90 --left-pwm 30 --right-pwm 30
```

최소 4프레임·0.15초 구간을 사용하며 재생 영상, 오래된 프레임, 역순·중복 시간, 로봇 누락, 큰 횡미끄러짐을 거부한다. 출력의 `candidate_curve_points`는 카메라로 추정한 후보일 뿐이다. `high_variability`가 참이거나 일정 속도 구간을 확보하지 못했으면 채택하지 않는다. 실제 PWM 적용·바퀴 미끄러짐을 이 로그만으로 증명할 수 없으므로 여러 번 반복해 검토한다. 도구는 프로필을 자동 수정하거나 `motion_calibrated`를 켜지 않는다.

곡선·극성·축 간격·반경·실제 정지 거리를 확인한 뒤에만 개인 설정의 `motion_calibrated`를 활성화한다. 서보의 열림/닫힘 펄스도 기구를 직접 보면서 보정한다. 처음부터 0°/180° 끝까지 움직이는 설정을 만들지 않는다.

## 8. 카메라와 AprilTag 보정

현재 실제 런타임은 고정한 카메라의 영상 좌표를 사용한다. 드론 시뮬레이션은 실제 비행 제어·이동 카메라 보정이 아니다. 움직이는 드론 영상으로 실물 주행을 켜는 기능은 준비되어 있지 않다.

```powershell
.\.venv\Scripts\python.exe tools/generate_robot_tags.py --config config/robot_tags.json --output-dir recordings/printable-tags
.\.venv\Scripts\python.exe -m robo_control.vision calibrate --camera 0 --output recordings/camera.json
.\.venv\Scripts\python.exe -m robo_control.vision detect --camera 0 --calibration recordings/camera.json --tags config/robot_tags.json --fleet config/qualifier_senior.json --track --preview --frames 300 --report recordings/tags-first.jsonl
```

태그의 검은 정사각형 변 길이를 실측하여 `tag_size_mm`에 mm로 넣는다. 흰 여백 전체 크기를 넣지 않는다. 인쇄용 PNG 옆의 앞방향 참고 이미지를 보고 부착 방향을 맞춘다. 태그를 돌려 붙였다면 `heading_offsets_rad`에 그 차이를 반영한다.

카메라 보정 창에서는 경기장 모서리를 좌상→우상→우하→좌하 순서로 찍는다. 기본 크기 1143 × 1181 mm는 저장소의 잠정 경기장 참조다. 실제 경기장 실측값이 다르면 `--field-size-mm 가로 세로`를 명시한다. 다른 해상도나 렌즈·줌·초점·카메라 위치로 바꾸면 기존 보정값을 재사용하지 않는다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision check --calibration recordings/camera.json --points local_state/check-points.json --max-error-mm 10
```

`check-points.json`에는 보정에 쓰지 않은 여러 지점의 `pixel_points`와 실측 `expected_mm` 배열을 넣는다. 위 10 mm는 시험 기준 예시이며 집기 오차 허용치에 맞춰 더 엄격하게 정한다. 태그가 차체 위로 올라가 있으므로 바닥 평면 보정만으로 높이 시차가 사라지는 것은 아니다. 실제 태그 높이에서 경기장 여러 위치를 재확인한다.

네 대 각각의 실측 태그 크기·앞방향·축 중심 오프셋을 확인한 뒤 태그 설정의 `hardware_verified`를 `true`로 바꾼다. 실제 출력 경로는 이 값, 태그 크기, 모든 로봇의 방향/축 중심 오프셋을 요구한다. 값이 0인 오프셋도 “확인해서 0”이어야 한다.

색 인식은 `--colors config/object_colors.json`을 추가한다. 조명에 맞는 HSV 범위와 물체 실측 크기를 확인한다. 같은 색 물체가 여러 개라는 이유만으로 특정 임무의 디스크·원기둥 ID를 자동 단정하지 않는다.

## 9. 실제 주행과 집기 임무의 완료 기준

실제 카메라 입력, 확인된 장치 프로필, 태그 등록, 실측 충돌 반경과 속도 곡선이 준비된 상태에서 낮은 속도의 한 점 이동을 먼저 수행한다. 원격 서버를 쓰더라도 카메라에서 명령까지의 지연 기준을 만족해야 하므로 첫 시험은 같은 로컬 네트워크의 컴퓨터에서 진행한다.

실측해서 검토한 목표 파일을 `local_state/goals.json`에 준비한 후 실행한다. 처음에는 한 로봇의 짧은 이동 목표만 넣고 다른 로봇은 정지시킨다. 설정에 등록된 네 대의 태그는 모두 보여야 한다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.hardware_runtime --camera 0 --calibration recordings/camera.json --tags config/robot_tags.json --fleet config/qualifier_senior.json --goals local_state/goals.json --hardware-config local_state/hardware.json --enable-hardware --report recordings/hardware-goal-001.jsonl --duration-s 10
```

목표 파일은 `config/runtime_goals.example.json`을 참고해 만든다. `radii_mm`를 넣는 경우 하드웨어 프로필의 실측 반경과 일치시킨다. 예제의 `null`은 실측값으로 채워야 하며 시뮬레이터의 좌표를 그대로 실물에 적용하지 않는다. JSONL 파일은 기존 경로를 덮어쓰지 않으므로 시험마다 새 이름을 쓴다.

기존 임무의 실측 접근점·투하점·후퇴점을 검토한 경우에는 `--goals` 대신 `--mission local_state/mission.json`을 사용한다. 파일 형식은 `config/runtime_mission.example.json`을 참고한다. 물체 인식과 연결을 사용할 때는 `--colors`, 검토한 `--bindings`를 추가한다. 아래 표의 미측정 조작 단계는 그대로 대기하므로 전체 경기 자동 완주 명령으로 읽지 않는다.

실행 시 `--enable-hardware`를 명시한다. 영상 파일 재생과 fake-wire 시험은 실물 출력을 대신하는 검증 경로이며, 실제 출력의 근거로 사용하지 않는다. 목표점 좌표는 로봇 축 중심의 도착점이다. 물체 중심 좌표를 로봇 도착점으로 그대로 쓰면 집게 오프셋이 빠진다.

기존 임무 로직이 요구하는 확인과 현재 장착 부품의 차이는 다음과 같다.

| 단계 | 기존 논리 신호 | 현재 장착 부품으로 확인 가능한 범위 |
|---|---|---|
| 닫기 / 열기 | `servo_closed`, `servo_open` | MG90S 명령 송신만으로 실제 도착 확인 불가 |
| 디스크 보유 / 통로 비움 | `optical_present`, `optical_clear` | H1 TCRT5000 장착 위치·극성·검출 안정화 후 확인 가능 |
| 원기둥 보유 / 비움 | `gripper_present`, `gripper_clear` | 비버에는 해당 접촉·광센서가 없음 |
| 수납 | `arm_retracted` | 현 CAD에 독립된 수납 액추에이터가 없음; `arm_retract` 명령을 임의의 서보 동작으로 대응시키지 않음 |
| 의료키트 공급 / 배출 통로 | `hopper_loaded`, `hopper_clear` | 현재 비버에는 이를 직접 측정하는 센서가 없음 |
| 목적지 안착 | `object_released`, `object_settled`, 물체 ID·위치·각도 | 같은 물체의 실제 영상 관측과 영역 판정 필요 |

현재 구매한 부품만으로도 주행·서보 출력·H1 센서 읽기 코드를 실행하고 시험할 수 있다. 그러나 신호가 없는 집기 단계를 시간 경과나 ACK만으로 성공 처리하지 않는다. 실제 임무는 그런 확인 단계에서 대기하거나 오류로 정지한다. 누락된 논리 신호는 측정 가능한 센서 또는 검증한 영상 판단으로 연결해야 한다.

특히 기존 `hopper_gate_open_one`은 “의료키트 한 개만 배출” 의도다. 비버 CAD는 두 상자를 쌓아 한 뒤 게이트로 내보내므로 한 개씩 분리 배출이 자동 보장되지 않는다. B2의 두 개를 같은 목적지로 함께 내보내려면 실물 배출량과 착지 위치를 확인한 뒤 배치 작업의 완료 조건도 맞춰야 한다. 한 번 게이트를 열고 두 개의 개별 임무를 모두 성공으로 기록하지 않는다.

태그 유실·프레임 지연·통신 유실·120초 종료 시 정지 동작은 실제 바퀴를 띄운 시험과 저속 주행 시험에서 확인한다. 오류 후에는 해당 로봇과 물체 상태를 확인하고 새 실행으로 시작한다. 조작 결과가 불명확한 상태에서 집게·투하 명령을 자동 재전송하지 않는다.

펌웨어의 모터 정지는 입력을 모두 LOW로 두는 코스트 방식이다. 정지 명령이 기계적 제동 거리를 보장하지 않는다. 정지·오류 시 서보 펄스도 해제되므로 물체를 계속 쥐고 있는다고 가정하지 않는다. 비상 정지가 래치된 보드는 원인을 확인한 뒤 재부팅해야 한다.

## 10. 확인 자료와 남겨야 할 기록

- 각 보드의 내부 ID, 펌웨어 버전, IP와 실제 핀 대응.
- 각 모터 극성, 네 방향 속도 곡선, 사용한 배터리 전압과 적재 조건.
- 서보별 기구가 간섭 없이 동작하는 최소/최대 펄스와 열림/닫힘 값.
- 카메라 보정 파일, 태그 치수·오프셋, 독립 기준점 오차.
- 실물 차체 반경, 게이트 개방 반경, 실제 정지 거리.
- 실행 JSONL과 오류 발생 직전 영상. 실제 인증 토큰은 기록하지 않는다.

이전 개념 문서에는 엔코더 미정, 실제 I/O 미연결, 메카넘 휠 배열 등의 과거 상태가 남아 있다. 새 하드웨어 경로를 사용할 때는 이 안내와 현재 `hardware --help`, 펌웨어 설정, 개인 장치 설정을 우선한다. 과거 가짜 장치·시뮬레이터의 성공 결과를 실제 장치 검증 결과로 읽지 않는다.
