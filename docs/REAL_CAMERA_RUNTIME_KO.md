# 실제 카메라 → 측정 좌표 → 목표 속도 실행 안내

> 후속: [렌즈 보정](LENS_CALIBRATION_KO.md)을 사용하는 field schema 2도 지원한다.
> 렌즈 값은 실행 설정에 내장 복사되며 검출 행에 적용 여부/식별자를 기록한다.

기준: 2026-09-07, `codex/real-camera-runtime` (팀원 `1613d57`에서 시작, `7a1c37d` 추가 반영).

`python -m robo_control.runtime`은 고정 USB 카메라 또는 로컬 영상 파일을 기존
AprilTag 검출·연속 추적·목표 속도 계산에 연결한다. **로봇 위치를 생성하거나 명령
속도로 가상 위치를 갱신하지 않는다.** 모터/서보 드라이버는 아직 없으며 결과는
`MockActuatorBank` 메모리와 JSONL에만 남는다.

## 실물용 코드와 모의 시험의 구분

| 구성 | 현재 동작 | 검증 경계 |
|---|---|---|
| USB/영상 디코딩, 보정, AprilTag, HSV | 실제 영상 픽셀 처리 | 자동 영상 시험; 현장 정확도는 미측정 |
| `PoseTracker`, `ObjectTracker` | 검출값의 시간·ID·누락 검사 | 합성 관측 및 영상 회귀시험 |
| `ClosedLoopController` | 측정 위치와 명시 목표의 오차로 속도 계산 | 이상적 휠/제동 가정; 실물 구동 보장 아님 |
| 새 `runtime` | 영상 프로세스와 중앙 감시 루프로 위 모듈 연결 | 파일 디코딩·고장 주입·연속 실행 |
| `MockActuatorBank` | 명령 유효성·만료 검사, 메모리에 속도값 저장 | 모터·펌웨어 아님 |
| `qualifier`, `control_loop --demo` | 합성 센서/이상적 이동으로 상태기계 시험 | 실물 임무 완수 증거 아님 |
| 웹 동선·드론 비교 | 가상 위치·기구·가림으로 시간/점수 계산 | 최신 45.18초/160점, 이전 47.06초 모두 시뮬레이션 수치 |

현재 H1은 햄스터, H2/B1/B2는 비버다. 태그 0/1/2/3 매핑은 유지하며 H2가 세 번째
비버(B3)다. 역할은 ID 접두사가 아니라 fleet 설정으로 읽는다.

## 고정 카메라 실행

저장소 루트, Python 3.11 이상. 먼저 [카메라 보정](CAMERA_CALIBRATION_KO.md)과
[태그 장착](APRILTAG_TRACKING_KO.md) 절차를 따른다.

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[vision]"
.\.venv\Scripts\python.exe -m robo_control.vision calibrate --camera 0 --output recordings/camera.json
.\.venv\Scripts\python.exe -m robo_control.runtime --camera 0 --calibration recordings/camera.json --tags config/robot_tags.json --fleet config/qualifier_senior.json --report recordings/runtime-camera-001.jsonl --duration-s 120
```

- `--goals` 생략 시 모든 로봇의 속도는 0이다. 우선 이 상태로 4대 관측을 확인한다.
- `--report`는 **새 파일**이어야 한다. 기존 기록/설정/영상을 덮어쓰지 않는다.
- GUI는 없다. 장착 확인은 기존 `vision detect --preview --track`을 사용한다.
  독점 카메라 드라이버라면 미리보기를 종료한 뒤 runtime을 실행한다.
- 한 세션은 고정 카메라 한 대만 지원한다. `--moving-camera`는 거부한다.
  위치·높이·해상도가 바뀌면 종료하고 다시 보정한다.
- Ctrl+C, 시간 만료, 영상 프로세스 종료/오류는 최종 0 명령으로 세션을 닫는다.
  자동 재연결·자동 재출발하지 않으므로 원인을 확인하고 새로 실행한다.
- 프로그램은 USB 영상 입력 외에 웹사이트/모터/조종기/UDP를 열지 않는다.

녹화 파일은 카메라 대신 `--video recordings/match.avi`를 지정한다. 파일의 명목 FPS로
재생 간격을 맞추며, 미디어 시각은 속도 계산에, 호스트 단조 시각은 만료 검사에 쓴다.
FPS가 없으면 거부한다. 정상 EOF는 디코딩 프레임 수가 파일의 명목 프레임 수 이상일 때만
기록한다. 프레임 수를 확인할 수 없거나 중간에 읽기가 끝나면
`video_decode_failed_or_unverified_end`로 실패 종료한다. 메타데이터 일치는 무결성 인증이 아니다.

## 목표 계산 시험

`config/runtime_goals.example.json`은 **빈 목표 예제**다. 그대로 사용해도 이동 목표를
만들지 않는다. 현장 좌표와 회전 포락 반경을 검토한 별도 JSON의 형식은 다음과 같다.

```json
{
  "schema_version": 1,
  "coordinate_system": "bottom_left_x_right_y_up_mm",
  "goals": {"H1": {"x_mm": 500.0, "y_mm": 600.0, "heading_rad": 0.0}}
}
```

위 좌표는 형식 예시이지 실제 작업점이 아니다. 단위는 mm/rad, 원점은 좌하단,
+X는 오른쪽, +Y는 위쪽, 양의 회전은 반시계다. 목표 없는 로봇은 정지한다.
반경+여유가 외벽 밖으로 나가는 목표, 미등록 ID, 비정상 수치는 실행 전에 거부한다.

`radii_mm`를 추가하면 **등록된 모든 로봇**의 차체·집게·적재물·회전 범위를 포함한
반경이 필요하다. 기본 150mm는 기존 제어 코어의 모의 가정이다. 시험 영상 생성기의
40mm 반경은 태그 시험값이므로 실물 설정에 복사하지 않는다. 최대 속도 180mm/s,
메카넘 치수, 제동감속도 역시 미실측이다. 검증 결과를 얻기 위해 임의로 줄이면 안 된다.

runtime은 명시 지점의 목표 속도를 계산할 뿐 A* 경로, 의료 물체 집기/해제, 임무 순서,
장애물 우회 경로를 실행하지 않는다. 위험 시 우회하지 않고 전부 정지한다.
`--colors`에 검토한 HSV 프로파일을 주면 기존 물체 추적을 기록하지만, 자동 집기나
경로 장애물 지도에는 아직 연결되지 않는다.

## 실행 구조와 정지 기준

```text
영상 프로세스: USB/영상 → 프레임 검사 → 태그/색 검출
                              ↓ 최신 JSON 1개 (공유 메모리)
중앙 50Hz 루프: 출처/설정 검사 → 기존 추적 → 목표 속도 계산 → 검증용 명령 저장
                  ↓ 관측 만료·프로세스 감시                   ↓
                  전체 0 명령                        비동기 JSONL 기록
```

| 조건 | 동작 |
|---|---|
| 초기/손실 후 재관측 | 기본 3개 연속 유효 프레임 전까지 0 |
| 새 프레임 누락/중복/미등록/잘못된 좌표·출처·시각 | 해당 tick에서 전부 0, 재확인 시작 |
| 새 프레임이 전혀 도착하지 않음 | 마지막 유효 관측 획득 시각+200ms부터 첫 감시 tick에서 0 |
| 새 명령이 없음 | 발행 시각+300ms 만료; 빈 poll은 TTL을 갱신하지 않음 |
| 감시 tick 간격/계산 지연 100ms 초과 | 세션 종료 잠금; 뒤늦은 정상 관측으로 재개 불가 |
| 프로세스 죽음/관측원 종료 | 최종 0, bounded join 후 필요하면 영상 프로세스 종료 |
| 기록 장치 지연/오류/큐 포화 | 디스크 쓰기로 제어 tick을 막지 않고 실패 종료 |
| 로봇 간/네 외벽 예측 차체·제동 영역 침범 | 전체 0; 경계/충돌 원인 기록 |

카메라 `read()`/검출이 멈춰도 중앙 루프는 다른 프로세스에서 돈다. 기본 256KiB 공유
메모리는 최신 레코드 한 개만 보존한다. 양쪽이 잠금을 기다리지 않으므로 생산자가 잠금
도중 죽어도 중앙 감시는 멈추지 않는다. JSONL 큐도 기본 128행으로 제한한다.
출력값을 먼저 0으로 만든 뒤 영상 정리는 최대 약 1.5초, 기록 정리는 최대 2초 시도한다.
기록 실패로 파일에 최종 행이 없어도 CLI 요약에는 최종 0 상태와 실패가 남는다.

**200/300/100ms는 호스트 소프트웨어 기준이다.** Windows/Python은 실시간 OS가 아니므로
물리 정지 지연 상한을 보장하지 않는다. PC가 죽으면 Python watchdog도 실행되지 않는다.
OpenCV 호스트 읽기 시작 시각은 센서 노출 시각이 아니므로 장치 내부 버퍼의 오래된 영상은
검출하지 못할 수 있다. 펌웨어 watchdog·물리 비상정지와 노출/버퍼 지연 측정 없이는
실제 모터를 연결하면 안 된다.

## 기록 읽기

```powershell
.\.venv\Scripts\python.exe -m robo_control.runtime_report recordings/runtime-camera-001.jsonl
```

첫 `session_started` 행에 태그/보정/역할/목표/반경/한계가 저장된다. 영상 프로세스에는
검증한 설정 복사본을 넘기므로 실행 중 JSON 파일 수정이 세션 설정을 바꾸지 않는다.
`configuration_id`는 영상 설정/입력의 SHA-256 식별자이지 보안 인증·실측 합격값이 아니다.
`observation`은 새 프레임이 있을 때만 존재한다. `null`은 빈 poll이며 `command=null`은
명령을 재발행하지 않았다는 뜻이다. `actuator`는 현재 검증용 출력 상태다.

분석기는 세션/모드 혼합, 시간 역행, 중복 시퀀스, 잘린 JSONL, 출력 등록 누락,
최종 비영점 출력을 거부하고 호스트 관측 나이/tick 간격/정지 사유를 집계한다.
`log_complete=true`는 기록 구조와 최종 0 행 확인이지 전체 이미지 기록이나 물리 정지
확인이 아니다. 최신값 교체 수는 실행 요약의 `replaced_frames`에 남는다. 잠금 경합으로
발행 자체가 생략된 프레임까지 세지는 않는다.

- `input_mode`: `live_camera` 또는 `video_replay`; `output_mode`: 항상 `dry_run_commands`.
- `device_io`, `hardware_ready`, `motion_permitted`: 항상 false.
- 종료 코드 0: 관측 기록이 있고 정상 시간/Ctrl+C/확인된 EOF로 종료, 기록/프로세스 정리 완료.
  0이어도 4대 추적 성공을 뜻하지 않으므로 `usable_observation_frames`를 확인한다.
- 종료 코드 1: 실행/관측원/기록 실패. 2: 설정/초기화 실패.

## 자동 시험과 팀의 다음 작업

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_runtime*.py" -v
.\.venv\Scripts\python.exe tools/verify_camera_runtime.py --output-dir recordings/runtime-check-001 --seconds 120
```

생성기는 4개 정지 AprilTag 영상에서 H1을 10초마다 0.6초 가린다. 실제 파일 디코딩부터
검출/추적/목표 계산/기록까지 실행하며, 비영점 명령의 신선도·재확인 조건과 추적 위치가
해당 프레임의 검출값인지 전 행 검사한다. `--seconds 600`으로 10분 시험도 가능하다.
`verification.json`과 영상/로그는 Git 제외 경로 `recordings/`에 보존한다. 생성 영상은
현장 카메라 정확도·성능이나 실제 주행 검증으로 발표하면 안 된다.

다음은 현장 카메라/태그/렌즈/조명/버퍼 지연 측정, 보드/모터/엔코더/휠/집게 사양 확정,
펌웨어 로컬 시계·명령 만료·물리 비상정지와 벤치 시험 순서다. 그다음 관측-임무/경로
엔진과 실제 통신을 연결한다. 기존 UDP 어댑터는 고수준 JSON 스캐폴드이므로 runtime
검증용 packet을 그대로 모터에 보내지 않는다. 두 카메라 융합, 이동 카메라 동적 보정,
집기/해제 센서 확인은 별도 작업이다. [팀 작업 기록](REAL_RUNTIME_WORK_KO.md)을 함께 읽는다.
