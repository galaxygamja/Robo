# 관측·임무·통신 runtime 통합

2026-09-07. 기존 관측/임무/경로/차동 제어기와 `robo-wire v1`을 하나의 감시 루프에 연결했다.
**실제 장치 전송이 아니라 4대 가짜 수신기 통합 모드**다. 모터·센서·무선·직렬 연결은 없다.

## 실행과 기본 동작

기존 runtime은 기본 설정을 유지한다. 새 모드는 `--mission`과 `--wire-fake`를 함께 명시한다.
메카넘 기본 `--goals` 출력에는 적용할 수 없다. 아래 입력 파일은 실제로 준비한 파일로 대체한다.
`runtime_mission.example.json`의 null 값은 접근 자세/반경 등을 검토한 값으로 채워야 한다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.runtime --video recordings/match.avi --calibration recordings/camera.json --fleet config/qualifier_senior.json --mission recordings/mission-reviewed.json --wire-fake --report recordings/wire-runtime-001.jsonl --duration-s 120
.\.venv\Scripts\python.exe -m robo_control.runtime_report recordings/wire-runtime-001.jsonl
```

고정 카메라로 관측하려면 `--video ...`를 `--camera 0`으로 바꾼다. 카메라 입력을 쓰더라도
출력은 가짜 수신기에만 전달된다. 보고서는 기존 파일을 덮어쓰지 않는 새 경로여야 한다.
센서 입력 없이 집기/배출이 완료되지는 않으며, 센서 확인 단계에서는 제한시간 뒤 종료한다.

연결 순서:

```text
카메라/영상 → 기존 검출·추적 → WorldState → 임무·경로 → 차동 차체 명령
                                                            ↓
                    기존 로컬 출력 검사 ← runtime 감시 → 4대 wire 송신기
                                                            ↓ JSON 바이트
                     4대 수신기 모델 ← hello/arm/drive/stop → 응답 검사
                                                            ↓
                                      전체 임무 보류 / 상태·종료 로그
```

로컬 `MockActuatorBank`는 제어 출력 검사를 위해 유지한다. 실제로 전송된 명령의 확인 여부는
별도 `wire` 기록을 봐야 한다. 로컬 검사 출력이 0이라는 이유로 원격 수신을 확인했다고 하지 않는다.

## 기동·대기·오류 규칙

| 상태 | 처리 |
|---|---|
| 카메라 기동/초기 관측 대기 | 수신기 연결/arm을 시작하지 않고 로컬 출력 0 |
| 정상 관측 3프레임 | 최초 hello→arm 실행, 기동 tick 자체는 출력 0 |
| 연결 완료 뒤 새 프레임 | 새 차동 명령을 4대 각각에 전송 |
| 한 로봇만 이동 | 나머지 3대도 새 관측에 근거한 0속도 명령으로 허가 유지 |
| 정상 도착/조작 대기 | 0속도 유지 가능. ACK를 조작 완료 신호로 사용하지 않음 |
| 정상 ACK 대기 | 새 명령을 쌓거나 이전 명령을 재전송하지 않음. 이것만으로 임무 오류 처리하지 않음 |
| 어느 한 대의 통신 오류 | 전체 통신 허가 잠금, 로컬 출력 0, 경로 폐기, 모든 로봇에 최선의 정지 요청 |
| 기동 후 관측 오류·200ms 만료 | 전체 보류/정지 요청. 정상 영상만 돌아왔다고 자동 재가동하지 않음 |
| EOF/기간 종료/프로세스·기록 오류/운영자 종료 | 로컬 출력 즉시 0, 전체 정지 요청, 한 개의 최종 closed 기록 |

runtime은 감시 지연·종료·관측 만료와 새 관측의 유효성을 먼저 확인해 잘못된 tick의
대기 중 이동 명령을 폐기한다. 그 뒤 `RuntimeWireSupervisor`가 응답을 처리하고 장치 시계의
만료를 검사하며, 새 제어 패킷은 이후 계산한다. 응답 전에 만든 패킷을 버퍼에서 꺼내 다시 보내지 않는다.
송신 대기 때문에 건너뛴 명령은 다음 새 관측의 새 명령으로 대체한다.

실제 오류인 hold/정지/충돌/관측 유실은 0속도 유지 예외로 우회할 수 없다.
이 차이는 `WireCommandSender.drive_from_packet(..., keep_armed_zero=True)`로 명시하며,
기존 송신기의 기본 동작은 바꾸지 않았다.

## 명시적인 재연결

CLI는 통신 오류 후 자동 재연결하지 않는다. 원인과 로봇/임무 상태를 확인하고 현재 실행을
종료한 뒤 **검토한 새 임무 계획과 새 보고서로 다시 실행**한다. 이미 조작했을 수 있는
임무를 무조건 처음부터 재생하지 않는다. 실제 장치 상태를 확인하는 작업은 이 모드 밖이다.

프로그램에서 아직 열린 이동 단계 세션은 다음 API를 사용할 수 있다:

```python
event = session.reconnect_wire(host_now, operator_confirmed=True)
```

- 기존 통신 오류 잠금과 명시 확인이 모두 있어야 한다. 끊긴 모델 링크를 먼저 복구해야 한다.
- 오래된 큐/연결 세대/decoder는 폐기하지만 송신기의 원본 명령 순서 기록은 보존한다.
- 연결 시 출력은 0이다. 이전 WorldState를 무효화하고 **새 정상 관측 3프레임**을 확인한 뒤 진행한다.
- 종료된 runtime, 바뀐 관측 세션/보정으로 닫힌 WorldState, 조작 중 오류가 난 임무는 재개할 수 없다.
  이 경우 새 세션과 검토한 계획이 필요하다.
- 조작 중 통신 오류는 `manipulation_interrupted`로 종료한다. 집기/배출이 이미 일어났는지
  알 수 없으므로 같은 동작을 자동 재발급하지 않는다.
- 보류 중에도 기존 전체 임무/단계 제한시간은 흐른다. 통신 복구가 경기 시간을 초기화하지 않는다.

## 로그와 ‘정지 확인 불명’

시작 행 및 각 tick의 `transport_mode`는 `mock` 또는 `fake_wire`로 고정한다.
기존 `output_mode=dry_run_commands`는 유지한다. 가짜 수신기 모드는 각 tick에 다음을 추가한다:

- 전체 통신 상태 `idle/connecting/ready/pending/fault/closed`, 최초 오류, 큐 길이와 누적 카운터.
- 로봇별 sender/receiver 상태, 연결 여부, `stop_requested`, `stop_acknowledged`.
- `unconfirmed_stop_robot_ids`: 정지를 요청했지만 해당 연결·명령의 정상 ACK가 없는 로봇.
- 수신기 모델에 남은 목표 속도. 없는 센서나 실제 속도를 만들지 않는다.

정지 패킷 또는 ACK도 유실될 수 있다. 이때 종료를 기다리며 감시 루프를 막지 않는다.
최종 로컬 출력은 0이어도 가짜 수신기의 마지막 목표값은 TTL까지 남을 수 있으며, 그대로 기록한다.
직접 가짜 수신기의 정지 메서드를 호출해서 끊긴 통신을 성공한 것처럼 만들지 않는다.

보고서 분석 결과:

| 필드 | 뜻 |
|---|---|
| `final_stop_recorded` | 기존 로컬 검사 출력의 최종 0 기록 |
| `final_wire_zero` | 가짜 수신기 모델에 기록된 최종 목표값이 모두 0 |
| `final_wire_stop_acknowledged` | 모든 로봇의 정지 ACK를 일치하는 연결/순서로 확인 |
| `unconfirmed_stop_robot_ids` | 최종 정지 응답 미확인 로봇 |
| `wire_fault_ticks`, `wire_fault_reasons`, `wire_counters` | 통신 오류와 송수신/유실 진단 |
| `physical_stop_verified` | 항상 false. 위 기록은 실물 정지 증명이 아님 |

분석기는 유효한 ‘정지 응답 불명’ 로그도 읽을 수 있다. 따라서 분석 명령 종료 코드 0은
파일 구조가 유효하다는 뜻이지 모든 수신기의 정지 성공이 아니다. 반면 runtime 실행 명령은
통신 오류가 남거나 최종 정지 ACK가 불명인 경우 성공 종료 코드 0을 반환하지 않는다.
기존 mock 보고서도 계속 읽는다. 세션/모드 혼합, 로봇 누락/중복, 잘못된 허가/확인 주장은 거부한다.

## 시험과 하드웨어 경계

`wire_options` / `configure_link()`는 Python 시험용 장애 주입 API다. 로봇별 요청/응답 지연,
다음 N개 패킷 유실·중복, 연결 끊김과 잘못된 응답을 시험하며 실제 네트워크 설정이 아니다.
큐는 최대 256개 바이트 이벤트, poll은 최대 512개 이벤트 처리로 제한한다.

장치마다 시계 원점은 다르지만 watchdog은 **동일 Python 프로세스에서 poll하는 모델**이다.
그 프로세스가 멈췄을 때 실제 모터가 정지한다는 보장이 아니다. 실제 매체/인증·무결성,
독립 펌웨어 타이머·모터 enable·물리 비상정지, 집게/센서 프로토콜은 여전히 별도 작업이다.
바퀴 치수·PWM·엔코더·모터 극성은 추정하지 않았다.

검증 명령:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p 'test_runtime_wire*.py' -v
.\.venv\Scripts\python.exe -m unittest discover -s tests -q
```

전용 시험은 생성 영상 디코딩→관측→임무→송수신→보고서, 한 대 이동/세 대 대기,
정상 ACK 지연, 유실/손상/중복 응답의 전체 잠금, 재연결, 센서 단계 중단,
종료 시 정지 응답 불명과 기존 보고서 호환을 포함한다. 새 프레임이 이전 관측의
정확한 200ms 만료 시점에 도착하는 경우, 감독 주기 초과/잘못된 관측 시 대기 중
명령의 선행 적용 방지, 고장 난 통신 시계에서도 종료 기록을 남기는 경우도 검사한다.

2026-09-07 최종 로컬 검증:

| 검증 | 결과 |
|---|---|
| 전체 Python 회귀 시험, OpenCV 5 | **556개 통과** |
| 이번 추가 시험 | **68개**: 통신 감독 30, 보고서 19, runtime/영상 통합 19 |
| OpenCV 4 환경의 추가 시험 | 68개 통과 |
| OpenCV 없는 기본 Python의 추가 시험 | 68개 실행, 67개 통과·영상 시험 1개 건너뜀 |
| 변경 Python 파일 Ruff / diff 검사 / runtime 도움말 | 통과 |

기존 CI의 unittest 검색에 새 시험이 자동 포함된다. 위 결과는 로컬 실행이며 원격 CI의
결과가 아니다. 실제 카메라·네트워크·장치·물리 정지 검증은 이번에 수행하지 않았다.
