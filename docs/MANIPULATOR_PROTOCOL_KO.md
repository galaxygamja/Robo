# 조작 명령·센서 응답 논리 규격 초안과 가짜 장치 시험

2026-09-08. 하드웨어 없이 가능한 우선순위 1 작업이다. **보드·배선·센서 선정 또는
실물 운용 승인이 아니다.** 주행 `robo-wire` v1과 별도인 `robo-manipulator-draft` v1을
추가했다. 기존 주행/임무 실행, 팀원 CAD·웹 동선·장치 사양은 변경하지 않는다.

## 완료한 코드와 재현

- `robo_control/manipulator_wire.py`: 엄격한 논리 규격, 임무 의도 복사 함수, 로봇별 송신 상태기계.
- `robo_control/fake_manipulator.py`: 수신기 시계 기반 허가/만료, 중복 실행 방지, 명시적 시험 센서 입력.
- `robo_control/manipulator_lab.py`: H1/H2/B1/B2 바이트 왕복과 통신 오류 재현. H2도 비버 역할이다.
- `tests/test_manipulator_wire.py`, `tests/test_manipulator_lab.py`: 전용 회귀와 기존 임무의 합성 피드백 거부 시험.

```powershell
.\.venv\Scripts\python.exe -m robo_control.manipulator_lab --compact
.\.venv\Scripts\python.exe -m unittest discover -s tests -p 'test_manipulator*.py' -v
```

lab은 메모리 바이트만 사용한다. 장치별 다른 시계 원점, ACK 유실, 센서 응답 지연,
중복 실행, 취소·정지와 최종 논리 동작 제거를 13개 조건으로 검사한다. 로봇 위치 적분,
경기 동선 시뮬레이션, 득점 또는 실제 집기 성공을 생성하지 않는다.
`synthetic=true`, `device_io=false`, `hardware_ready=false`, `physical_success=null`,
`physical_safe_state="unconfigured"`, `mission_feedback_eligible=false`를 유지한다.

## 1. 메시지와 식별

기존 `wire_codec`의 UTF-8 JSON 객체 + LF, 4096바이트 상한, 중복 키/NaN/불완전 프레임
거부를 재사용한다. 주행 수신기는 이 별도 프로토콜을 거부한다. 두 프로토콜을 같은 물리
매체로 다중화하는 규격은 아직 없으며 기존 `--wire-fake`에 자동 연결하지 않았다.

모든 요청은 아래 **정확한 필드 집합**을 사용한다. 모르는 필드는 거부한다.

```text
protocol="robo-manipulator-draft", version=1, type,
robot_id, host_session_id, hello_id, boot_id, link_id,
seq, permit_id, operation
```

`operation`은 `command_id`, `phase`, `action`, `timeout_ms` 네 필드다. 임무의
`session:index:phase_serial` 명령 ID를 보존하며, ID는 ASCII 영숫자/밑줄/콜론/하이픈
1..128자다. 로봇 ID는 기존 fleet 형식이다. seq는 정수 1..2^53-1이며 bool은 금지한다.
`timeout_ms`는 명시적 정수 1..4000으로, 시험 계약의 상한이지 검증한 실물 작동 시간은 아니다.

| type | 의미 |
|---|---|
| hello | 한 조작의 신선한 허가 요청. boot/link/permit은 null. 기존 논리 동작 제거 |
| execute | hello ACK의 boot/link/단일 사용 permit과 동일 operation을 사용해 한 번 접수 |
| sample | 해당 operation의 명시적 시험 센서 표본 조회. permit=null, 만료 연장 없음 |
| cancel / stop | 현재 연결의 논리 조작 중단. permit=null. 이전 seq도 정지를 우선 |
| estop | stop과 함께 수신기 비상정지 잠금. 원격 해제 명령 없음 |

명령별 seq는 증가한다. 로봇/호스트 세션/hello/부팅/연결/operation을 모두 대조한다.
한 로봇에는 응답 대기 요청 하나만 허용한다. cancel/stop/estop은 대기 요청을 대체할 수 있다.
4대는 각자 독립된 endpoint이며, 분산 원자성이나 전체 로봇 자동 정지를 제공하는 API는 아니다.

## 2. 기존 임무 의도와 대응

| 임무 phase | 허용 action |
|---|---|
| close_servo | disc_latch_close / gripper_close |
| confirm_grip, confirm_load, confirm_clear | hold — 센서 확인 구간의 논리 의도 |
| retract | arm_retract |
| release_servo | disc_latch_open / gripper_open / hopper_gate_open_one |

`operation_from_intent(intent, timeout_ms=...)`는 현재 `MissionExecutor.snapshot()`의
장치 출력·dispatch가 꺼진 정상 의도를 복사한다. navigate/approach/carry/retreat는 거부한다.
호출자가 원래 intent의 robot_id/session_id에 맞는 sender를 선택해야 한다.
수신기 생성 시 `supported_actions`를 명시해야 하며 지원하지 않는 동작은 거부한다.
기구별 자동 capability 추정은 하지 않는다.

`hopper_gate_open_one`은 기존 논리 이름일 뿐, 정확히 한 개가 배출됐다는 센서 근거가 아니다.
`hold` 역시 모터 전원 유지나 안전한 파지 보장을 뜻하지 않는다. 실제 PWM/각도/핀 번호,
힘/전류 제한, 취소 시 잡고 있기 또는 열기 선택을 임의로 정하지 않았다.

## 3. 신선도, 만료, 재시도

1. hello를 받은 수신기 시각에서 **300ms 이내**의 단일 사용 permit을 발급한다.
2. execute 도착 시에도 수신기 자체 시계로 검사한다. 늦게 도착했다고 허가 수명을 늘리지 않는다.
3. 수신기 조작 deadline은 **permit 발급 시각 + timeout_ms**다. execute/ACK/sample 시각으로
   갱신하지 않는다. 호스트 deadline은 begin 시각 + timeout_ms로 더 보수적으로 감시한다.
4. 호스트 ACK 대기는 **300ms 미만**, 센서 표본의 보수적 나이는 **200ms 미만**이다.
   경계값에서 이미 만료다. 각 clock은 유한·비음수·단조여야 하며 시계 오류는 잠긴다.
5. 호스트 `poll()`과 수신기 `tick()`을 입력이 없을 때도 각각 호출해야 한다. Python 호출이
   멈추면 하드웨어도 정지한다는 보장은 없다. 실물 독립 watchdog은 펌웨어 작업이다.

ACK 유실/손상/식별 불일치는 **실행 결과 불명**이다. 자동 재송신·재연결·성공 처리는 없다.
호스트는 begin한 명령 ID를 해당 sender 수명 동안, 수신기는 실행한 (host_session_id, command_id)를
해당 부팅 동안 기억한다. 두 이력은 각각 최대 4096개이고 가득 차면 거부한다. 오래된 항목을
버려 중복 실행을 허용하지 않는다. 새 hello로도 동일 부팅의 실행 ID를 재실행하지 못한다.

**재부팅·호스트 프로세스 교체·새 session/command ID를 가로지르는 영구 exactly-once 보장은 없다.**
결과 불명인 집기/배출을 새 ID로 다시 제출해서는 안 된다. 운영자 확인 또는 후속 영속 작업장부와
실제 센서 근거에 의한 복구 규격이 필요하다. `begin()`은 명시적 호출 API이지 복구 승인 기능이 아니다.

cancel/stop은 이미 벌어진 물리 동작을 되돌리지 않는다. 정상 stop ACK도 **논리 중단 접수**만 뜻한다.
hello ACK를 잃은 상태에서는 boot/link를 모르므로 sender.stop은 전송할 메시지 없이 로컬 종료한다.
이 경우 execute를 만들 수 없고 수신기의 미사용 offer만 만료된다. execute 대기 중 취소는
확인된 문맥으로 전송하며, 뒤늦은 execute가 도착해도 허가는 폐기되어 있다.

호스트가 오류로 닫혀도 원격으로 stop이 자동 송신되는 것은 아니다. 호출자는 가능한 문맥으로
`sender.stop(now)`을 최선의 노력으로 전달하고 수신기 watchdog도 계속 감시해야 한다.
프레임 스트림 오류는 decoder 종료와 endpoint.disconnect 및 같은 정지 경로로 처리해야 한다.
현재 lab/단위시험 바깥의 runtime 전체 정지 연동과 실제 어댑터는 아직 없다.

## 4. ACK와 센서 근거 분리

응답은 요청 필드에 다음 필드를 추가한다. `type=response`, `request_type=원래 type`이고,
hello는 새 boot/link/permit을 반환한다. 이후 permit은 null이다.

```text
result, reason, state, signals, sample_sequence, sample_age_ms,
synthetic=true, device_io=false, hardware_ready=false,
execution="unknown", physical_success=null
```

상태는 offered/active/closed, 결과는 accepted/rejected다. 정상 reason은 ok다.
**ACK의 signals/sample_sequence/sample_age_ms는 반드시 null**이다. sample 응답에만
명시적으로 주입한 표본이 있거나 전체 null(표본 없음)이 온다.

논리 센서 키는 servo_closed, servo_open, optical_present, gripper_present,
arm_retracted, hopper_loaded, optical_clear, gripper_clear, hopper_clear다.
표본이 있으면 모든 키를 보내며 값은 true/false/null(미확인)이다. 숫자 1은 true가 아니다.
센서 종류가 확정됐거나 이 신호만으로 임무 성공이 충분하다는 뜻은 아니다.

`inject_test_sample(command_id, signals, receiver_now)`는 execute 이후의 **별도 시험 이벤트**다.
ACK를 받았거나 시간이 지났다는 이유로 센서를 자동 true로 만들지 않는다. 수신기 시계의
표본 시각을 보존하고 sample_age_ms를 올림한다. 조회해도 표본 번호나 시각이 바뀌지 않는다.
호스트는 `(응답 받은 시각 - 요청 보낸 시각) + sample_age_ms`를 나이 상한으로 사용한다.
서로 다른 시계의 절대 시각을 빼지 않으며, 지연 응답이 신선한 표본으로 재포장되지 않는다.
같은 표본 번호를 다시 받으면 닫는다. 조회 주기는 실제 센서 갱신 주기 합의가 필요하다.

호스트 결과는 `observed_at_s`를 만드는 대신 `valid_until_host_s`를 가진 **합성 진단 근거**다.
읽기 전 `poll(now)`로 만료를 적용한다. `snapshot()` 자체는 과거 상태 복사본이다.
`MissionExecutor`가 받을 실제 feedback으로 변환하는 함수는 만들지 않았다. 기존 임무가
robot/session/command ID가 일치하는 이 합성 근거도 거부하는 회귀시험을 추가했다.

`object_released`, `object_settled`, piece_id/좌표/방향은 장치 표본으로 제공하지 않는다.
그리퍼가 열렸다는 신호를 물체가 목적지에 놓였다는 증거로 바꾸지 않는다. confirm_clear의
실제 시각 근거와 집기 후 소유권/가림 처리 계약은 별도 작업이다.

## 다음 단계와 중복 작업 방지

로컬 검증: 전용 **54개** 통과, 전체 Python/OpenCV 5 회귀 **752개** 통과(107.380초).
영상 의존성 없는 `python -S`도 752개 중 133개 영상 시험 skip으로 통과(36.274초)했다.
변경 Python 파일 Ruff, compileall, `git diff --check` 통과. wheel/sdist 빌드 후 wheel을
별도 `recordings/manipulator-package-20260908/installed`에 설치하고 `-I -S` 격리 실행으로
13개 lab 조건 통과를 확인했다. 기존 `.venv` 의존성은 변경하지 않았다.
CI에는 lab 실행과 wheel/sdist 설치 후 smoke를 추가했다.
원격 CI 결과나 실제 장치 시험 통과를 의미하지 않는다.

- 이번 완료: 논리 명령·센서 스키마, 4대 가짜 송수신, 중복/지연/취소/만료 시험, 임무 경계 검증.
- 2026-09-08 후속 완료: [ID 교차·가림·다중 장애물·통신 지연 복합 회귀](COMPOUND_FAULTS_KO.md) 추가.
- 별도 합의 후: 실제 센서의 의미·갱신률·상충 신호 처리, 집게 안전 상태, 단일 큐브 배출 확인,
  영속 작업장부와 재부팅 후 결과 대조, 주행/조작의 공통 정지 및 경로·접촉 interlock 설계.
- 보드/매체 선정 후: 프로토콜 통합/협상, 주소·인증·무결성, 전송 어댑터, 실제 펌웨어.
- 하드웨어 필요: 그립 유지/해제 안전성, 모터/서보 출력, 실제 제동·독립 비상정지·집기/배출 성공 시험.

시험용 capability나 논리 센서 이름을 확정 장치 사양으로 복사하지 않는다. 팀원 작업과
동시 운용을 위한 live runtime 통합은 이번 완료 범위로 표시하지 않는다.
