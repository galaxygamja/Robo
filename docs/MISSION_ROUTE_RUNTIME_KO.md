# 관측 → 임무 → 경로 실행 연결

> 통신 후속: `--wire-fake` 선택 시 [전체 통신 감독](RUNTIME_WIRE_KO.md)을 연결한다.
> 기본 mock 출력과 실제 센서 미연결 경계는 유지한다. 통신 오류는 전체 보류이며 자동 재출발하지 않는다.

2026-09-07. `python -m robo_control.runtime --mission ...`으로 기존 카메라 런타임에서
예선 임무를 실행한다. 출력은 차동 차체 속도 **기록 전용**이며 실제 모터/서보 통신은 없다.

## 연결된 흐름

```text
USB/영상 → 기존 검출/추적 → ObservationWorldAdapter → WorldState
                                                   ↓
예선 task_plan + 검토한 로봇 접근 자세 → Manipulator 임무 단계
                                                   ↓
기존 A* → 경유점 → 방향 맞추기/전진/최종 방향 맞추기
                                                   ↓
차체 전방 속도·회전 속도 → MockActuatorBank → JSONL
                     ↑
새 카메라 관측으로 도착 확인 / 별도 센서로 조작 완료 확인
```

`world_state.py`의 공개 계약은 [관측 상태 안내](OBSERVATION_WORLD_STATE_KO.md)를 따른다.
관측 상태를 새로 만들지 않고 그 어댑터의 불변 WorldState를 소비한다. 매 tick에서
update/poll하고, ready 및 로봇별 유효성을 확인한다. 임무 물체의 연습 좌표는
측정값으로 복사하지 않으며 미연결 물체의 위치는 `None`이다.

## 실행 계획과 명령

`config/runtime_mission.example.json`을 별도 파일로 복사하고 `null`을 검토한 값으로
채운다. 예제 자체는 실행 전에 거부된다. 경기장 보정, 물체 배치와 실제 로봇의
집게/팔/적재물까지 포함한 반경 및 접근 자세가 필요하다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.runtime --camera 0 --calibration recordings/camera.json --tags config/robot_tags.json --fleet config/qualifier_senior.json --mission recordings/mission-reviewed.json --report recordings/mission-001.jsonl --duration-s 120
.\.venv\Scripts\python.exe -m robo_control.runtime_report recordings/mission-001.jsonl
```

녹화 시험은 `--camera 0` 대신 `--video recordings/match.avi`를 쓴다. `--mission`과
`--goals`는 동시에 쓸 수 없다. 둘 다 생략하면 기존처럼 이동 목표 없이 기록한다.
기존 `--goals`/`control_loop --demo`의 메카넘 시험 모드는 보존돼 있다.

계획의 필드:

| 필드 | 의미 |
|---|---|
| `schema_version`, `coordinate_system` | 1, `bottom_left_x_right_y_up_mm` |
| `radii_mm` | 모든 등록 로봇의 차체·회전·팔·적재물을 포함한 반경 |
| `cell_mm` | A* 격자 크기 10~100mm, 전체 격자 최대 4096칸 |
| `obstacles_mm` | 검토한 고정 장애물 `{x_mm,y_mm,width_mm,height_mm}` 목록, 최대 64개 |
| `tasks` | 실행 순서. 기존 fleet `task_plan`의 고유 task ID를 부분 선택 또는 전체 선택 |
| task의 `pickup/drop/retreat` | **로봇 회전중심**의 x/y mm와 heading rad. 물체 중심 좌표와 다르다 |

예를 들어 `move-D1`의 물체 배치점은 기존 예선 task에 보존된다. `drop`에 그 물체
좌표를 그대로 넣으면 팔 길이/차체 간섭이 무시되므로 로봇 접근 자세를 따로 정한다.
명시된 목표가 반경을 포함해 외벽 또는 장애물을 침범하면 시작 전에 거부한다.
필드의 임시 배치와 실측치를 혼동하지 않는다.

## 경로와 임무 진행 조건

- 한 임무를 수행하는 로봇 한 대만 이동 목표를 갖는다. 다른 로봇은 새로 관측한
  위치에 계속 있는 것으로 취급한다. 이전 계획의 시간 tick이 지났다는 이유로
  다른 로봇이 비켰다고 가정하지 않는다. 동시 4대 최적 시간예약은 이번 구현이 아니다.
- 기존 SpaceTimePlanner/A*를 mm→m 변환해 사용한다. 이 모드에서 격자 tick은 공간
  탐색 단계다. 실제 차동 이동/회전은 관측으로 끝날 때까지 경로 점유를 유지한다.
- 현재 경유점 도착은 새 관측 2회로 확인한다. 위치 오차 3mm 이내, 최종 방향 오차
  0.03rad 이내, 측정 병진속도 10mm/s 이하·회전속도 0.1rad/s 이하가 필요하다.
  이 값은 소프트웨어 시험값이며 디스크 중심 2mm 미만의 배치 판정을 대체하지 않는다.
- `approach → align_pickup → close_servo → confirm_grip → retract → carry → align_drop
  → release_servo → confirm_clear → retreat`를 기존 Manipulator로 진행한다. 큐브는
  기존처럼 `confirm_load`부터 시작한다. 전역 task 순서와 1·2·1 소유 배정을 유지한다.
- 경로 연결 구간을 관측마다 재검사한다. 고정 장애물 및 외벽에는 현재/제안 속도와
  TTL/제동 여유를 검사하고, 다른 로봇 위험은 기존 전체 로봇 충돌 검사도 적용한다.
- 경로 없음/가로막힘은 종료 잠금이다. 이동 중 관측 유실은 전체 0 출력 및 경로 폐기,
  3프레임 재확인 뒤 현재 측정 위치에서 재계획한다. 조작 중 관측 유실은 해당 조작을
  무작정 다시 보내지 않도록 임무 오류로 잠근다.
- 전체 임무 시계는 최초 감시 tick부터 120초다. 이동 단계 30초, 센서 단계 4초에
  미완료이면 오류로 종료한다. 기존 200ms 관측·300ms 명령·100ms 감시 기준을 유지한다.

## 차동 출력과 센서 경계

새 모드는 `drive_model="differential_body"`다. `forward_velocity_mm_s`와
`angular_velocity_rad_s`가 차체 목표이며 세계좌표 속도는 관측 방향으로 회전한
전방 성분이다. 옆 이동은 없다. 후방 목표는 먼저 방향을 바꿔 전진한다.
휠 치수/극성/출력 보정값이 미정이므로 `wheel_velocity_rad_s=[]`이며 PWM이나
좌우 휠 속도를 추정하지 않는다. 검증기는 옆 성분, 잘못된 모델, 휠 배열,
속도/만료/세션 오류를 거부한다.

`LiveControlSession.advance(record, now_s, feedback=...)`가 센서 어댑터용 입력 경계다.
CLI에는 아직 센서 전송 계층이 없으므로 영상만 실행하면 첫 실제 센서 확인 단계에서
대기하다 제한시간에 종료한다. 영상 또는 시간이 지났다는 이유로 집기를 성공 처리하지 않는다.

피드백 형식은 같은 호스트의 단조 시계 기준이다:

```python
feedback = {
    "session_id": session.controller.session_id,
    "command_id": intent["command_id"],  # 해당 task/phase에 발급된 값
    "robot_id": intent["robot_id"],
    "observed_at_s": sensor_host_timestamp,
    "synthetic": False,
    "signals": {"servo_closed": True},
}
```

새 단계 진입 뒤 측정한 200ms 미만 값만 수락한다. 다른 세션/이전 단계/로봇/합성
피드백은 진행 근거가 되지 않는다. `confirm_clear`는 required_signals 외에도
`piece_id`, 측정 `piece_x_mm`, `piece_y_mm`, `piece_yaw_rad`가 필요하며 기존 목적지의
완전 수용 판정을 통과해야 한다. 없는 센서나 측정하지 않은 큐브 각도를 임의로 채우지 않는다.
피드백의 문자열 ID는 중복 방지·연결 계약이며 장치 인증 프로토콜은 아니다.

JSONL `mission`에는 임무 단계, 경로/경유점, 단독 이동 로봇, WorldState, 확인된 작업
목록과 오류를 남긴다. `manipulator_intent`는 `dispatch_enabled=false`인 의도 기록이다.
실제 조작 송신은 구현하지 않았다. `score=null`이며 확인된 임무 수를 득점으로 계산하지 않는다.
`mission_completed`는 모든 선택 임무의 센서/이동 조건을 확인한 소프트웨어 종료 사유다.
단순 영상 EOF나 종료 코드 0은 임무 완료가 아니다.

## 검증과 남은 작업

2026-09-07 최종 검증: 전체 Python **392개 통과**(다른 채팅의 WorldState 시험 포함),
새 임무/차동/영상 통합 시험 **16개 통과**, 변경 Python 파일 Ruff 통과,
`git diff --check` 통과. 전체 실행 로그는 Git 제외 경로
`recordings/mission-final-regression.log`에 보존했다. 실제 카메라/로봇의 시험 결과가 아니다.

`tests/test_mission_runtime.py`는 옆/후방 목표·각도 경계, A* 장애물 우회·경로 없음,
새 관측 도착·센서 거부·두 임무 연속 진행·관측 만료·WorldState 미연결 물체·출력 TTL,
실제 MJPG 디코딩부터 CLI 임무 로그/최종 0까지 검사한다. 위치를 갱신하는 시험용
plant는 테스트 파일에만 있으며 운영 코드에는 없다.

남은 작업은 실측 접근 자세/기구 보정, 센서 전송 어댑터와 실제 장치 통신,
독립 펌웨어 정지·벤치 검증이다. HSV 임시 트랙과 임무 물체의 명시 연결 및 그
관측을 이용한 동적 픽업점/장애물 지도, 동시 다중 로봇 경로 실행은 별도 확장이다.
