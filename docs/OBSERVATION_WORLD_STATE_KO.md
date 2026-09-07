# 관측 → 임무 상태: 다른 채팅과의 구현 경계

2026-09-07 구현·검증 완료, 계약 v1. 이 채팅은 아래 파일만 담당한다.

- `robo_control/world_state.py`
- `tests/test_world_state.py`, `tests/test_world_state_integration.py`
- 이 문서

다른 채팅의 임무/경로 실행은 이 계약을 소비한다. 기존 `runtime*`, `control_loop`,
`planner`, `qualifier`, `models`, 공용 설정/문서를 이 작업에서 수정하지 않는다.
작업 트리에는 다른 채팅의 변경도 있으므로 브랜치 전환·일괄 커밋·push를 하지 않는다.

## 공개 API

```python
from robo_control.world_state import ObservationWorldAdapter, PieceSpec

adapter = ObservationWorldAdapter(
    roles={"H1": "hamster", "H2": "beaver", "B1": "beaver", "B2": "beaver"},
    pieces=[PieceSpec("R1", "cylinder", "red")],
    source_name="webcam:0", session_id="one-source-session",
    field_size_mm=(1143.0, 1181.0), is_replay=False,
    configuration_id=None,  # runtime 사용 시 시작 행의 실제 식별자로 고정
)
# record는 detect --track 또는 runtime_tick.observation의 전체 행이다.
world = adapter.update(record, now_s, source_session_id="one-source-session")
world = adapter.poll(now_s)  # 새 프레임이 없을 때; 확인 횟수/나이를 새로 만들지 않음
world = adapter.invalidate(now_s, reason="upstream_rejected_frame")  # 알고 있는 오류는 poll과 구분
world = adapter.bind_piece("R1", "O0001", now_s, evidence="operator mapping in this session")
world = adapter.unbind_piece("R1", now_s)
world = adapter.close(now_s)
```

`source_session_id`는 필수이며 입력 소유자가 세션마다 새로 부여한다. runtime에서는
바깥 `runtime_tick.session_id`를 사용한다. 서로 다른 PC의 단조 시각을 직접 비교하지 않는다.
카메라/세션/보정/좌표계가 바뀌면 같은 어댑터를 재사용하지 않는다.

`WorldState`와 내부 로봇/물체/임무 물체 레코드는 불변 데이터다. `as_dict()`는 독립 JSON
사본이다. `world.robot(id)`, `world.piece(id)`로 상태를 찾는다.
`world.robots` / `world.objects` / `world.pieces`는 튜플이다.

- `world.ready`: 프레임·등록 로봇들이 정상이고 3프레임 재확인한 관측 상태.
  모터 허가·경로 안전·전체 물체 확인·전체 임무 가능을 뜻하지 않는다.
- 로봇: `robot_id`, `role`, `position_mm`, `heading_rad`, `velocity_mm_s`,
  `observed_at_s`, `state`, `valid_for_control`, `reason`. `position_m` 속성으로 명시 변환.
- 물체: `object_id`, `kind`, `colour`, `position_mm`, 관측 시각·상태·불확실성,
  `owner_robot_id`, `lifecycle`, `position_evidence`, `position_valid`, `valid_for_pick`.
- 임무 물체: `piece_id`, `kind`, `colour`, `track_id`, `position_mm`, `yaw_rad`,
  `owner_robot_id`, `lifecycle`, `observed_at_s`, `position_valid`, `valid_for_pick`, `reason`, `binding_evidence`.
  미연결/미관측 좌표와 미측정 각도는 `None`이다.

`PieceSpec`은 임무 물체의 ID/종류/색 카탈로그다. `PieceSpec.from_piece(piece)`는 기존
`qualifier.Piece`의 이 세 값만 복사한다. 연습 좌표, released, held_by는 복사하지 않는다.
같은 색이거나 가까운 연습 위치라는 이유로 임무 ID를 자동 배정하지 않는다.

물체 연결은 해당 세션의 신선하고 확인된 관측에 대한 명시 기록이다. 물리 동일성 인증이
아니며, 같은 트랙을 두 임무 물체에 연결하지 않는다. 유실·모호성으로 끊긴 ID를 새 ID로
자동 대체하지 않는다. `released_pending` 위치는 힌트일 뿐 측정 좌표로 승격하지 않는다.

## 임무/경로 채팅이 주의할 사항

1. 매 실행 tick에서 `update` 또는 `poll`로 **새 WorldState**를 받아 검사한다.
   과거 불변 스냅샷을 잡고 있으면 시간 만료가 자동 적용되지 않는다.
   runtime이 잘못된 입력을 거부하며 `observation=None`을 내보내면 `invalidate`를 호출한다.
   **정상 프레임 사이의 빈 tick만 poll**이며, 명시적인 입력 실패·종료를 빈 tick으로 숨기지 않는다.
   런타임 종료는 `close`로 전달한다. 경로 충돌과 같은 실행 판단은 관측 진실성과 별도다.
2. 전역 ready와 개별 로봇/물체 유효성을 모두 검사한다. 최초 확인 전, 새 잘못된 프레임,
   유효 관측 200ms 만료는 작업 보류 상태다. 폐쇄된 세션은 다시 열리지 않는다.
3. `models.Point`/기존 경로계획은 m 단위를 쓰는 부분이 있다. `position_m` 또는 명시적
   `/1000` 변환을 사용한다. 바닥 원점은 좌하단, +X 오른쪽/+Y 위, +X=0rad/반시계 양수다.
4. 전송된 소유/생애주기 정보는 기존 tracker의 주장이다. 이를 실제 센서 확인이나
   작업 완료·득점으로 바꾸지 않는다. 새 `qualifier.Piece`의 기본 `released=True`나
   `yaw_rad=0`도 관측 근거 없이 채워 넣으면 안 된다. 큐브 각도는 현재 검출되지 않는다.
   `position_mm`에 마지막 좌표가 남아 있어도 `position_valid=false`이면 현재 측정으로 쓰지 않는다.
   객체는 upstream의 confirmed 플래그만 믿지 않고 최소 3연속 프레임을 요구한다.
5. 이 계층은 임무 배정·경로 생성·도착/집기/놓기 성공 판정·점수 계산·모터 출력을 하지 않는다.
   구동 방식과 독립적이며 기존 메카넘 제어기를 차동용으로 고치는 일도 다른 작업이다.

## 구현 및 검증 결과

- `ObservationWorldAdapter`와 불변 로봇/물체/임무 물체 상태를 구현했다. 등록된 4대의
  관측, 원천 세션·좌표계·설정 식별자, 시각/순서, 전역 준비 상태를 함께 관리한다.
- 오류 프레임은 부분 반영하지 않는다. 관측 만료·누락·모호한 물체 ID·세션 변경과
  정상 프레임 사이의 빈 tick을 구분한다. 재확인 없이 과거 관측을 되살리지 않는다.
- 임무 물체의 명시적 연결/해제 API를 구현했다. 시나리오는 종류/색/ID 카탈로그로만
  사용하며 위치·사전 적재·집기 성공·배치 성공·득점을 관측 사실로 만들지 않는다.
- 다른 채팅의 `runtime_session.py`에서 이 API를 사용하는 것도 확인했다. 해당
  런타임/경로 코드는 이 작업의 수정 대상이 아니며 세부사항은
  [임무·경로 실행 안내](MISSION_ROUTE_RUNTIME_KO.md)를 따른다.

2026-09-07 로컬 검증:

| 검증 | 결과 |
|---|---|
| 전체 Python 회귀 시험, OpenCV 5 환경 | 398개 통과 — 다른 채팅의 임무 시험 포함 |
| 이 작업의 전용 시험, OpenCV 5 환경 | 34개 통과 |
| 이 작업의 전용 시험, OpenCV 4 환경 | 34개 통과 |
| OpenCV 없는 기본 Python 환경 | 34개 실행 중 33개 통과, 영상 시험 1개 건너뜀 |
| 신규 Python 파일 Ruff | 통과 |

전용 시험은 기존 runtime 출력과 JSON 왕복, 4개 AprilTag와 색 물체를 그린 시험용
이미지의 실제 OpenCV 검출/추적, 물체 ID 연결, 200ms 만료, 3프레임 재확인,
잘못된 입력·세션, 불변 데이터, 물체 유실/재등장/소유/놓기 힌트를 검사한다.
**실제 카메라·장치·현장 정확도 검증은 하지 않았다.** 코드의 `ready`는 현장 운용 승인도 아니다.

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p 'test_world_state*.py' -v
.\.venv\Scripts\ruff.exe check robo_control/world_state.py tests/test_world_state.py tests/test_world_state_integration.py
```

## 다음 연결 작업

현재 임무 물체 매핑은 Python API다. 실제 영상에서 어느 임시 트랙이 어느 임무
물체인지 운영자가 확인하는 설정/UI 및 세션별 연결 기록은 별도 구현이 필요하다.
같은 색의 물체가 여러 개일 때 자동으로 고유 ID를 확정하는 기능은 아니다.

임무 실행 측은 최신 `WorldState`를 소비하면서, 물체별 `position_valid`/
`valid_for_pick`과 임무 조건을 별도로 확인해야 한다. 동적 픽업 접근점·장애물 지도,
센서로 확인하는 조작 결과, 실제 송신/펌웨어 정지는 이 계층 밖의 작업이다.
