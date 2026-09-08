# 조작 명령 실행 이력과 재시작 안전 장부

2026-09-08. CAD/배출 방식/팔 회수 규격 결정은 팀원과 합의할 항목으로 남겨두고,
보드·기구와 독립적인 **호스트 재시작 시 자동 재실행 방지**를 구현했다.
실제 장치나 live mission runtime에 자동 연결하지 않았으며 기존 명령 의미는 변경하지 않았다.

## 구현 범위

- `operation_journal.py`: SQLite 표준 라이브러리 기반 장부, 명시적 생성·고정 ID 열기,
  읽기 전용 조회, 작업자 확인 후 격리 처리.
- `journal_sender.py`: 기존 가짜 `ManipulatorSender`를 조합하는 `JournaledManipulatorSender`.
  조작을 호출자에게 넘기기 전에 이력을 커밋한다. 원래 sender는 호환성을 위해 변경하지 않았다.
- `tests/test_operation_journal.py`: 실제 임시 DB, 프로세스 강제 종료·재시작, 동시 예약,
  잠금/저장 실패/손상/다른 장부/ACK 유실과 가짜 수신기를 검사한다.

이는 영구적인 물리 exactly-once 보장이나 집기 성공 인증이 아니다. 가짜 센서는 그대로
synthetic이고 실제 임무의 성공 판정 입력으로 바꾸지 않는다.

## 식별과 저장 순서

장부는 생성 시 fleet과 무작위 `journal_id`를 고정한다. 재시작 시 **이미 저장해 둔 ID**를
지정해 기존 DB를 열어야 한다. 없거나 빈 파일, 다른 ID, 다른 스키마, 손상된 DB이면 거부한다.
없는 장부를 자동 생성하거나 잘못된 장부에서 새 ID를 알아내 재시작하는 우회는 금지한다.

각 조작에는 다음 식별을 저장한다.

| 식별 | 의미 |
|---|---|
| work_id | 같은 경기/작업의 같은 조작이면 재시작·로봇 재배정에도 유지하는 논리 작업 ID |
| robot_id | 실제 맡긴 로봇. 장부의 고정 fleet에 있어야 함 |
| session_id | 이번 호스트 송신 세션 |
| command_id / operation | 기존 조작 규격의 명령 ID·phase·action·timeout |

예: `match01-D1-close` 같은 work_id를 호출자가 제공한다. 새 프로세스마다 무작위로 만드는
세션 ID를 work_id로 쓰면 안 된다. 동일한 물리 작업인지 의미를 추론해 판별하는 기능은 아니다.

1. `begin(..., work_id=...)`: 장부 예약 커밋 **후** hello 메시지를 반환한다.
   아직 hello조차 전송되지 않았더라도 이후 종료되면 보수적으로 미확인 작업으로 남긴다.
2. `execute(now)`: 내부 sender 검증 후 execute-intent를 커밋 **후** 메시지를 반환한다.
   커밋 실패·잠금·알 수 없는 결과이면 execute를 호출자에게 넘기지 않는다.
3. 유효 execute ACK는 별도 비트로 기록한다. 이것은 **명령 접수 근거**이지 물리 성공이 아니다.
4. `stop/cancel/estop`: **디스크를 접근하지 않고** 메시지를 즉시 반환한다. SQLite 대기가
   정지 요청을 지연시키지 않게 했다. 유효 stop ACK를 받았을 때 요청/ACK 기록을 남긴다.
   ACK를 잃으면 stop 기록이 없을 수 있지만 기존 조작은 계속 미확인·차단 상태다.

장부 flags는 execute-intent=1, accepted-ACK=2, stop-request=4, stop-ACK=8의 비트 조합이다.
접수 ACK는 execute-intent 뒤에만, stop ACK는 stop-request 뒤에만 기록할 수 있다.
execute-intent는 한 번만 기록된다. 센서 표본과 다른 프로세스의 단조 시각은 복구 근거로 저장하지 않는다.

## 재시작 시 차단 정책

- 한 로봇에 미확인 작업이 있으면 **새 세션·새 command_id·새 work_id도 차단**한다.
- 동일 work_id는 전체 fleet에서 한 번만 예약할 수 있다. 다른 로봇에 재배정해 우회할 수 없다.
- `(robot_id, session_id, command_id)`를 유지한 채 work_id만 바꾸는 우회도 거부한다.
- stop ACK와 가짜 수신기 출력 0을 얻어도 물리 집기·배출 결과는 불명이다. 자동 해제하지 않는다.
- 최대 4096개 시도만 보존하며 가득 차면 차단한다. 오래된 항목 삭제·만료·자동 재전송은 없다.
- `BEGIN IMMEDIATE`와 UNIQUE 제약으로 같은 로컬 DB를 연 두 프로세스의 예약 경쟁을 직렬화한다.
  잠금 대기는 200ms로 제한하고, 실패한 writer는 다시 열어 확인하기 전까지 고장 상태로 남긴다.
- 이미 격리된 작업을 가진 이전 sender가 아직 execute를 반환하지 않았다면 후속 장부 검사가 차단한다.
  **이미 반환·전송한 명령 바이트를 장부 변경으로 회수할 수는 없다.**

## 조회와 수동 격리

영구 운용 상태는 삭제 가능한 영상 시험 산출물과 분리한다. 예시 `local_state/`는 Git에서 제외했다.
DB에는 장치/작업 이력이 있으므로 GitHub에 올리지 않는다. 이번 개발에서는 실제 운용 장부를
생성하지 않았으며 자동 시험의 임시 DB만 사용했다.

```powershell
# 처음 준비할 때 한 번만. 기존 경로면 실패한다.
New-Item -ItemType Directory -Path local_state
.\.venv\Scripts\python.exe -m robo_control.operation_journal init local_state/operations.sqlite3 --robots H1 H2 B1 B2

# 읽기 전용. 조작/재연결/정지 해제 없음.
.\.venv\Scripts\python.exe -m robo_control.operation_journal inspect local_state/operations.sqlite3
```

init 출력의 journal_id를 운용 설정에 별도로 고정해야 한다. 현재 `device_spec.json`이나
실제 runtime이 이 장부를 읽는 기능까지 구현한 것은 아니다.

격리 명령은 **장치와 이전 제어 프로세스를 먼저 정지·비활성화하고 결과를 확인한 작업자만** 사용한다.
아래 ID/작업/근거는 실제 기록으로 채워야 하며 에이전트가 임의로 확인해 주지 않는다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.operation_journal quarantine local_state/operations.sqlite3 --journal-id STORED_JOURNAL_ID --work-id STORED_WORK_ID --reviewed-by OPERATOR --evidence REVIEW_RECORD --confirm-devices-and-old-process-stopped
```

격리 후 상태는 `quarantined_never_retry`다. **해당 work_id의 재실행은 계속 금지**하며, 이력을
삭제하지 않고 다음의 다른 작업을 허용한다. `physical_success`는 항상 null이다.
작업자의 중단 확인 선언은 센서 검증/전자서명/독립 물리 정지 증거가 아니다.
아직 불확실하면 격리하지 말고 차단 상태를 유지해야 한다.

## 호출 API의 경계

```python
from robo_control.operation_journal import OperationJournal
from robo_control.journal_sender import JournaledManipulatorSender

with OperationJournal.open(path, expected_id=stored_journal_id) as journal:
    sender = JournaledManipulatorSender("H1", session_id=new_host_session_id, journal=journal)
    hello = sender.begin(reviewed_operation, now_s, work_id=stable_work_id)
    # 기존 가짜 수신기와 프레임을 왕복하고 sender.accept_frame(...) 호출.
    # 신선한 hello ACK 후에만 sender.execute(current_host_time) 가능.
    # 오류/만료 시 sender.stop(...)의 반환 메시지를 최선의 노력으로 전달.
```

송신기는 전송 어댑터가 아니다. 실제 매체 송신·fleet 전체 정지 조정·센서 성공 판정은 하지 않는다.
저장 실패 후에도 stop을 요청할 수 있지만 원격 도착을 보장하지 않는다.
동기 DB 쓰기/무결성 검사는 시간이 걸릴 수 있으므로, 향후 실제 runtime 통합에서는
저장 작업이 관측 watchdog을 막지 않는 구조와 저장 후 신선도 재검사가 필요하다.

## 한계와 다음 단계

### 검증 기록

- 전체 Python/OpenCV 5 회귀 **808개 통과**, 126.702초.
- 신규 장부/송신기 전용 **36개 통과**. OpenCV 없는 `python -S`에서도 36개 전부 통과(2.198초).
- 실제 하위 프로세스의 강제 종료 후 커밋 보존/미커밋 롤백, 두 프로세스의 동시 예약,
  DB 손상/잠금/쓰기 실패와 가짜 장치 ACK 유실을 검사했다. 물리 전원 차단 시험은 아니다.
- Ruff, compileall, `git diff --check` 통과. wheel/소스 배포본 빌드 후 별도 설치본을
  `-I -S`로 실행해 장부 생성→예약→닫기→동일 ID 재열기를 확인했다.
  빌드 증거는 `recordings/journal-package-20260908/`에 있으며 기존 `.venv` 의존성은 변경하지 않았다.
- CI의 wheel/sdist 설치 후 새 CLI 도움말과 sender import 확인을 추가했다. 원격 CI는 미검증이다.

### 남은 경계

- SQLite FULL 동기화는 저장장치/OS가 flush를 이행한다는 전제다. 실물 전원 차단·파일시스템
  내구성 시험은 수행하지 않았다. 갑작스러운 **프로세스** 종료 시험과 구분한다.
- 오래된 백업 복원, DB 행의 고의 삭제, work_id 의미의 변경, 새 장부로 우회하는 행동을
  인증/탐지하는 시스템은 아니다. 장부와 고정 ID를 함께 보존해야 한다. 불확실하면 운용 차단.
- 네트워크 공유 DB/클라우드 다중 호스트 복제, 펌웨어의 영구 이력은 구현하지 않았다.
- 손상 장부 자동 수리·재생성·항목 삭제 기능은 없다. 이력 보존 후 별도 복구 판단이 필요하다.
- 장치별 실제 배출 단위·팔 동작·센서·기구 안전 상태는 팀원 합의가 남아 있다.
  이번 장부 작업은 CAD와 어긋나는 명령을 승인하거나 실제 장치로 연결하지 않는다.

다음 개발 후보는 이 장부/조작 송신기를 가짜 장치 기반 임무 runtime에 명시적으로 연결하고,
주행·조작의 공통 정지 경계를 검증하는 것이다. 실제 기구 동작·성공 판정 연결은 합의 후 진행한다.
