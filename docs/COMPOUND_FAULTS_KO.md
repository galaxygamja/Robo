# 가림·ID 교차·다중 장애물·통신 지연 복합 회귀

2026-09-08. 하드웨어 없는 우선순위 2 작업. 기존 검출·추적·임무·안전 정책을
완화하거나 재구현하지 않고, 이를 함께 통과하는 **생성 픽셀 기반 회귀시험**을 추가했다.
팀원 CAD/웹 동선/장치 사양은 수정하지 않는다.

## 실행

기존 프로세스 분리 영상 CLI 3개 시나리오에 복합 시나리오 8개를 이어서 실행한다.

```powershell
.\.venv\Scripts\python.exe tools/verify_mission_operations.py --output-dir recordings/operations-new --seconds 6 --compound-faults
```

복합 시험만 별도로 재현할 수도 있다.

```powershell
.\.venv\Scripts\python.exe tools/verify_compound_faults.py --output-dir recordings/compound-new
.\.venv\Scripts\python.exe -m unittest discover -s tests -p test_compound_faults.py -v
```

출력 폴더는 반드시 새 이름을 사용한다. 실패한 실행도 덮어쓰거나 삭제하지 않는다.
선택 영상 의존성이 없으면 픽셀 시험은 명시적으로 skip하며, 실제 영상 시험 통과로 세지 않는다.
소스 배포본에 두 검증 도구를 포함하도록 MANIFEST를 갱신했다.

## 기존 시험과의 차이

기존 `verify_mission_operations.py`는 실제 영상 파일·영상 작업 프로세스·호스트 시계·runtime CLI를
검증한다. 이번 추가 시험은 **같은 실제 OpenCV DetectionPipeline + LiveControlSession +
RuntimeWireSupervisor**에 생성 픽셀과 결정적 시험 시계를 직접 공급한다.
새 오류 주입 옵션을 실제 로봇 runtime CLI에 노출하지 않았다.

20ms 간격의 시험 시각은 운영체제 지연을 측정한 값이 아니다. 제어 결과를 적분한 위치,
가짜 집기 센서, 실제 무선/직렬 출력은 없다. 기존 프로세스 분리·실시간 영상 검증을
대체하지 않으며, 두 종류를 한 명령으로 함께 실행할 수 있게 했다.

모든 로봇 위치는 태그 픽셀 검출 결과다. 물체는 색상·면적·형상 검출과 기존 시간 추적을 거친다.
기존 시험용 보정/색상/영역 연결을 재사용한다. 추가 장애물이 태그 마스크 바깥에서 보이면서
차체 외곽과 겹치도록 이 시험의 로봇 반경만 80mm로 명시했다. 이것은 **합성 fixture 치수**이며
실제 로봇/CAD 치수나 운용 설정으로 채택한 것이 아니다.

## 시나리오와 정확한 기대 결과

| 시나리오 | 복합 조건 | 기대 종료 사유 |
|---|---|---|
| multi_clear | 물체 3개, 정상 차동 명령·수신 기준선 | operator_stop |
| occlusion_request_delay | 물체 가림 + 이동 요청 120ms 지연 | object_obstacle_lost |
| occlusion_ack_delay | 물체 가림 + 이동 ACK 120ms 지연 | object_obstacle_lost |
| crossing_request_delay | 같은 색 물체 접근·겹침·교차 + 요청 지연 | object_obstacle_unconfirmed_or_ambiguous |
| crossing_ack_delay | 같은 색 물체 접근·겹침·교차 + ACK 지연 | object_obstacle_unconfirmed_or_ambiguous |
| envelope_request_delay | 기존 물체 3개 + 차체 외곽에 새 물체 진입 + 요청 지연 | mission_obstacle_envelope |
| envelope_ack_delay | 동일한 새 물체 진입 + ACK 지연 | mission_obstacle_envelope |
| occlusion_stop_ack_loss | 물체 가림 + 이동 ACK와 이후 H1 stop ACK 유실 | object_obstacle_lost; H1 정지 확인 불명 |

각 시험은 먼저 정상 관측·대상 연결·관측 픽업점·장애물 지도·비영(0이 아닌) 가짜 명령을 확인한다.
9번째 프레임 전 통신 오류를 설정하고 10번째부터 픽셀 위험을 넣는다. 교차 시험은 두 빨간
물체가 겹쳐 식별이 불명확해진 11번째 프레임에서 정지한다. 색상이 같다는 이유로 임무 대상
ID를 다시 붙이지 않는다. 교차가 끝난 뒤 어느 쪽이 원래 물체인지 알아냈다는 시험이 아니다.

## 검증하는 불변 조건

- 위험 직전 이동 명령 ACK 대기를 실제로 확인한다. 요청 지연은 아직 미접수,
  ACK 지연은 이미 접수된 상태라는 차이도 별도 회귀시험으로 확인한다.
- 위험을 발견한 프레임과 이후에는 **4대 모두 accepted_drive_count가 증가하지 않는다**.
- 가림/혼동 때문에 장애물 이력이 없어지거나 새 물체 이력으로 교체되지 않는다.
- 연결된 D1의 track ID가 바뀌지 않는다. 실제 성공/득점/센서 근거를 생성하지 않는다.
- 모든 로봇에 stop을 요청하고, 로컬 출력은 정지 이후 계속 0이다.
- 가림을 해제하거나 물체가 다시 분리된 정상 픽셀을 계속 공급해도 닫힌 임무가 자동 재시작하지 않는다.
- stop ACK를 잃으면 가짜 watchdog 후 목표 출력이 0이어도 H1은 정지 확인 불명으로 남는다.

검증기의 성공 조건도 시험한다. 로그에 사후 이동 접수, 임의 종료 사유, 장애물 제거,
대상 ID 교체, 성공 위조, 빠진 전체 정지 요청, 비영 출력 또는 거짓 ACK 확실성을 넣으면 거부한다.
8개 시나리오는 함수 기반 시험 20개에 포함되므로 이를 28개 테스트 함수로 중복 계산하지 않는다.

## 증거 파일과 시각 경계

상위 `verification.json`은 전체 성공일 때만 생성한다. 시나리오별로 다음을 남긴다.

- `runtime.jsonl`: **첫 임무 종료에서 끝나는** 기존 형식 보고서. 기존 엄격한 분석기로 검사한다.
- `shutdown-trace.jsonl`: 최초 관측부터 종료 이후 wire 감시까지 전체 tick. 별도 불변 조건 검증 대상이다.
- `detections.jsonl`: 종료 이후 다시 나타난 물체까지 포함하는 실제 픽셀 검출 원본 기록.
- `injections.json`: 어떤 프레임 전에 어떤 가짜 링크 옵션/픽셀 오류를 적용했는지 기록.
- `frame-008.png`, 최초 오류 프레임 PNG, `frame-020.png`: 기준선·오류·재등장 픽셀 증거.
- 시나리오별 `verification.json`: 정확한 정지 사유, 최초 정지 tick, 최종 ACK 불명 목록과 보고서 분석.

종료 후 도착한 ACK를 최초 종료 시점에 받은 것처럼 소급하지 않는다. 따라서 요청 지연
시나리오는 runtime 보고서 끝에서는 H1 stop 미확인이고, 후속 shutdown trace 끝에서는
확인될 수 있다. ACK 유실 시나리오는 끝까지 미확인이다. 어느 경우든 **물리 정지 검증은 false**다.
폴링 수신기 모델은 프로세스가 멈춘 상황의 독립 하드웨어 watchdog을 보장하지 않는다.

## 남은 범위

### 로컬 검증 기록

- 전체 Python/OpenCV 5 회귀 **772개 통과**, 125.074초.
- 신규 전용 **20개**는 OpenCV 5 및 OpenCV 4.14.0에서 통과. OpenCV 4 실행은 18.737초.
- OpenCV 5에서 기존 영상 CLI 3개 + 복합 8개를 `--compound-faults` 한 명령으로 통과.
  재현 증거: `recordings/compound-integrated-cv5-20260908/verification.json`.
- Ruff, compileall, `git diff --check` 통과. 소스 배포본 빌드 및 두 검증 도구 포함 확인:
  `recordings/compound-package-20260908/robo_control-0.1.0.tar.gz`.
- 최초 시험 중 종료 이후 tick을 기존 보고서에 계속 넣은 형식 오류를 수정했다.
  종료 이후 증거는 별도 shutdown trace로 보존하며 기존 분석기의 거부 규칙은 완화하지 않았다.
  실패 시도의 폴더도 남겼고 통과 증거로 취급하지 않는다. 실제 장치/원격 CI는 이번 단계에서 검증하지 않았다.

### 다음 입력

합성 회귀가 현장 인식률, 실제 충돌 회피, 무선 지연 분포 또는 집기 성공을 인증하지 않는다.
다음은 실제 경기장/카메라 녹화 영상과 기대 판정으로 회귀 자료를 확장하는 작업이다.
이는 로봇 없이도 가능하지만 영상 입력이 필요하다. 실제 센서 의미·기구 안전 상태·보드/통신
선정과 live 조작 통합은 별도 합의 및 구현 단계다.
