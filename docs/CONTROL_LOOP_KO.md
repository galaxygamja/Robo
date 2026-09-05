# 측정 좌표 폐루프와 독립 명령 watchdog: 모의 출력 전용

`robo_control/control_loop.py`는 매번 새로 측정된 위치와 명시 목표 사이의 오차를 계산하고, 제한된 속도와 메카넘 휠 속도 **모의 명령**을 만든다. 하드웨어 연결, 모터 극성 결정, 실제 주행, 서보 제어, 드론 제어를 구현한 모듈은 아니다. `enable_hardware=True`는 예외로 거부한다. GPIO·serial·radio·UDP 전송을 만들지 않는다.

## 실행

저장소 루트에서 실행한다. 이 모듈과 단위 시험은 표준 라이브러리만 사용한다.

```powershell
py -m robo_control.control_loop --demo
py -m robo_control.control_loop --demo --robots 5 --compact
py -m unittest discover -s tests -p test_control_loop.py -v
```

데모는 `PoseTracker → ClosedLoopController → MockActuatorBank → 이상적 모의 위치 갱신 → PoseTracker`로 되먹임한다. 로봇 사이를 **800mm** 띄운 넓은 합성 시험장이다. 실제 경기장 배치, 120초 물체 운반, 5대 혼잡 해소를 시험하는 데모가 아니다. 4·5·12대 모두 목표 오차 3mm 이내와 방향 오차 0.03rad 이내로 수렴하는 것은 이 이상적 모델에 한한 결과다. 끝에 명령 송신을 중단하고 actuator watchdog이 전부 0으로 바뀌는지도 확인한다.

## 입력과 출력 계약

생성 시 `roles`는 내부 ID→`hamster`/`beaver` mapping, `goals`는 ID→`{x_mm, y_mm, heading_rad?}` mapping이다. 역할은 ID 접두사에서 추측하지 않는다. 목표를 주지 않은 등록 로봇은 정지한다. `radii_mm`는 등록 로봇 모두의 차체·팔·적재물을 포함하는 원형 안전봉투 반경이며 기본 150mm는 보수적인 모의 가정이다. 실물 반경을 확인하지 않은 채 줄이면 안 된다.

`tick(record, now_s)`의 `record`는 비전 `--track` JSONL의 **전체 행**이다. `PoseTracker.update()` 반환값만 전달하면 원본 프레임 metadata가 없으므로 거부한다. 필요한 값은 다음과 같다.

- `source_name`, 증가하는 `sequence`, `captured_at_s`.
- `observation_usable=true`, `stop_required=false`.
- 등록 ID마다 정확히 한 개의 `tracks` 항목. 각 항목에 `robot_id`, `robot_center_mm`, `heading_rad`, `observed_at_s`, `velocity_mm_s`, `angular_velocity_rad_s`, `state="observed"`, `valid_for_control=true`.

좌표는 mm, 좌하단 원점, +X가 오른쪽이며 방향 0이다. 각도는 rad, 반시계가 양수다. 현재 입력은 한 호스트 프로세스 세션의 단조 시계만 지원한다. 프레임과 track 시각이 일치해야 하며 200ms보다 오래된 관측, 미래 시각, 역순·재사용 프레임, 다른 소스, 누락·중복·미등록·잘못된 수치는 전체 팀 대기 명령을 만든다. 단절 중 위치를 추측해 계속 이동하지 않는다.

Windows 등에서 단조 시각 눈금이 같아도 프레임 sequence가 엄격히 증가하면 컨트롤러의 헤더 검사는 수락한다. 같은 시각을 별도의 경과 시간으로 해석해 가속도를 더하지는 않는다. 상위 PoseTracker의 속도 관측은 녹화일 때 증가하는 미디어 시각도 필요하고, 실제 카메라에서 위치 시각 차이가 0이면 안전하게 거부한다. `status="source_closed"`, PoseTracker의 `tracking_session_closed=true`, 또는 컨트롤러 계약의 `localization_session_closed=true`이면 다른 정상 플래그가 모순되게 들어와도 종료 상태가 잠긴다. 이후 프레임이나 비상정지 해제로 다시 열리지 않으며, 새 카메라 세션과 새 컨트롤러·actuator 세션을 명시적으로 구성해야 한다.

출력은 JSON 직렬화 가능한 packet이다.

- `schema_version=1`, `session_id`, 증가하는 `sequence`, `issued_at_s`, `ttl_s=0.3`.
- `status`: `tracking_goal`, `at_goal`, `hold`.
- `stop_reason`, `conflicts`, `emergency_stop`.
- `robots`: 각 내부 ID의 `velocity_world_mm_s=[vx,vy]`, `angular_velocity_rad_s`, `pose_heading_rad`, `wheel_velocity_rad_s=[FL,FR,RL,RR]`, `at_goal`.
- `device_io=false`, `hardware_ready=false`, `motion_permitted=false`는 항상 유지한다. `mock_motion_permitted`만 모의 출력 허용 여부를 나타낸다.

세계 좌표 속도를 로봇 전방/왼쪽 속도로 회전 변환한 뒤 이상적인 X배열 메카넘 식을 적용한다. 실제 휠의 롤러 배열·모터/엔코더 부호·반경은 별도로 검증해야 한다. 소프트웨어의 위치 이득, 최대 속도 180mm/s, 회전 1.5rad/s, 가속도 및 3mm 목표 허용치는 **현재 모의 값**이지 실물 성능 보장이 아니다.

특히 **3mm는 일반 이동 목표의 데모 허용치이며 LAB 디스크 정렬 합격치가 아니다.** 현재 기준의 디스크 중심 오차는 2mm 미만이어야 하므로 이 컨트롤러의 `at_goal`을 디스크 배치 성공이나 득점으로 사용하면 안 된다. 별도 근접 정렬, 측정 오차 예산, 물체 완전 수용·분리 확인이 필요하다.

## 충돌 예측

매번 모든 로봇 쌍을 검사한다. 각 중심에서 현재 위치와 측정 속도·제안 속도의 짧은 미래 끝점으로 swept hull을 만들고 다음 여유를 더한다.

`차체/팔/적재물 반경 + 속도 × 명령TTL + 속도² / (2 × 가정 제동감속도)`

이 확장 영역 사이가 15mm 이하이면 전체 출력이 0이 된다. 도착 시간을 무시한 영역 비교여서 실제로 시간차 통과가 가능한 상황도 막을 수 있다. 측정 속도가 남아 있으면 정지 제안을 했더라도 제동 위험을 검사한다. 이는 위험 검출기이지 최적 경로·장애물 지도·출발구역 유효성 검사·교착 해소기 또는 물리 안전 증명이 아니다. 실제 제동거리와 지연을 측정하기 전에는 이 모델로 실제 모터를 켜면 안 된다.

## 정지와 독립 watchdog

`set_paused(True)`는 다음 tick에서 전체 대기 packet을 만든다. `emergency_stop()`은 latch를 세우며 신선한 위치가 들어와도 자동 해제되지 않는다. 명시적인 `reset_emergency_stop()` 후에도 새 프레임이 필요하다.

`MockActuatorBank(robot_ids, session_id=controller.session_id)`는 컨트롤러와 별도의 객체다. `receive(packet, now_s)`로 packet을 받으며 `tick(now_s)`는 컨트롤러 호출 여부와 독립적으로 실행해야 한다. 발행 시각 기준 300ms에 전부 0이 되므로 늦게 도착한 명령이 수신 순간부터 다시 300ms 살아나지 않는다. 역순·중복 시퀀스, 다른 프로세스 session, 누락 로봇, 위조 하드웨어 허용 값, 속도 초과, 휠 식 불일치도 거부한다.

Actuator 자체 비상정지도 별도로 latch된다. 컨트롤러 latch만 해제해서는 actuator가 재시작하지 않는다. 양쪽의 명시 해제 후 **새 시퀀스**가 필요하다. 해제는 이전 명령을 다시 재생하지 않는다.

프로세스가 완전히 종료되면 이 Python 객체도 실행되지 않는다. 이 시험이 실제 펌웨어 watchdog을 제공하는 것은 아니다. 실물 연결 단계에서는 로봇 쪽 독립 시계·통신 watchdog·하드웨어 비상정지가 필요하다. 다른 호스트의 monotonic 시각을 직접 비교하는 전송도 아직 지원하지 않는다.
