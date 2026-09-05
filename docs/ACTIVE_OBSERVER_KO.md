# 박쥐의 능동 관측: 시점 선택과 여러 관측원의 좌표 선택 계약

`robo_control/active_observer.py`는 드론을 단순한 고정 상공 카메라 대신, **현재 임무에 가장 필요한 위치를 더 잘 볼 수 있는 시점으로 옮기는 관측 계획기**로 다룬다. 실제 기체 명령은 내보내지 않는다. 표준 라이브러리만 쓰며 비행 SDK, 실시간 동적 OpenCV 보정, 렌즈/기체 자세 추정, 무선 연결은 구현하지 않았다.

## 실행과 확인 가능한 결과

저장소 루트에서 실행한다.

```powershell
py -m robo_control.active_observer --demo --compact
py -m unittest discover -s tests -p test_active_observer.py -v
```

데모의 두 경우는 **동일한 물체·로봇 위치, 같은 높이 600mm 가림 상자, 같은 H1 fallback 관측 누락**을 사용한다. 달라지는 것은 드론을 원래 위치에 세워 두는지, 후보 시점 중 필요한 관측을 가장 많이 회복하는 곳으로 제한 속도로 이동하는지다.

- 고정 시점: 전체 로봇 관측이 충족된 프레임 0/60.
- 능동 시점: 49/60프레임에서 전체 관측 충족, 합성 시간 1.1초 후 최초 회복.
- 두 경우 모두 모든 관측원을 중단하면 freshness watchdog 이후 대기 요청.

이 수치는 **이상적인 투영과 보정 metadata를 제공한 결정론적 합성 예제** 결과다. 실제 영상의 검출률, 비행 성능, 경기 점수, 5대 경기 완주, 실제 제어 안전성 향상률을 측정한 것이 아니다. 공개 웹 시뮬레이터의 별도 물리/시간 모델과 수치가 같다고 가정하지 않는다.

데모는 별도 기준 표식 네 개의 위치도 명시하고, 이동 중 매 시점의 화각·가림으로 보이는 기준 표식 수를 검사한다. 다만 metadata의 재투영 오차와 높이 모델은 이상적으로 주어진 값이며 실제 픽셀에서 추정한 보정이 아니다.

## 1. 필요한 곳을 보는 시점 선택

주요 API:

```python
planner = ActiveObserverPlanner(candidates, bounds=FlightBounds(), camera=CameraModel())
decision = planner.plan(current_pose, targets, occluders, now_s=host_time)
```

- `DronePose(x_mm, y_mm, z_mm, yaw_rad=0)`: mm 좌표와 rad 방향. 카메라는 수직 아래를 보며 yaw만 모델링한다. 실제 roll/pitch 흔들림은 별도 어댑터 영역이다.
- `ObservationTarget(target_id, kind, position_mm, ...)`: 로봇/물체/투하 확인 구역의 마지막 위치 힌트. `kind`는 `robot`, `object`, `drop_zone`이다. `position_mm`는 길이 3의 `(x,y,z)` 좌표다.
- `missing`, `age_s`, `task_state`와 `weight`: 관측 누락·노후화·집기·운반·투하 확인의 우선순위를 반영한다. 이미 fallback이 보는 대상은 드론의 추가 관측 이익을 낮춘다.
- `marker_size_mm`, `uncertainty_mm`: 표식/관측 대상의 모델 크기와 위치 힌트의 불확실성. 실제 크기와 같은 항목으로 취급하면 안 된다.
- `Occluder(id, min_mm, max_mm)`: 높이를 갖는 3차원 직육면체. 대상 높이와 드론 높이를 함께 사용하므로, 평면 지도에서 선이 상자를 지난다는 이유만으로 모두 가려졌다고 판단하지 않는다.

입력 힌트는 마지막 유효한 측정이나 사전에 확인한 위치여야 한다. 안 보이는 물체의 현재 정답 위치를 실제 시스템이 알고 있는 것으로 가정하면 안 된다. 위치가 불확실할수록 `uncertainty_mm`를 늘려야 한다. 힌트가 완전히 사라진 상태의 탐색 지도나 임의 드론 수색은 이 모듈이 자동으로 생성하지 않는다.

기본 카메라 모델은 가로 70°·세로 60° 화각, 1280×720, 관측 대상 최소 18픽셀이다. 실물 드론 사양을 확인한 값이 아니다. 핀홀 투영에서 높이가 커지면 보이는 범위는 넓어지지만 물체 픽셀 크기가 줄어든다. 전체 표식과 불확실성 여유가 화각 안에 들어오는지 확인하고, 중심 및 네 모서리의 3D ray/box 교차로 부분 가림을 검사한다. 다섯 광선 샘플이 모든 픽셀의 가림 부재를 증명하는 것은 아니다.

후보는 최대 128개, 관측 목표는 최대 256개, 가림 상자는 최대 128개다. 후보별 예상 관측 가치에서 이동·높이 변경 비용을 뺀다. 이전 시점보다 개선이 충분하지 않으면 바꾸지 않고, 기본 0.4초 유지 시간으로 작은 변화에 왕복하는 동작을 줄인다. 전역 최적해나 대회 최적 점수를 증명하는 알고리즘은 아니다.

목표 목록이 비면 유지 시간 중이더라도 이전 시점을 즉시 버리고 현재 위치 대기를 제안한다. 유지 시간은 임무가 없어졌는데 계속 이동하는 근거가 아니다.

기본 위치 범위·고도 650~1100mm, 수평 300mm/s·수직 150mm/s·yaw 1rad/s, 기체 여유 60mm는 **모의 값**이다. 규정의 허용 높이나 실제 기체 한계를 대신하지 않는다. 후보까지의 기체 여유 포함 경로를 확인하고, 속도 제한을 적용한 실제 짧은 다음 구간도 다시 확인한다. 평면·높이 제한, 유지 시간, 관측 이익이 모두 맞아야 이동 힌트를 만든다.

출력:

- `selected_pose`: 선택한 최종 관측 시점.
- `next_pose`: 이번 시간 간격에서 속도 제한을 적용한 제안 위치. 실제 도달했다고 주장하는 상태가 아니다.
- `expected_visible_ids`, `current_expected_visible_ids`: 기하 모델의 예상 관측 대상.
- `weighted_coverage`, `movement_cost`, `net_benefit_over_current`, 후보별 `views`와 거부 이유.
- `physical_commands=false`, `device_io=false`, `flight_sdk_implemented=false`.
- `calibration_required_per_frame=true`, `dynamic_calibration_implemented=false`.

## 2. 움직인 카메라는 프레임마다 보정 증거가 필요하다

`calibration_gate(frame, moving=True, ...)`는 보정 **metadata 검사기**다. 대응점을 검출하거나 호모그래피를 계산하지 않는다. 다음 metadata를 해당 프레임의 sequence·시각과 정확히 연결해 요구한다.

- 프레임/보정의 해상도 일치.
- 공통 `field_frame_id` 일치.
- 기준점 4개 이상과 유효한 기준점 배치 표시.
- 목표 높이 모델 유효 표시.
- 유한한 재투영 오차 및 위치 오차 상한.
- 이동 관측원이라면 `dynamic_reference_update=true`.

이 필드에 true를 쓰는 행위가 실제 보정 구현이나 정확도 검증이 아니다. 실제 어댑터는 매 프레임 보이는 충분히 퍼진 기준점, 렌즈 모델, 기체 자세/높이, 표식 높이 효과를 검증한 결과로만 metadata를 발행해야 한다. 정지 시점에서 저장한 보정을 움직인 카메라에 그대로 복사하면 안 된다. 현재의 `robo_control.vision --moving-camera` 거부 정책을 우회하지 않는다.

## 3. 여러 관측원에서 로봇당 좌표 하나만 선택한다

```python
selector = MultiSourcePoseSelector(
    robot_ids,
    source_sessions={"fallback": "fallback-1", "drone": "drone-1"},
    moving_sources=("drone",),
)
result = selector.ingest(frame, now_s)
watchdog_result = selector.snapshot(now_s)
```

관측원은 1~16개, 로봇 ID 목록은 고정 4대에 묶이지 않는다. 입력 `frame`에는 다음이 필요하다.

- `source_id`, `session_id`, 증가하는 `sequence`, `status="detected"`.
- 같은 호스트 시계 영역의 `captured_at_s`, `received_at_s`, `clock_domain`.
- `image_size_px`, 앞 절의 `calibration` metadata.
- `robots`: `robot_id`, `robot_center_mm`, `heading_rad`, `confidence`, `error_bound_mm`.

기본 최소 confidence 0.6, 최대 위치 오차 상한 15mm, 프레임 나이 250ms 미만을 사용한다. 행의 `error_bound_mm`은 프레임 보정의 위치 오차 상한보다 작다고 주장할 수 없다. confidence와 error bound는 외부 관측기가 제공한 값이지 이 모듈이 영상을 보고 얻은 값이 아니다. 별도 실제 오차 검증이 필요하다.

**위치 오차 상한 15mm와 재투영 오차 상한 6mm는 이동/관측 계획 데모의 가정이며, LAB 디스크 중심 오차 2mm 미만 요건에 적합한 정렬 기준이 아니다.** 이 게이트를 통과했다고 디스크를 성공적으로 배치했거나 득점했다고 처리하면 안 된다. 별도 근접 정렬·센서 오차 예산·물체 완전 수용·분리 확인이 필요하다.

품질을 통과한 관측 중 가장 최근 시각을 우선하고, 같은 시각이면 작은 오차 상한과 높은 confidence를 우선한다. 여러 카메라의 값을 평균내거나, 한 로봇을 두 번 세거나, 다른 색 물체를 로봇처럼 선택하지 않는다. 한 관측원의 프레임에 중복·미등록·잘못된 로봇/보정이 있으면 그 관측원의 해당 결과를 비운다. 다른 유효 관측원은 계속 사용할 수 있다.

선택 전에 현재 신선한 관측원끼리 같은 로봇의 위치도 비교한다. 두 위치의 거리가 `오차상한 A + 오차상한 B + max_displacement_speed_mm_s × 촬영시각 차이`를 넘으면 `observation_conflicts`에 기록하고 그 로봇은 선택하지 않는다. 따라서 동시에 약 1m 떨어진 좌표를 보고한다고 confidence가 높은 쪽을 믿거나 중간 좌표를 만들지 않는다. 한 로봇만 충돌해도 전체 관측이 불완전하므로 `stop_required=true`다. 오차 범위와 가능한 이동량 안의 비동기 관측은 비교 후 최신 측정을 선택한다.

`max_displacement_speed_mm_s`의 기본 1500mm/s는 모의 가정이며 설정 범위는 0~5000mm/s다. 실제 장비 연결 때 실측 최대 속도로 제한해야 한다. 이 검사는 현재 freshness 범위 안의 **위치** 일관성 검사로, 각도·렌즈 왜곡·카메라 오차를 해결하는 융합기가 아니다. 정상적인 새 관측이 서로 일치하거나 비교 상대가 만료·종료되면 해당 현재 모순은 사라진다. 만료된 관측을 새로운 위치나 비교 증거로 되살리지는 않으며, 과거 모순의 별도 영구 latch/운영자 해제 정책은 구현하지 않는다. 하드웨어 출력은 여전히 비활성이다.

이미 더 최신 관측이 있었던 로봇은 그 관측원이 고장 났다고 이전 관측원의 오래된 좌표로 되돌리지 않는다. 새 프레임이 들어올 때까지 누락으로 표시한다. 늦게 도착한 역순·중복 프레임은 현재의 최신 결과를 덮어쓰지 않는다. `source_closed`는 해당 session을 잠그며, 재접속은 `restart_source(source_id, 새_session_id, now_s=...)`로 명시해야 한다. 이전 session ID를 다시 활성화하지 않는다.

출력 `selected_poses`에는 각 로봇이 최대 한 번 등장하며, 선택한 source/session/sequence, 좌표·각도, age·quality가 포함된다. `selected_sources`, `missing_robot_ids`, `source_status`, `observation_conflicts`도 제공한다. 전체 등록 로봇이 충족되지 않으면 `observation_usable=false`, `stop_required=true`다. `hardware_ready`, `motion_permitted`, `device_io`는 항상 false다.

이 선택기는 **Kalman/EKF 융합기 또는 PoseTracker 자체가 아니다.** 출력에 속도를 임의로 0으로 채우지 않으며, 프레임별 timestamp를 공통 최신 시각으로 위장하지 않는다. 폐루프 제어기에 연결하려면 선택된 측정의 시각·출처를 보존한 추적/상태 추정 어댑터가 별도로 필요하다. 다른 컴퓨터의 monotonic 시각을 이름만 같게 해서 합칠 수 없으며 실제 시계 동기화도 별도 구현이다.

## 시험 범위와 미구현

시험은 높이에 따른 가림 변화, 화각·yaw·픽셀 크기, 일부 표식 가림, padded flight corridor, 속도 제한·hysteresis·목표 필요도, per-frame 보정 검사, 여러 관측원 보완, 한 ID 한 관측, timestamp 역행·session 종료, 5·6·12대 registry와 관측 단절 대기를 다룬다.

실제 영상 획득, 동적 기준점 검출, 기체 비행·풍압, propeller 안전, 자동 이착륙, 무선/펌웨어 watchdog, 실제 주행·집기 성공, 대회 규정 적합성은 이 모듈로 검증하지 않았다. 실제 장비 연결은 다음 단계로 유지한다.
