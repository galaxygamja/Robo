# M3 실제 영상의 로봇 AprilTag 검출

이 모듈은 보정된 고정 카메라에서 지상 로봇 `H1`, `H2`, `B1`, `B2`의
태그 ID, 태그 중심, 로봇 회전중심과 방향을 측정한다. 검출은 원본 영상에서
수행하고, 태그 코너를 M2 보정값으로 경기장 평면의 mm 좌표로 변환한다.

현재 결과는 프레임별 **관측값**이다. 여러 프레임을 잇는 추적, 가림 예측,
ESP32 명령 송신과 모터 제어는 아직 연결하지 않았다.
`detect` 한 실행은 카메라 한 대만 처리한다. 고정 카메라 2대의 시간 동기화,
겹치는 로봇 관측 선택과 융합은 별도 단계다.

## 1. 태그 설정

기본 설정은 [config/robot_tags.json](../config/robot_tags.json)이다.

| 태그 ID | 로봇 |
|---:|---|
| 0 | H1 |
| 1 | H2 |
| 2 | B1 |
| 3 | B2 |

검출기는 이 목록을 순회하도록 작성되어 있어 로봇을 추가할 때 검출 코드를
수정할 필요는 없다. 실제 다섯 번째 로봇을 추가하기 전에는 통신·충돌계획과
출발구역 시험도 함께 확대해야 한다.

`dictionary_name`은 기본 `DICT_APRILTAG_36h11`이다. 서로 다른 사전의 같은
숫자 ID는 다른 태그이므로 생성·인쇄·검출 설정을 반드시 일치시킨다.

```powershell
.\.venv\Scripts\python.exe tools/generate_robot_tags.py --config config/robot_tags.json --output-dir recordings/printable-tags
```

이 명령은 검출기와 같은 OpenCV 버전·사전에서 각 로봇의 **흰 quiet zone이
포함된 `_printable.png`**와 `ROBOT FRONT / MARKER +X` 화살표가 있는 방향
확인용 PNG를 만든다. 실제 인쇄에는 `_printable.png`를 사용하고 흰 여백을
자르지 않는다. 기존 파일은 `--overwrite` 없이는 덮어쓰지 않는다. OpenCV는 AprilTag 코너 및 생성 이미지
방향이 다른 AprilTag 도구와 다를 수 있다고 공식 문서에서 경고한다. 인터넷에서
받은 태그 PNG와 생성 PNG를 섞지 말고, 실제 인쇄물의 화살표 방향을 카메라
미리보기에서 확인한다.

현재 `tag_size_mm`은 `null`이다. 로봇에 붙이는 **검은 태그 사각형 한 변**을
정한 뒤 실제 인쇄물을 자로 재서 mm 값을 넣는다. 주변 흰 여백이나 카드 전체
크기는 포함하지 않는다. 값이 있으면 검출 크기가 설정값에서 기본 35% 이상
벗어난 관측을 `tag_size_mismatch`로 제외한다. 태그 높이 효과가 아직 보정되지
않았으므로 실제 위치별 측정을 마치기 전에는 허용범위를 성급히 줄이지 않는다.

`heading_offsets_rad`는 `로봇 방향 - 태그 +X 방향`이다. 태그 +X는 OpenCV가
반환하는 canonical corner 0에서 corner 1로 향한다. 생성된 태그를 바로 놓으면
화면상 위쪽 변의 왼쪽에서 오른쪽 방향이다. 태그를 차체에 돌려 붙였다면 해당
로봇의 보정각을 rad로 기록한다.

`robot_center_from_tag_mm`은 태그 중심에서 실제 로봇 회전중심으로 향하는
벡터다. `forward_mm`은 최종 로봇 진행방향, `left_mm`은 로봇 왼쪽 방향이 양수다.
태그를 회전중심에 정확히 붙이면 둘 다 0이다.

`hardware_verified`는 실측 완료 기록용이며 기본값은 `false`다. 이 값이 자동으로
모터 출력을 허용하지는 않는다.

## 2. 실제 사진과 카메라에서 검출

먼저 같은 카메라 위치와 해상도로 경기장 보정을 만든다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision calibrate --camera 0 --output recordings/camera.json
```

정지 사진을 검사하면 실제 로봇을 움직이지 않고 태그 배치와 설정을 확인할 수 있다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision detect --image recordings/field-with-robots.png --calibration recordings/camera.json --tags config/robot_tags.json --report recordings/tag-check.jsonl --output recordings/tag-check.png
```

USB 카메라 300프레임을 검사하고 화면에서 결과를 보려면 다음과 같이 실행한다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision detect --camera 0 --calibration recordings/camera.json --tags config/robot_tags.json --frames 300 --preview --report recordings/tag-live.jsonl --output recordings/tag-live-last.png
```

녹화 영상 회귀시험도 같은 검출 경로를 사용한다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision detect --video recordings/run.mp4 --calibration recordings/camera.json --tags config/robot_tags.json --frames 10000 --report recordings/tag-replay.jsonl
```

`--require-complete-observation`을 사용하면 처리한 모든 프레임에 등록 로봇이
정확히 한 번씩 보이지 않을 때 종료코드 1을 반환한다. 일반 검출 모드는 태그가
없는 프레임도 정상적인 측정 결과로 기록하고 종료코드 0을 반환한다.

## 3. 출력 계약

JSONL의 각 정상 프레임에는 다음 값이 포함된다.

- `tag_center_px`: 원본 영상에서 태그 중심의 투영 좌표
- `tag_center_mm`: 경기장 평면에 투영한 태그 중심
- `robot_center_px`: 장착 오프셋을 반영한 로봇 회전중심의 영상 좌표
- `robot_center_mm`: 제어 연결에 사용할 로봇 회전중심의 경기장 mm 좌표
- `heading_rad`: 경기장 +X가 0, 반시계 방향이 양수인 로봇 방향
- `corners_px`, `corners_mm`: 검출된 태그 코너
- `observed_tag_size_mm`: 네 변 길이의 평균
- `captured_at_s`, `received_at_s`, `processed_at_s`: 호스트 단조 시계의 처리 시각
- `media_time_s`: 녹화 파일에만 있는 별도 재생 위치
- `dictionary_name`, `tag_size_mm`: 해당 실행의 태그 사전과 실측 크기 설정
- `hardware_verified`: 이 카메라·태그 장착 조건의 실물 검증 기록(기본 `false`)

`observation_complete=true`는 해당 프레임에 등록 태그가 모두 정확히 한 번씩
유효하게 보였다는 뜻이다. 카메라 보정·실측·추적·통신·비상정지까지 확인되었다는
뜻은 아니며, 이 값 하나로 모터를 활성화하면 안 된다.

다음 경우 해당 태그 관측을 제어 입력에서 제외한다.

- 등록되지 않은 ID: `unknown_tag`
- 같은 ID가 두 번 검출됨: `duplicate_tag`
- 보정된 로봇 중심이 경기장 밖: `out_of_field`
- 태그 방향 또는 투영이 유효하지 않음: `degenerate_heading`, `invalid_projection`
- 실측 태그 크기와 설정이 크게 다름: `tag_size_mismatch`

누락된 로봇은 `missing_robot_ids`에 들어간다. 추적 계층을 구현하면 마지막
정상 관측 시각부터 500ms 후 해당 로봇을 정지시키는 안전 상태와 연결한다.

## 4. 실물에서 반드시 측정할 것

현재 자동시험은 원근·회전·중복·누락을 넣은 합성 이미지와 실제 PNG/MJPG
디코딩을 사용한다. 실제 카메라의 렌즈 왜곡, 흔들림, 조명, 모션 블러와 태그
인쇄 품질은 검증하지 못한다.

특히 바닥 네 모서리로 만든 평면 변환을 차체 위의 높은 태그에 적용하면,
카메라 중심에서 멀수록 태그 위치가 실제 바닥 접점과 어긋날 수 있다. 다음
실물 시험에서 오차를 측정한 뒤 렌즈 보정과 태그 높이 보정을 추가한다.

1. 네 로봇의 인쇄 태그 ID와 전방 화살표를 눈으로 대조한다.
2. 각 로봇을 경기장 내부 최소 9개 지점에 놓고 회전중심을 실측한다.
3. 각 지점에서 0°, 90°, 180°, 270° 방향을 확인한다.
4. 위치 오차 15mm 이하, 방향 오차 5° 이하인지 기록한다.
5. 100프레임 중 99프레임 이상에서 필요한 태그가 검출되는지 조명별로 확인한다.
6. 태그 하나를 가리고 `missing_robot_ids` 및 이후 500ms 정지 동작을 확인한다.

실측 전에는 `hardware_verified=false`를 유지한다. 합격한 카메라 위치·해상도,
태그 크기, 장착 오프셋과 조명 조건을 함께 기록해야 결과를 재현할 수 있다.

공식 API 근거: [OpenCV ArUco·AprilTag 검출](https://docs.opencv.org/5.0/main_modules/objdetect_aruco.html),
[OpenCV ArucoDetector](https://docs.opencv.org/5.0/main_modules/classcv_1_1aruco_1_1ArucoDetector.html).
