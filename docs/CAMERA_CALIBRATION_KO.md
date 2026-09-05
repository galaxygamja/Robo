# M2 실제 영상 입력과 경기장 좌표 보정

이 모듈은 실제 웹캠·USB 카메라와 저장된 영상 파일에 사용하는 Python 코드다.
입력 영상에서 네 모서리를 지정하면 경기장 평면을 위에서 본 형태로 펴고,
픽셀 좌표를 mm 좌표로 변환한다. 모터 명령을 보내는 기능은 다음 단계다.
한 실행은 카메라 한 대만 처리한다. 고정 카메라 2대를 사용할 때 필요한
카메라별 보정 관리, 프레임 시간 동기화와 겹치는 관측의 융합은 아직 구현하지 않았다.

## 1. 설치와 실행 위치

저장소 루트의 PowerShell에서 실행한다. Python 3.11 이상이 필요하다.
가상환경이 없다면 먼저 `python -m venv .venv`를 실행한다.

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[vision]"
.\.venv\Scripts\python.exe -m robo_control.vision --help
```

OpenCV와 NumPy는 영상 기능에만 필요한 선택 의존성이다. 기존 기본 패키지는
이 라이브러리가 없어도 import·실행할 수 있다.

## 2. 카메라 보정

카메라를 고정한 뒤 경기장의 네 모서리가 모두 보이도록 한다. 카메라 인덱스가
0이 아니면 해당 번호를 사용한다. 보정은 카메라에서 얻은 한 프레임을 고정해 수행한다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision calibrate --camera 0 --output recordings/camera.json --preview-output recordings/calibrated.png
```

열린 창에서 **경기장 기준 좌상(TL) → 우상(TR) → 우하(BR) → 좌하(BL)**를
순서대로 클릭한다. 네 점을 찍고 Enter를 누르면 JSON이 저장된다.
R은 점 선택 초기화, Esc/Q 또는 창 닫기는 취소다. 좌상단은 경기장의 고정된
방향을 뜻하므로 카메라를 돌렸을 때도 같은 물리적 모서리를 지정해야 한다.
자동 정렬로 모서리 의미를 추정하지 않는다. 거꾸로 도는 순서·교차·중복점은 거부하지만,
네 점 전체를 한 칸씩 잘못 지정한 경우는 독립 기준점을 확인해야 발견할 수 있다.

정지 사진이나 녹화 영상의 첫 프레임으로도 같은 보정값을 만들 수 있다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision calibrate --image recordings/field.jpg --output recordings/camera.json
.\.venv\Scripts\python.exe -m robo_control.vision calibrate --video recordings/field.mp4 --output recordings/camera.json
```

GUI 없이 실행하려면 여덟 개의 픽셀 숫자를 `--corners`로 전달한다. 아래 값은
640×480 시험 영상의 예시일 뿐 실제 카메라 보정값이 아니다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision calibrate --image recordings/field.jpg --corners 80 40 560 60 600 440 40 420 --field-size-mm 1143 1181 --output recordings/camera.json
```

기본 경기장 크기는 저장소의 예선 설정과 같은 1143×1181mm이며,
실측한 외곽 크기가 다르면 `--field-size-mm 폭 높이`로 지정한다.
JSON에는 원본 영상 해상도, 명명된 네 모서리, 경기장 크기, 좌표계와 버전이 담긴다.
카메라 위치·방향·해상도·크롭·렌즈 설정을 바꾸면 재보정해야 한다.

## 3. 실제 영상 처리

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision run --camera 0 --calibration recordings/camera.json --frames 300 --preview --report recordings/frames.jsonl --output recordings/last-rectified.png
```

`--frames`는 읽을 최대 프레임 수이고 기본 100이다. Q/Esc로 미리 종료할 수 있다.
`--output`에는 마지막으로 정상 처리한 프레임을 저장하며 원본 영상은 수정하지 않는다.
영상 파일은 원래 재생 속도를 기다리지 않고 디코딩하며 끝에 도달하면 종료한다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision run --video recordings/field.mp4 --calibration recordings/camera.json --frames 100 --report recordings/replay.jsonl --output recordings/replay.png
```

처리 성공/거부 수와 이유를 표준 출력 JSON으로 반환한다. 종료코드 0은 적어도
한 프레임 처리 및 거부 없음, 1은 프레임 거부/빈 입력/카메라 읽기 실패,
2는 잘못된 인자·보정·파일·의존성 오류다. OpenCV는 파일 끝과 디코딩 실패를
같은 읽기 실패로 반환하므로 파일 종료 사유는 `eof_or_decode_failure`로 기록한다.
손상된 파일의 완전한 디코딩을 종료코드 0만으로 보증하지 않는다.

## 4. 좌표와 시간 계약

- 입력 픽셀: 왼쪽 위 원점, x 오른쪽, y 아래.
- 경기장 좌표: 왼쪽 아래 원점, x 오른쪽, y 위, **mm**.
- 보정 영상 픽셀: 다시 왼쪽 위 원점이다. 변환된 mm 값과 혼용하지 않는다.
- `pixels_per_mm=1`이면 1mm당 1픽셀이다. 출력 크기는 각 축
  `ceil(경기장 mm × pixels_per_mm) + 1`로, 양끝 경계를 포함한다.

```python
from robo_control.vision.calibration import FieldCalibration

calibration = FieldCalibration.load("recordings/camera.json")
xy_mm = calibration.pixel_to_field_mm([[320, 240]])
pixel_xy = calibration.field_mm_to_pixel(xy_mm)
display_xy = calibration.field_mm_to_rectified_px(xy_mm, pixels_per_mm=1.0)
```

기존 `models.Point`와 `RobotCommand.target`은 **m**이므로 전달 경계에서
mm 값을 1000으로 나눠야 한다. 예선 조작 코어의 좌표는 mm다.
현재 패키지에는 공통 `WorldState`가 없고, 보정 결과를 임무 엔진에 연결하는
관측 모델은 로봇/물체 검출 및 추적 단계에서 만든다.

`CameraFrame`은 호스트의 단조 시계로 읽기 시작 `captured_at_s`, 읽기 완료
`received_at_s`를 기록하고, 처리 결과는 별도로 `processed_at_s`를 기록한다.
이름이 captured이더라도 현재 `timestamp_basis=host_read_start`는 **센서 노출시각이 아니다**.
동기식 읽기 대기·처리·소비 지연은 검사할 수 있지만 카메라/드라이버 안의
기존 버퍼 지연은 이 값으로 측정할 수 없다. 현재 단계는 종단 지연 200ms를
실측하여 보증한 상태가 아니다. 향후 카메라 선택 후 버퍼 동작과 노출시각을 검증한다.

녹화 프레임은 `is_replay=true`이고 `media_time_s`는 파일의 별도 시간축이다.
재생 파일이 과거에 촬영되었다는 이유로 호스트 시계와 비교하지 않는다.
현재 예선 엔진은 경기 시작부터의 경과시간을 쓰므로 향후 연결 시
`관측 monotonic - 경기 시작 monotonic`으로 변환해야 한다.

`FrameProcessor`는 기본 200ms보다 오래된 프레임을 변환 전후 모두 거부한다.
중복/역순 프레임 번호, 역순/미래/비정상 시각, 영상원 변경, 잘못된 이미지와
해상도 불일치도 거부한다. 재연결·영상원 교체 시 새 processor를 만든다.
미리보기·파일 작업 때문에 지연이 생기면 로그로 확인할 수 있다.

## 5. 실측 정확도 확인

보정에 쓴 네 점의 오차만 재면 정확도를 과대평가한다. 경기장 내부 여러 곳에
독립적으로 실측한 기준점을 두고, 그 점의 원본 픽셀과 실제 mm 좌표를 입력한다.
예를 들어 다음 형식으로 `recordings/check-points.json`을 만든다(숫자는 예시).

```json
{
  "pixel_points": [[160, 320], [400, 180], [500, 300]],
  "expected_mm": [[220, 300], [760, 800], [950, 380]]
}
```

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision check --calibration recordings/camera.json --points recordings/check-points.json --max-error-mm 15
```

RMS 및 최대 오차를 mm로 출력하며 최대 오차가 기준보다 크면 종료코드 1이다.
15mm는 초기 위치 추적 목표이고 포획·정밀 배치의 합격 기준을 뜻하지 않는다.
렌즈 왜곡 보정, 바닥보다 높은 로봇 태그의 시차 보정, 자동 모서리 검출은
이번 구현에 포함하지 않았다. 특히 광각 카메라는 독립 기준점 오차를 확인한 뒤
필요하면 렌즈 보정 단계를 추가해야 한다.

## 6. 검증과 다음 구현

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

시험에는 독립적인 투영식으로 만든 내부 좌표 정답, 비대칭 색 마커 영상,
실제 파일 코덱으로 기록·재생하는 짧은 영상, 카메라 실패·EOF·자원 해제,
프레임 지연과 순서 오류를 포함한다. 합성 시험 입력의 성공과 실물 카메라
정확도 측정 결과는 구분한다. CI에는 영상 의존성을 설치하는 Windows/Linux
작업을 추가했고, 기본 의존성 없는 시험도 유지한다.

다음 단계인 AprilTag ID·위치·방향 검출은 `robo_control.vision.tags`와
`detect` 명령으로 구현했다. 사용법은 [AprilTag 검출 설명](APRILTAG_TRACKING_KO.md)을
따른다. 그 뒤 색상 물체 검출과 추적, 명령·텔레메트리 계약, ESP32 한 대
폐루프로 연결한다. 고정 카메라 두 대를 실제로 연결하기 전에는 카메라별 단일
영상 정확도를 먼저 통과시키고 동기화·관측 융합 시험을 별도 구현한다.

API 기준 문서: [OpenCV 원근변환](https://docs.opencv.org/4.x/da/d54/group__imgproc__transform.html),
[OpenCV 영상 입력](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html).
