# 실제 카메라 렌즈 왜곡 보정

2026-09-07 후속 구현. 모터·드론·시뮬레이션 동선과 독립된 실제 영상 처리 코드다.
카메라 사진 수집, 렌즈 파라미터 추정, 경기장 좌표 변환 연결을 제공한다.
**실제 카메라의 위치 정확도를 측정한 상태는 아니다.**

## 1. 두 보정의 역할

렌즈 왜곡은 직선이 휘어 보이는 효과이고, 원근 보정은 비스듬히 본 경기장 평면을
펴는 변환이다. 네 모서리만 맞춘 원근 변환으로 렌즈 왜곡 전체를 보정할 수는 없다.
이 구현은 OpenCV의 표준 pinhole/Brown5 모델을 사용한다. 계수 순서는
`[k1, k2, p1, p2, k3]`이며 fisheye·rational·thin-prism 모델은 거부한다.
[OpenCV 카메라 모델/API](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html).

```text
같은 카메라/렌즈/초점/해상도로 체커보드 사진 수집
                  ↓
렌즈 내부 파라미터 + 별도 보정 검사용 사진 확인
                  ↓
고정 카메라에서 원본 경기장 네 모서리 선택 (--lens)
                  ↓
원본 픽셀 → 렌즈 보정 좌표 → 경기장 mm
                  ↓
독립 실측 기준점 확인 → 기존 태그/색 검출/runtime 사용
```

체커보드는 위치·거리·기울기를 바꾸어 찍는다. 한 장을 복사하거나 보드를 같은 방향으로
조금씩 평행 이동한 사진만으로는 충분하지 않다. OpenCV도 여러 위치·방향의 패턴을
사용한 보정을 안내한다. [공식 보정 안내](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html).
이 도구는 최소 12장, 권장 24장으로 시작하며 일부 사진은 파라미터 계산에서 제외해 검사용으로 남긴다.

## 2. 준비와 원본 사진 수집

Python 3.11 이상과 기존 `.[vision]` 의존성이 필요하다. 예시 보드는 **내부 교차점
가로 7개/세로 5개**이므로 사각형 칸은 8×6개다. 칸 수와 내부 교차점 수를 혼동하지 않는다.
인쇄판을 평평한 단단한 판에 붙이고, 잘린 모서리 없이 화면 중앙·네 방향 가장자리·다른 기울기를
골고루 담는다. 칸의 실제 한 변 길이는 자로 재고 이후 `--square-size-mm`에 입력한다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision.lens_capture --camera 0 --camera-label "camera-A fixed-focus 640x480" --inner-corners 7 5 --output-dir recordings/lens-photos-A --count 24
```

- `--output-dir`은 **없는 새 폴더**여야 한다. 사진과 `capture.jsonl`을 저장한다.
- 보드가 검출된 것을 보고 Space를 눌러 저장한다. Q/Esc/창 닫기는 수집 종료다.
- 미리보기만 화면에 맞게 축소한다. 파일은 주석이나 축소가 없는 원본 해상도 PNG다.
- 너무 작은 보드, 누락, 거의 같은 자세, 중복 프레임은 저장하지 않는다. 카메라 출처/
  재생 모드/해상도가 바뀌면 세션을 닫는다. 중간 종료해도 저장한 사진은 보존한다.
- 기본 수집 제한은 600초다. 최소 12장 미만 또는 읽기 실패는 종료 코드 1, 설정 오류는 2다.
- 오프라인 사진 선택의 호스트 나이 제한은 기본 1초이며 `--max-photo-age-ms`로 최대
  2초까지 지정할 수 있다. **runtime의 200ms 관측 만료 기준과는 별개**다.
- 수집 GUI는 동기식 카메라 읽기를 사용한다. 멈춘 드라이버에 대한 종료 지연을 보장하지 않는다.
  실시간 로봇 운용에는 프로세스가 분리된 `robo_control.runtime`을 사용한다.

자동초점·줌·전자 흔들림 보정·자동 크롭/회전 설정은 수집부터 실제 운용까지 동일하게 유지해야 한다.
도구가 카메라의 초점이나 ISP 설정을 잠그지는 않는다. `camera-label`은 **수동 식별 메모**이지
USB 시리얼 인증이 아니다. 같은 USB 인덱스에 다른 카메라를 꽂으면 자동으로 판별하지 못한다.
외부 앱으로 찍은 사진도 사용할 수 있지만 리사이즈·크롭·회전·이미지 보정이 없는 동일 카메라
원본을 사용한다. 휴대폰 사진으로 USB 카메라의 렌즈 값을 대신 구하면 안 된다.

## 3. 렌즈 값 구하기

아래 24mm는 예시다. 실제 인쇄한 칸 길이로 바꾼다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision.lens_calibrate --images recordings/lens-photos-A --inner-corners 7 5 --square-size-mm 24 --camera-label "camera-A fixed-focus 640x480" --output recordings/lens-A.json --report recordings/lens-A-fit.json
```

폴더는 바로 아래 PNG/JPG/JPEG/BMP만 읽는다. 여러 파일 경로를 명시적으로 나열할 수도 있다.
파일 수는 12~100개, 파일당 50MB/16MP 이하다. 출력과 보고서는 새 파일이어야 하며
입력 이미지나 기존 결과를 덮어쓰지 않는다. 미검출 사진은 보고서에 남기고, 실제 검출된
서로 다른 보드 사진이 최소 12장 있어야 한다. 해상도 혼합·파일 내용 중복은 거부한다.

검출 순서의 매 6번째 사진을 검사용으로 남기므로 24장 중 20장으로 렌즈 값을 구하고
4장으로 검사한다. 검사용 사진에서는 렌즈 값을 다시 맞추지 않고 보드 위치/자세만 구한다.
최종 보고서에는 사진별 SHA-256, 원본 교차점 좌표, 계산/검사 구분, 사진별 RMS와 최악 교차점
오차, 화면 범위, 추정된 보드 기울기 다양성을 기록한다.

기본 품질 기준은 사진별 RMS 1px 이하 및 모든 교차점 오차 3px 이하다.
`--max-view-rms-px`로 RMS를 바꾸면 개별 교차점 한계는 그 값의 3배가 된다(허용 RMS 최대 2px).
화면 양축의 절반 이상을 포함하고 보드 추정 법선 사이 최대 각도가 10도 이상이어야 한다.
실패 사진을 자동으로 제외해서 통과시키지 않는다. 보고서가 좋다는 이유로 결과를 실물 정확도
합격으로 바꾸지 않으며, 왜 실패했는지 확인하고 새 사진을 수집한다.

수치 검사에는 역변환 잔차, radial 변환 단조성, 화면 표본 지점의 Jacobian 검사도 포함한다.
이들은 잘못된 추정값을 일부 차단하는 검사이지 전체 광학계의 정확도/전역 가역성 인증이 아니다.

## 4. 경기장 보정에 포함하기

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision calibrate --camera 0 --lens recordings/lens-A.json --output recordings/field-A.json
```

경기장 TL→TR→BR→BL 순서로 **원본 영상 위에서** 네 점을 찍는다. `--corners` 숫자도
원본 픽셀 기준이다. 미리 렌즈 보정한 이미지의 좌표를 주면 이중 보정되므로 사용하지 않는다.
렌즈와 경기장 프로파일의 해상도는 정확히 같아야 한다. 자동 확대/축소는 하지 않는다.

- 기존 렌즈 없는 field schema 1은 그대로 동작한다.
- `--lens`를 사용하면 field schema 2에 렌즈 프로파일을 **내장 복사**하고
  `pixel_geometry=raw_camera_pixels`를 저장한다. 나중에 외부 lens 파일을 수정해도 이 field 값은 바뀌지 않는다.
- schema 1에 lens 값을 몰래 추가하거나, schema 2에서 lens/원본 픽셀 표시를 빼면 거부한다.
- schema 2는 이전 코드가 지원하지 않으므로 팀 전체가 같은 코드 버전을 사용한다.
- `pixel_to_field_mm()` 입력과 `field_mm_to_pixel()` 출력은 계속 원본 픽셀이다.
  태그 검출은 원본에서 하고 코너 좌표를 보정한다. 심한 왜곡 때문에 태그 자체를 읽지 못하는
  문제까지 해결됐다는 의미는 아니다.
- 색 검출용 경기장 영상은 원본에서 직접 역매핑한다. 중간 보정 영상의 밖으로 이동한
  모서리가 잘리는 문제를 피하며, 맵은 제한된 크기로 캐시한다. 보정 출력은 최대 16MP다.

## 5. 독립 기준점과 runtime

보정에 사용하지 않은 경기장 내부 실측점들의 **원본 픽셀/mm** 좌표를 준비한다.
기존 [기준점 파일 형식](CAMERA_CALIBRATION_KO.md)을 따른다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.vision check --calibration recordings/field-A.json --points recordings/check-A.json --max-error-mm 15
.\.venv\Scripts\python.exe -m robo_control.runtime --camera 0 --calibration recordings/field-A.json --fleet config/qualifier_senior.json --report recordings/runtime-lens-A-001.jsonl --duration-s 120
```

runtime은 내장 렌즈 값을 별도 영상 프로세스까지 그대로 전달한다. 검출 행의
`lens_correction_applied`, `lens_calibration_id`와 시작 행의 전체 보정값으로 사용한 설정을 확인한다.
모든 출력은 여전히 검증용 명령/로그이며 실제 모터에는 전달하지 않는다.

**15mm는 초기 위치 추적 검사 예시이지 디스크 정밀 배치 합격치가 아니다.** 체커보드 재투영
오차(px)와 경기장 위치 오차(mm)도 같지 않다. 렌즈 보정을 해도 바닥보다 높은 로봇 태그의
시차, 휘어진 보드/경기장, 움직이는 카메라, 센서 버퍼 지연, 흔들림은 남는다. 실제 장착 높이·
렌즈/초점·조명에서 별도 실측 검사를 해야 하며, 이동 카메라 동적 보정은 미구현이다.

## 6. 소프트웨어 시험

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_lens*.py" -v
.\.venv\Scripts\python.exe tools/verify_camera_runtime.py --output-dir recordings/lens-check-001 --seconds 120 --lens-fixture --colors-fixture
```

단위 시험은 독립 3D 투영, 보류 사진의 오류 주입, 왜곡 영상의 실제 디코딩/체커보드/태그 검출,
원본 사진 저장, schema 호환성과 거부를 확인한다. 두 번째 명령은 생성한 왜곡 영상에서
4대 태그·색상 추적·반복 가림·정지/재확인을 시험한다. 이 입력은 실제 카메라로 촬영한 것이 아니므로
실측 카메라 정확도나 로봇 주행 시험으로 발표하면 안 된다.
