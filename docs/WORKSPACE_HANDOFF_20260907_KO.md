# 작업 환경 및 채팅 맥락 인계

확인일: 2026-09-07. 작업 폴더: `C:\Users\User\projects\Robo`.

## 가져온 맥락

- 같은 프로젝트의 다른 채팅 `Robo Git 저장소 확인`(01a06c03-6842-7ec2-bdf0-6b3b59d7591e)을 조회했다. 최근 일부 턴은 조회 API에서 본문이 비어 있어, 확인 가능한 이전 대화와 최신 저장소 인계 문서로 보완했다. 숨겨진 메모리 전체를 복사한 것은 아니다.
- 기준 문서: `ROBOTICS_PROJECT_FULL_RECORD_KO.txt` 상단의 2026-09-07 인계, `docs/SOFTWARE_PLAN_KO.md`, `docs/REAL_CAMERA_RUNTIME_KO.md`, `docs/LENS_WORK_20260907_KO.md`.
- 한국 예선, 지상 4대: H1 햄스터, H2/B1/B2 비버. H2는 세 번째 비버이며 태그 ID 1을 유지한다. 옛 6대 구성은 이전 실험 기록이다.
- 실제 영상 입력, 경기장/렌즈 보정, AprilTag, HSV, 로봇/물체 연속 추적, 카메라→목표 속도 계산 runtime은 구현됐다. 기존 기능을 중복 구현하지 않는다.
- 실제 모터 출력은 미연결이다. 실물 보정·장치 사양·펌웨어 정지·실제 통신·임무/경로 통합 및 다중 카메라 융합은 남아 있다. 시뮬레이션 160점/45.18초는 실물 결과가 아니다.
- 같은 폴더에서 다른 채팅이 계속 작업 중이다. 확인 도중 `docs/DEVICE_COMMAND_SPEC_KO.md`와 `config/device_spec.json`이 추가됐다. 새 문서에 따르면 2륜 차동구동 확정, 엔코더 미장착 가능성은 높지만 미확정, 보드/모터/휠 사양은 미정이다. 이 파일들은 다른 채팅의 진행 중 변경으로 보존했다.

## Git 확인

- origin: `https://github.com/galaxygamja/Robo.git`.
- 시작 시 main과 원격 main은 `79db65c4ac0c6b3ba3f5e69faeb12b39959b228c`로 일치했다. 원격은 `git ls-remote`로 직접 확인했다.
- 시작 시 작업 트리는 깨끗했다. 이후 다른 채팅의 문서/규격 변경이 생겼으므로 종료 시 전체 작업 트리는 깨끗하지 않을 수 있다.
- 기존 브랜치와 `pre-camera-calibration local scaffold backup 2026-09-05` stash를 보존했다. 이번 환경 준비에서는 브랜치 전환, commit, push, pull을 하지 않았다.

## 환경과 검증

- 기존 `.venv`: Python 3.12.5, OpenCV 5.0.0(ArUco 사용 가능), NumPy 2.5.2. `pip check` 통과.
- Node.js 24.15.0. 웹의 요구 버전은 22.13.0 이상이다.
- 웹 의존성을 기존 `pnpm-lock.yaml` 기준 `install --frozen-lockfile`로 설치했다. pnpm 11.19.0의 설치 스크립트 승인 절차에 따라 esbuild, sharp, workerd 세 패키지의 설치를 완료했다.
- Python 전체 회귀시험: 348개 통과. 웹 로직 시험: 61개 통과.
- Python 합성 예선 데모 실행: 160점, `device_io=false`.
- 최초 샌드박스 실행은 임시 폴더 접근/하위 프로세스 생성 제한으로 실패했다. 동일 시험을 제한 밖에서 실행하여 통과했다. 최초 실행이 tests 아래 남긴 임시 폴더 3개는 정리했다.
- 결과 로그는 Git 제외 경로 `recordings/environment-python-tests-20260907-unrestricted.log`, `recordings/environment-web-tests-20260907.log`에 있다.
- 실제 카메라와 모터 시험, 웹 배포는 이번 점검 범위에 포함하지 않았다.

## 이어서 실행

프로젝트 루트 PowerShell:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -q
.\.venv\Scripts\python.exe -m robo_control.qualifier --compact
.\.venv\Scripts\python.exe -m robo_control.runtime --help
node --experimental-strip-types --test web_simulator/tests/*.test.ts
```

웹 개발 서버는 `web_simulator` 폴더에서 `pnpm dev`를 사용한다. 이 컴퓨터의 pnpm 래퍼는 CMD를 호출하므로 필요하면 다음처럼 직접 실행한다:

```powershell
Set-Location C:\Users\User\projects\Robo\web_simulator
node node_modules/vinext/dist/cli.js dev
```

다음 기능 작업 전에는 Git 상태와 `docs/DEVICE_COMMAND_SPEC_KO.md`를 다시 확인한다. 동시에 진행되는 다른 채팅의 변경을 포함해 임의로 커밋하지 않는다.
