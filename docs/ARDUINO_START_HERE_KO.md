# Arduino IDE로 로봇 보드에 코드 넣기

대상: **햄스터 H1 1대 + 비버 B1/B2/B3 3대**, 사용자가 확인한 **ESP32-C3 SuperMini** 보드. 이번 수정은 Arduino 보드에서 실행하는 펌웨어입니다. 기존 Python 영상 처리·서버 코드는 바꾸지 않습니다.

## 1. “void”가 무엇이고 무엇을 열어야 하나?

Arduino에서 사용하는 언어는 **C/C++**입니다. `void`는 언어 이름이 아니라 함수가 값을 돌려주지 않는다는 뜻입니다. `setup()`은 부팅 후 한 번 실행하고, `loop()`는 이후 계속 반복하는 함수입니다. [Arduino setup 설명](https://docs.arduino.cc/language-reference/en/structure/sketch/setup/)과 [loop 설명](https://docs.arduino.cc/language-reference/en/structure/sketch/loop/)에서 기본 구조를 볼 수 있습니다.

이번 파일도 다음 구조를 사용합니다. 아래 짧은 예제만 새 파일로 복사하지 말고, 실제 배포 폴더 전체를 사용하세요.

```cpp
void setup() {
  roboSetup();
}

void loop() {
  roboLoop();
}
```

`roboSetup()`과 `roboLoop()`의 실제 모터·서보·Wi-Fi·안전 정지 코드는 함께 들어 있는 `src/`에 있습니다. `.ino` 한 파일에 전부 붙여 넣지 않아도 Arduino IDE가 같이 빌드합니다.

| 파일/폴더 | 역할 | 처음에 할 일 |
|---|---|---|
| `arduino/RoboRobot/RoboRobot.ino` | Arduino IDE에서 여는 시작 파일 | 더블클릭 또는 IDE의 파일 열기 |
| `arduino/RoboRobot/RobotSettings.h` | 올릴 로봇 번호와 출력 허가 | 로봇마다 번호 선택 |
| `arduino/RoboRobot/Secrets.example.h` | Wi-Fi/토큰 양식 | `Secrets.h`로 복사 |
| `arduino/RoboRobot/Secrets.h` | 본인의 비공개 연결 설정 | Wi-Fi/토큰 입력; 공개 금지 |
| `arduino/RoboRobot/src/` | 실제 C++ 제어 코드 | 함께 보관; 임의로 삭제하지 않음 |
| `arduino/RoboRobot/build_opt.h` | 빌드 옵션 | 함께 보관 |
| `firmware/` | 같은 코드의 PlatformIO 개발용 원본 | Arduino IDE 사용자라면 직접 편집할 필요 없음 |

`.ino`만 바탕화면으로 꺼내지 말고 `RoboRobot` 폴더 전체를 유지합니다. 폴더 이름과 `.ino` 이름도 그대로 둡니다. GitHub에서 받은 ZIP은 먼저 압축을 풀어야 합니다.

## 2. 보드와 로봇을 정확히 선택하기

Arduino IDE를 쓰지만 **Arduino Uno 보드를 선택하는 것이 아닙니다.** 사용 보드는 ESP32-C3 SuperMini이고 IDE의 보드 항목은 `ESP32C3 Dev Module`입니다.

| 조립할 로봇 | `ROBOT_PROFILE` | 통신에서 쓰는 ID | 바퀴 구동 | 작업용 서보 |
|---|---:|---|---|---|
| 햄스터 H1 | 1 | H1 | SM-S4303R 연속회전 서보 ×2 | MG90S 게이트 ×1 |
| 비버 B1 | 2 | B1 | N20 DC 모터 ×2 + DRV8833 | MG90S 집게/투하부 ×2 |
| 비버 B2 | 3 | B2 | N20 DC 모터 ×2 + DRV8833 | MG90S 집게/투하부 ×2 |
| 비버 B3 | 4 | H2 | N20 DC 모터 ×2 + DRV8833 | MG90S 집게/투하부 ×2 |

세 번째 비버의 통신 이름 `H2`는 기존 태그·서버 연결을 유지하기 위한 별칭입니다. **B3라고 표시된 실물에는 프로필 4를 올리고, 서버 명령에서는 H2를 사용합니다.** 기존 서버 설정과 태그 이름을 무작정 B3로 바꾸지 않습니다.

한 번에 한 보드만 USB에 꽂습니다. 보드와 로봇에 H1/B1/B2/B3 스티커를 붙이고 업로드 기록을 남깁니다. 네 보드에 H1 설정을 그대로 복제하면 안 됩니다.

## 3. 전원을 연결하기 전 배선 확인

아래 번호는 보드의 **GPIO 번호**입니다. 커넥터의 물리적인 몇 번째 핀이라는 뜻이 아닙니다. 좌우는 로봇 뒤에서 전방을 보는 기준입니다.

| GPIO | 햄스터 H1 | 비버 B1/B2/B3 |
|---:|---|---|
| 3 | 왼쪽 SM-S4303R 신호선 | DRV8833 AIN1 |
| 4 | 연결하지 않음 | DRV8833 AIN2 |
| 5 | 오른쪽 SM-S4303R 신호선 | DRV8833 BIN1 |
| 6 | 연결하지 않음 | DRV8833 BIN2 |
| 7 | MG90S 디스크 게이트 신호선 | MG90S 앞 집게 신호선 |
| 10 | 연결하지 않음 | MG90S 뒤 투하 게이트 신호선 |
| 0 | TCRT5000의 디지털 D0 | 연결하지 않음 |
| GND | 모든 서보/센서/조절 전원과 공통 | 드라이버/서보/조절 전원과 공통 |

**햄스터 구동용 SM-S4303R에는 DRV8833가 필요하지 않습니다.** GPIO 신호선, 별도 서보 전원, 공통 GND의 세 연결을 사용합니다. 서보의 선 색만 믿지 말고 구매품 설명의 신호/+/GND를 대조합니다. 비버만 DRV8833 출력 AOUT1/AOUT2에 왼쪽 DC 모터, BOUT1/BOUT2에 오른쪽 DC 모터를 연결합니다.

전원 확인 사항:

- 2S LiPo는 완충 8.4 V입니다. GPIO, 5 V 서보, 6 V DC 모터 또는 보드 USB/5 V 입력에 그대로 넣지 않습니다.
- GPIO는 3.3 V 신호입니다. 모터·서보의 전력을 GPIO나 보드 3.3 V 핀에서 공급하지 않습니다.
- 서보는 구매품 허용 범위 안의 별도 안정된 전원으로 공급합니다. H1은 바퀴 서보 두 개까지 포함하므로 MG90S 하나만의 소비전류로 전원 용량을 정하면 안 됩니다.
- 비버의 DRV8833 VM에는 검증한 모터용 6 V 전원을 공급합니다. `nSLEEP/SLP`는 모듈 회로 설명에 따라 활성화합니다. 임의로 VM에 직결하지 않습니다.
- 전원 모듈이 배터리의 완충 상태뿐 아니라 실제 사용하는 방전 구간 전체를 지원하는지 확인합니다. 부하를 연결했을 때의 전압 강하도 확인합니다.
- 보드 USB 전원과 외부 5 V 전원을 동시에 쓸 때는 실제 SuperMini의 역급전 보호 여부를 확인합니다. 최초 업로드는 모터/서보 전원을 분리하고 보드만 USB로 공급합니다.
- TCRT5000 D0가 5 V로 올라가는 모듈은 GPIO0에 직결하지 않습니다. 3.3 V 호환 출력 또는 적절한 레벨 변환이 필요합니다.
- 현재 배터리 전압 감시선·엔코더는 가정하지 않습니다. 코드가 LiPo 저전압이나 실제 바퀴 회전속도를 자동 측정한다고 생각하면 안 됩니다.

모터·서보 전원을 즉시 끊을 수 있는 별도 스위치/분리 수단을 마련합니다. 소프트웨어 정지 명령만을 유일한 안전 수단으로 사용하지 않습니다.

## 4. Arduino IDE 설치와 버전 설정

1. [Arduino 공식 다운로드](https://www.arduino.cc/en/software)에서 Arduino IDE 2를 설치합니다.
2. IDE의 **파일 → 환경 설정 → 추가 보드 관리자 URL**에 다음 주소를 추가합니다.

```text
https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

3. 보드 관리자를 열고 `esp32`를 검색합니다. **Espressif Systems의 esp32, 버전 2.0.17**을 선택하여 설치합니다.
4. 라이브러리 관리자를 열고 **ArduinoJson by Benoit Blanchon, 버전 6.21.5**를 설치합니다.
5. `RoboRobot.ino`를 엽니다. 도구 메뉴에서 다음 설정을 맞춥니다.

| 항목 | 선택 |
|---|---|
| Board | ESP32C3 Dev Module |
| USB CDC On Boot | Enabled |
| Flash Mode | DIO |
| Port | 현재 연결한 보드의 실제 COM 포트 |
| Flash Size | 구매 보드의 실제 용량과 일치; 임의로 늘리지 않음 |
| Serial Monitor | 115200 baud |

Espressif 공식 [설치 안내](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)와 [ESP32-C3 USB CDC 안내](https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/cdc_dfu_flash.html)를 참고할 수 있습니다. 이 프로젝트는 일반 최신 버전 안내와 달리 재현성을 위해 **코어 2.0.17로 고정**합니다. 3.x 코어에서는 LEDC API가 달라질 수 있으므로 자동 업그레이드하지 않습니다. 다른 코어/칩 선택은 이 펌웨어의 컴파일 오류로 차단합니다.

`Servo.h`를 별도로 설치하거나 예전 Uno용 서보 예제를 붙이지 않습니다. 현재 펌웨어가 ESP32의 PWM 기능으로 구동/작업 서보를 처리합니다.

## 5. 개인 Wi-Fi와 로봇 번호 입력

`RobotSettings.h`의 다음 줄을 해당 로봇에 맞춥니다.

```cpp
#define ROBOT_PROFILE 1
```

H1은 1, B1은 2, B2는 3, B3는 4입니다. `#ifndef`/`#endif` 구조는 그대로 두고 숫자만 바꿉니다. 다른 곳에 같은 정의를 여러 번 만들지 않습니다.

파일 탐색기에서 `Secrets.example.h`를 복사하여 같은 폴더에 **`Secrets.h`**라는 이름으로 만듭니다. 확장자 숨김 때문에 `Secrets.h.txt`가 되지 않았는지 확인합니다. IDE를 다시 열면 탭에 보이며 다음 세 값을 편집합니다.

```cpp
#define WIFI_SSID "본인의_2.4GHz_WiFi_이름"
#define WIFI_PASSWORD "본인의_WiFi_암호"
#define ROBOT_SHARED_TOKEN "직접생성한24글자이상64글자이하의토큰"
```

위 한글은 설명용 자리표시자이며 실제 토큰에 쓰지 않습니다. 토큰은 영문 대소문자·숫자·밑줄·하이픈만 사용한 임의의 24~64자입니다. 서버와 네 로봇의 값이 같아야 합니다. SSID와 암호는 실제 값을 넣으며 Wi-Fi 암호는 최소 8자입니다. 토큰/암호를 GitHub, 채팅, 공개 화면 캡처에 올리지 않습니다. `Secrets.h`는 Git에서 제외됩니다.

전용 WPA2/WPA3 로컬 2.4 GHz Wi-Fi를 사용합니다. 토큰은 UDP 암호화가 아니므로 공용망을 쓰거나 인터넷에 로봇 포트를 열지 않습니다. 서버와 보드가 같은 망에서 서로 통신할 수 있어야 하며 AP 격리 기능이 이를 막는지 확인합니다.

첫 업로드에서는 다음 두 값을 그대로 유지합니다.

```cpp
#define HARDWARE_OUTPUT_ENABLED 0
#define ACTUATOR_CALIBRATION_CONFIRMED 0
```

둘 중 하나라도 0이면 물리 출력은 비활성입니다. 설정을 1로 바꾼 것은 자동 측정 완료를 의미하지 않으며, 다음 단계의 감독된 시험을 허가한 것입니다.

## 6. 첫 업로드: 보드만 USB로 연결

1. 모터/서보 전원을 분리합니다. 데이터 전송 가능한 USB 케이블로 보드 한 대만 연결합니다.
2. 로봇 프로필, 보드 종류, 코어 버전, 포트를 다시 확인합니다.
3. IDE의 **검증(체크 표시)**을 누릅니다. 성공하면 **업로드(화살표)**를 누릅니다.
4. 시리얼 모니터를 115200으로 열고 필요하면 RESET을 한 번 누릅니다.
5. 부팅 로그의 표시 이름, wire ID, firmware 2.0.0, 구동 방식, 출력 `DISABLED`를 확인합니다.
6. Wi-Fi 연결 후 출력되는 실제 IP와 UDP 포트 4210을 기록합니다. 192.168.4.101 등의 예시 주소가 자동으로 배정된다고 가정하지 않습니다.

H1 로그의 구동 방식은 `continuous_servo`, 비버는 `dc_hbridge`여야 합니다. B3는 표시 B3/통신 H2가 맞습니다. **기존 서버는 이 새 메타데이터를 자동 대조하지 않으므로 사람이 확인해야 합니다.** 프로필이 다르면 출력 전원을 연결하지 말고 올바른 번호로 다시 업로드합니다.

포트가 안 보이면 케이블/USB 포트를 확인합니다. 필요한 경우 BOOT를 누른 상태에서 RESET을 눌렀다 떼고 BOOT를 놓은 뒤, 새로 나타난 포트를 선택합니다. 첫 업로드 후 수동 RESET이 필요할 수 있습니다. [Espressif USB CDC 절차](https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/cdc_dfu_flash.html)에 따른 방법이며, 보드의 실제 버튼 표시를 확인하세요.

## 7. 구동 설정의 뜻: 햄스터와 비버가 다르다

| 설정 | 대상 | 기본값/의미 |
|---|---|---|
| `DRIVE_LEFT_DIRECTION` | 전체 | 왼쪽 방향 +1; 실물에 따라 -1 |
| `DRIVE_RIGHT_DIRECTION` | 전체 | H1 -1, 비버 +1; 실제 양수 명령이 전진인지 확인 |
| `DRIVE_LEFT_NEUTRAL_US` | H1 | 왼쪽 중립 시험값 1500 µs |
| `DRIVE_RIGHT_NEUTRAL_US` | H1 | 오른쪽 중립 시험값 1500 µs |
| `DRIVE_LEFT_MIN_US` / `MAX_US` | H1 | 왼쪽 시험 범위 1380~1620 µs |
| `DRIVE_RIGHT_MIN_US` / `MAX_US` | H1 | 오른쪽 시험 범위 1380~1620 µs |
| `TOOL0_MIN_US` / `MAX_US` | 전체 | 작업 서보 0 허용 범위; 기본 900~2100 µs |
| `TOOL1_MIN_US` / `MAX_US` | 비버 | 작업 서보 1 허용 범위; 기본 900~2100 µs |

이 숫자는 **실제 제품/조립 기구에서 측정한 값이 아닙니다.** H1의 1500 µs도 개별 서보에서 정확히 멈추는 값이라고 보장할 수 없습니다. 서보 방향과 중립, 안전한 작업 서보 끝점을 실물에서 좁혀야 합니다. 보정값은 `RobotSettings.h` 또는 비공개 `Secrets.h`에 해당 매크로를 한 번 정의하여 반영하고 다시 업로드합니다. `src/`의 기본값을 직접 바꾸기보다 개인 설정에 보관하는 편이 업데이트 시 확인하기 좋습니다.

H1 바퀴는 연속회전 서보이므로 90도/180도로 돌려 놓는 위치 제어가 아닙니다. 중립보다 높거나 낮은 펄스를 주면 회전 방향과 회전량이 바뀝니다. 서버 명령의 `left_pwm/right_pwm`라는 기존 이름은 유지하지만 H1에서는 부호 있는 -96~96 명령을 중립 주변의 펄스로 변환합니다. 비버에서는 같은 숫자를 DC 모터 PWM으로 사용합니다. **어느 쪽도 mm/s가 아닙니다.** 서버에서 사용할 실제 mm/s 관계는 바닥 부하로 별도 측정합니다.

작업용 `servo_us`는 `[서보0, 서보1]`입니다. H1은 `[게이트, 0]`, 비버는 `[앞집게, 뒤투하부]`이며 H1 바퀴 서보 펄스를 여기에 넣지 않습니다. 값 0은 그 작업 서보의 펄스를 끕니다. 900~2100 전체 범위를 기구에 무작정 명령하지 말고 링크를 분리하여 중앙 부근부터 작은 변화로 확인합니다.

## 8. 기존 서버 연결과 짧은 감독 시험

이번 배포에서 Python 서버 코드는 수정하지 않습니다. 아래는 **이미 있는 도구**를 Arduino 펌웨어 시험에 연결하는 방법입니다. Python 파일을 보드에 올리는 것이 아닙니다. 기존 가상환경을 사용해도 되며, 새 설치가 필요하면 저장소 루트에서 다음을 실행합니다.

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[vision]"
.\.venv\Scripts\python.exe -m robo_control.hardware init --output local_state/hardware-arduino.json
.\.venv\Scripts\python.exe -m robo_control.hardware validate --config local_state/hardware-arduino.json
```

초기 validate가 `motion_ready:false` 또는 종료 코드 2를 내는 것은 미보정 상태라 정상입니다. 이 명령은 실제 보드를 움직이지 않습니다. `hardware-arduino.json`이 이미 있으면 덮어쓰지 말고 다른 이름을 씁니다. 현재 형식은 기존과 같은 **schema_version 1**입니다.

새 파일에 실제 IP/포트를 넣고 실물을 확인한 항목만 표시합니다. `network_confirmed`는 실제 개별 주소/로컬망 확인, `wiring_verified`는 배선·극성·부하 시 전압 확인입니다. 기존 H1 DC 모터용 곡선이나 `motion_calibrated:true`를 복사하지 않습니다. 최초에는 실측 곡선은 빈 상태로 두며, H1 센서 확인 전에는 `sensor_verified`도 false를 유지합니다.

서버 실행 PowerShell의 `ROBO_HW_TOKEN` 환경 변수에 `Secrets.h`와 같은 실제 토큰을 설정합니다. 공개 터미널 캡처나 로그에 남기지 않습니다. 아래는 자리표시자이며 그대로 실행할 값이 아닙니다.

```powershell
$env:ROBO_HW_TOKEN = "Secrets.h와같은실제영문숫자토큰으로교체"
```

출력 시험 전 조건:

1. 실제 로봇 한 대만 시험합니다. 좌우 구동 바퀴를 바닥에서 확실히 띄우고 본체를 안정되게 고정합니다.
2. 손·옷·전선을 바퀴와 기구에서 떼고 즉시 전원을 끊을 수 있게 합니다. 처음에는 작업 서보 링크를 분리합니다.
3. 부팅 로그와 실물 구동 방식, 전원 전압, 낮고 제한된 시험 범위를 확인합니다.
4. 위 조건을 마련한 뒤에만 두 출력 허가 값을 1로 바꾸고 다시 업로드합니다. 두 번째 플래그는 이 단계의 감독된 보정 시험에 대한 확인이지, 전체 주행 보정 완료 선언이 아닙니다.
5. 출력 전원을 연결하고 아래와 같이 짧은 시험을 한 번씩 실행합니다. 뜻을 모르는 자동 반복 실행은 하지 않습니다.

먼저 H1의 논리 정지값을 확인합니다. 이 명령은 양쪽 바퀴에 중립 펄스를 내므로 **중립이 틀리면 회전할 수 있습니다.**

```powershell
.\.venv\Scripts\python.exe -m robo_control.hardware bench --config local_state/hardware-arduino.json --robot H1 --left-pwm 0 --right-pwm 0 --duration-s 0.3 --enable-hardware --wheels-raised
```

멈추지 않는 쪽은 시험을 끝내고 출력 전원을 끈 뒤 해당 중립값을 조금씩 조절하여 다시 업로드/확인합니다. 기계적 중립 조절 기능이 있는 제품은 제조사 절차도 확인합니다. 서보가 펄스 없이도 예기치 않게 움직이면 소프트웨어만 믿지 말고 배선과 제품 동작을 점검합니다.

중립 확인 후 좌우를 따로 낮은 명령으로 시험합니다. 다음의 12는 **방향 확인용 시험 예시**일 뿐, 실측 속도나 항상 회전을 보장하는 숫자가 아닙니다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.hardware bench --config local_state/hardware-arduino.json --robot H1 --left-pwm 12 --right-pwm 0 --duration-s 0.3 --enable-hardware --wheels-raised
.\.venv\Scripts\python.exe -m robo_control.hardware bench --config local_state/hardware-arduino.json --robot H1 --left-pwm 0 --right-pwm 12 --duration-s 0.3 --enable-hardware --wheels-raised
```

비버는 ID를 B1/B2/H2로 바꾸어 같은 방식으로 각각 확인합니다. 낮은 PWM에서 DC 모터가 돌지 않을 수 있지만 곧바로 상한으로 올리지 않습니다. 양수 명령이 실제 로봇 전진 방향인지 확인하고 `DRIVE_*_DIRECTION`을 필요할 때만 반전합니다. 보정을 바꾸면 이전 속도 곡선은 다시 확인해야 합니다.

`servo_presets`에는 직접 측정한 작업용 두 펄스를 이름별로 넣습니다. 예를 들어 H1 `disc_latch_open`에 넣은 값이 실물에서 안전하다고 확인된 뒤에만 다음을 실행합니다. 프리셋 이름만 만들고 실제 성공을 가정하지 않습니다.

```powershell
.\.venv\Scripts\python.exe -m robo_control.hardware bench --config local_state/hardware-arduino.json --robot H1 --servo-preset disc_latch_open --duration-s 0.3 --enable-hardware --wheels-raised
```

## 9. 바닥 주행 전에 별도로 해야 할 것

띄운 바퀴 시험은 방향/중립 확인일 뿐 바닥 속도 보정이 아닙니다. 실측 바퀴 접지 중심 간격 `track_width_mm`, 적재물/열린 기구를 포함한 충돌 반경 `envelope_radius_mm`, 좌우 각각 전진·후진의 네 속도 곡선을 다시 측정합니다. 서버의 `left_forward`, `left_reverse`, `right_forward`, `right_reverse`는 `[명령 크기, 실측 mm/s]` 점들의 배열입니다. 모든 곡선은 `[0,0]`으로 시작하고 이후 측정값이 증가해야 합니다.

H1 곡선의 첫 열도 파일 형식상 PWM이라고 부르지만 실제 의미는 중립 주변 서보 구동 명령 크기입니다. 과거 N20의 PWM 곡선을 H1에 재사용하면 안 됩니다. 양수/음수 방향 모두, 배터리 상태와 적재 조건에서 측정해야 합니다.

기존 영상 로그 보정 도구 `python -m robo_control.hardware_calibration --help`는 그대로입니다. 이번 Arduino 전용 수정에서는 새 `--drive-type` 옵션을 추가하지 않았습니다. 카메라 로그 추정치는 실제 명령 시점/바퀴 미끄러짐 확인을 대체하지 않으며 개인 설정을 자동으로 안전하다고 승인하지 않습니다.

실제 위치 추적·색/태그 검출·경로 계산은 계속 서버에서 수행합니다. 이번 펌웨어 수정만으로 고정 카메라가 필요 없어지거나 드론 자율비행이 구현되는 것은 아닙니다. 기존 카메라 보정, 태그 장착 오프셋, 충돌 감시 및 센서 검증이 필요합니다. 측정/감독 단계를 건너뛰고 네 대를 동시에 경기장에 넣지 않습니다.

## 10. 정지와 고장 시 확인

- 부팅/arm/stop/disarm/estop 시 모터·서보 펄스를 끕니다. 부팅 중 자동으로 서보 중앙 각도로 움직이지 않습니다.
- H1 정상 command의 0은 중립 펄스이고, 통신 만료/정지의 출력 해제는 **펄스 없음**입니다. 둘은 물리적으로 같다고 보장하지 않습니다.
- 유효기간은 보드가 제어 허가를 발급한 시각부터 최대 250 ms입니다. 통신이 끊기거나 명령이 너무 늦으면 출력을 해제합니다. 관성 이동거리나 서보 내부 정지 반응까지 즉시 0이 되는 것은 아닙니다.
- estop은 재부팅 전까지 유지됩니다. 재부팅은 안전 확인 없이 운전을 재개하는 방법이 아닙니다. 집게 서보 펄스가 꺼지면 하중이 떨어질 수 있습니다.
- `hardware_enabled:false`라면 두 출력 허가 값과 다시 업로드했는지 확인합니다. 서버 확인 항목을 무작정 true로 바꾸지 않습니다.
- Wi-Fi가 안 붙으면 2.4 GHz, SSID/암호, 공유기 격리, 실제 IP를 확인합니다. 토큰 오류를 해결하려고 인증 코드를 제거하지 않습니다.
- 갑자기 보드가 재부팅되면 모터/서보 부하로 전압이 내려가는지 확인합니다. 서버 코드의 시간 지연만 늘려 해결했다고 판단하지 않습니다.

배포 중 컴파일/자동 테스트 성공과 실제 시험은 다릅니다. **실제 보드 업로드, 전압·핀·서보 끝점 측정, 실물 네 대 주행, 집기·투하, 대회 완주는 아직 사용자의 조립 환경에서 검증해야 합니다.** 자세한 개발 설명은 [펌웨어 README](../firmware/README.md), 통신 필드는 [PROTOCOL.md](../firmware/PROTOCOL.md)를 참고하세요.
