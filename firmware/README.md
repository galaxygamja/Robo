# Robo Arduino C++ 펌웨어 2.0.0

**Arduino IDE로 올리려면 [처음부터 따라 하는 한국어 안내서](../docs/ARDUINO_START_HERE_KO.md)를 먼저 읽으세요.** 열 파일은 `arduino/RoboRobot/RoboRobot.ino`입니다. 이 `firmware/` 폴더는 같은 제어 코드의 PlatformIO 개발·검증용 원본입니다.

대상 보드는 저장소에서 사용한 **ESP32-C3 SuperMini**입니다. Arduino IDE를 쓴다는 뜻이지 Arduino Uno/Nano 보드용이라는 뜻이 아닙니다. 실제 구매한 칩과 GPIO 인쇄를 확인하세요. ESP32 일반형/S3/Uno에 이 핀맵을 그대로 쓰면 안 됩니다.

## 로봇별 구동 방식

| 실물 이름 | 프로필 | 통신 ID | 바퀴 구동 | 작업용 서보 |
|---|---:|---|---|---|
| 햄스터 H1 | 1 | H1 | SM-S4303R 연속회전 서보 2개 | MG90S 디스크 게이트 1개 |
| 비버 B1 | 2 | B1 | N20 DC 모터 2개 + DRV8833 | MG90S 집게·투하부 2개 |
| 비버 B2 | 3 | B2 | N20 DC 모터 2개 + DRV8833 | MG90S 집게·투하부 2개 |
| 비버 B3 | 4 | H2 | N20 DC 모터 2개 + DRV8833 | MG90S 집게·투하부 2개 |

B3의 통신 ID `H2`는 기존 서버/AprilTag 연결을 보존한 것입니다. 두 번째 햄스터가 아닙니다. **H1에는 DRV8833를 거쳐 바퀴 서보를 연결하지 않습니다.** 이전 모든 로봇을 DC 모터라고 기술한 안내보다 이 표가 우선합니다.

## GPIO와 전원

| GPIO | 햄스터 H1 | 비버 B1/B2/B3 |
|---:|---|---|
| 3 | 왼쪽 SM-S4303R 신호 | DRV8833 AIN1 |
| 4 | 미사용 | DRV8833 AIN2 |
| 5 | 오른쪽 SM-S4303R 신호 | DRV8833 BIN1 |
| 6 | 미사용 | DRV8833 BIN2 |
| 7 | MG90S 디스크 게이트 신호 | MG90S 앞 집게 신호 |
| 10 | 미사용 | MG90S 뒤 투하 게이트 신호 |
| 0 | TCRT5000 디지털 D0 | 미사용 |

모든 GND를 공통으로 연결합니다. 2S LiPo는 완충 8.4 V이며 보드 GPIO·5 V 서보·6 V DC 모터에 직결하지 않습니다. 서보 전원은 보드 3.3 V 핀이 아닌 충분한 전류의 별도 조절 전원에서 공급합니다. 모터/서보 전원은 실물 부품 허용 범위와 부하 시 전압을 확인하고, 전원 모듈은 2S 배터리의 실제 사용 범위 전체에서 검증하세요. USB와 외부 보드 전원을 동시에 연결할 때는 구매한 SuperMini의 역급전 보호를 확인해야 합니다.

GPIO는 3.3 V 기준입니다. 5 V 출력 TCRT5000 D0를 직접 연결하지 않습니다. DRV8833 AOUT/BOUT은 각각 좌우 DC 모터에 연결하며, `nSLEEP/SLP`는 구매 모듈 회로에 맞춰 활성화합니다. 펌웨어는 별도의 nSLEEP 제어를 가정하지 않습니다. GPIO18/19(USB), 2/8/9(부트 관련)는 이 핀맵에서 사용하지 않습니다.

## 두 가지 빌드 경로

### Arduino IDE — 권장 시작점

1. 저장소 전체를 다운로드하고 `arduino/RoboRobot/RoboRobot.ino`를 엽니다. `.ino`만 따로 복사하지 않습니다.
2. 보드 관리자에서 **esp32 by Espressif Systems 2.0.17**, 라이브러리 관리자에서 **ArduinoJson 6.21.5**를 설치합니다.
3. 보드는 **ESP32C3 Dev Module**, USB CDC On Boot는 Enabled, Flash Mode는 DIO로 선택합니다. 실제 보드의 플래시 용량을 확인합니다.
4. 옆 탭 `RobotSettings.h`에서 `ROBOT_PROFILE`을 설정합니다. `Secrets.example.h`를 같은 폴더의 `Secrets.h`로 복사하여 Wi-Fi와 토큰을 적습니다. 최초에는 `HARDWARE_OUTPUT_ENABLED 0`, `ACTUATOR_CALIBRATION_CONFIRMED 0`을 유지합니다.
5. 실제 출력 전원은 분리한 상태로 로봇 한 대만 USB로 연결하고 검증·업로드합니다. 시리얼 모니터는 115200입니다.

이 코드는 ESP32 Arduino 코어 2.0.17의 LEDC API를 사용합니다. 3.x로 자동 변경하지 말고, 변경할 경우 별도로 이식·재검증하세요. Uno용 일반 Servo 라이브러리는 필요하지 않습니다.

### PlatformIO — 같은 원본의 개발용 빌드

```powershell
python -m pip install platformio==6.1.18
Copy-Item firmware/include/secrets.example.h firmware/include/secrets.h
python -m platformio run --project-dir firmware
```

실제 Wi-Fi/토큰은 Git 제외 파일 `firmware/include/secrets.h`에 적습니다. 환경 H1/B1/B2/B3가 각 로봇을 선택하며 H2는 B3와 같은 구동 코드의 이전 이름 별칭입니다. 이 경로에서는 환경 ID와 충돌하는 프로필을 추가하지 않습니다. `platformio.ini`가 Arduino ESP32 2.0.17, ArduinoJson 6.21.5를 고정합니다. 한 대만 빌드하려면 `-e H1` 등을 붙입니다.

사용자가 실제 업로드할 때의 명령 예시입니다. COM5는 실제 포트로 바꾸고, 로봇 ID를 한 대씩 확인합니다. 저장소 배포 자체는 보드에 업로드하지 않습니다.

```powershell
python -m platformio run --project-dir firmware -e H1 -t upload --upload-port COM5
python -m platformio device monitor --project-dir firmware --port COM5 --baud 115200
```

Arduino IDE의 `RobotSettings.h`와 PlatformIO의 `secrets.h`는 서로 다른 개인 설정입니다. 한 경로에서 편집한 값이 다른 경로에 자동 반영되지 않습니다.

## 출력 허가와 보정

실제 출력에는 `HARDWARE_OUTPUT_ENABLED 1`과 `ACTUATOR_CALIBRATION_CONFIRMED 1`이 **둘 다** 필요합니다. 둘 중 하나라도 0이면 모든 물리 출력은 꺼집니다. 두 번째 값은 시험자가 배선·안전한 시험 펄스 범위·구동 방식 확인을 명시하는 표시이지, 펌웨어가 자동 측정했다는 증거가 아닙니다. 켜기 전에 안내서의 한 대씩 띄운 바퀴 시험 절차와 독립적인 전원 차단 수단을 준비하세요.

H1 바퀴 서보는 각도를 보내는 서보가 아닙니다. `DRIVE_LEFT_NEUTRAL_US`, `DRIVE_RIGHT_NEUTRAL_US` 부근을 정지점으로 하여 펄스의 방향과 차이로 회전 방향/속도를 조절합니다. 기본 1500 µs와 범위 1380~1620 µs는 **실측되지 않은 초기 설정**입니다. 개별 제품에서 중립 미끄러짐이나 회전이 생길 수 있으므로 직접 확인합니다. `DRIVE_LEFT_DIRECTION`, `DRIVE_RIGHT_DIRECTION`은 각각 +1 또는 -1이며, 양수 명령이 실제 전진이 되도록 확인합니다.

`left_pwm/right_pwm`라는 통신 이름은 유지하지만 이제 공통 **부호 있는 구동 명령값**입니다. 기본 범위는 -96~96입니다. H1에서는 중립 기준 서보 펄스로, 비버에서는 DC 모터 PWM으로 변환합니다. 96 mm/s도 96%도 아니며 H1의 값은 DC 듀티가 아닙니다. 바닥 부하에서 네 방향 속도 곡선을 새로 실측해야 합니다.

작업용 MG90S의 `TOOL0_MIN_US/MAX_US`, `TOOL1_MIN_US/MAX_US` 기본 900~2100 µs도 실제 조립 기구가 허용한다는 보장이 없습니다. 링크를 분리한 시험과 간섭 확인으로 범위를 좁힙니다. `servo_us`는 작업용 두 채널이며 H1 바퀴 서보 두 개를 여기에 넣지 않습니다. H1의 두 번째 작업 채널은 항상 0입니다.

## 정지의 의미와 검증 한계

- 부팅·arm·disarm·stop·estop에서는 모터/서보 펄스를 끕니다. 기동만으로 중앙 각도로 움직이지 않습니다.
- 정상 H1 command의 구동 값 0은 보정한 중립 펄스입니다. 반면 disarm/통신 만료의 0 출력은 **펄스 없음**입니다. 그 둘의 물리 동작은 같다고 보장하지 않습니다.
- TTL은 보드가 permit을 발급한 시각부터 최대 250 ms입니다. 늦게 도착한 패킷으로 유효시간을 새로 시작하지 않습니다. 별도 안전 태스크가 만료를 감시합니다.
- 통신 단절/기한 만료/현재 제어자의 잘못된 명령은 출력 해제와 disarm을 유발합니다. 모터의 관성, 서보 내부 동작, 집게 하중 유지까지 소프트웨어만으로 보장하지 않습니다.
- estop은 재부팅 전까지 유지합니다. 하중이 떨어질 수 있으므로 안전한 위치에서 시험하고, 손으로 조작 가능한 별도 전원 차단 수단을 둡니다.
- 컴파일·C++ 단위 시험·서버 테스트는 실제 업로드, 회전 방향, 기구 끝점, 실물 네 대 무선 주행, 대회 완주를 증명하지 않습니다.

통신 상세는 [PROTOCOL.md](PROTOCOL.md)입니다. 이번 수정 범위는 Arduino 보드 펌웨어이며 기존 Python 서버는 변경하지 않습니다. 카메라·AprilTag·색 검출·경로 계획은 여전히 서버에서 수행합니다. ESP32에 OpenCV나 전체 서버를 올리지 않습니다. 드론 자동 비행이나 움직이는 드론 카메라의 실물 운전 검증이 추가된 배포도 아닙니다. 기존 서버는 새 구동 방식 메타데이터를 자동 대조하지 않으므로 사람이 보드의 부팅 로그와 실물을 대조해야 합니다.
